"""Gravity-constrained map alignment; no ROS dependency."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ImuSample:
    stamp_ns: int
    frame: str
    acceleration: np.ndarray
    angular_velocity: np.ndarray


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError('direction must contain three finite values')
    norm = np.linalg.norm(vector)
    if norm < 1e-9:
        raise ValueError('direction has zero magnitude')
    return vector / norm


def quat_to_rot(x, y, z, w):
    q = np.asarray([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('quaternion must be finite and nonzero')
    x, y, z, w = q / norm
    return np.array([
        [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
        [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
        [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])


def validate_transform(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError('transform must be a finite 4x4 matrix')
    r = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9) or
            not np.allclose(r.T @ r, np.eye(3), atol=1e-7) or
            not math.isclose(np.linalg.det(r), 1., abs_tol=1e-7)):
        raise ValueError('transform is not a rigid body transform')
    return matrix


def level_rotation(up):
    """Proper minimal rotation mapping camera-frame up to map +Z."""
    up = unit(up)
    v = np.cross(up, [0., 0., 1.])
    c = float(up[2])
    if c < -1 + 1e-12:
        return np.diag([1., -1., -1.])
    skew = np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
    return np.eye(3) + skew + (skew @ skew) / (1 + c)


def build_alignment(camera_base, xy, yaw, up_camera, previous=None, initial_base_height=0.):
    camera_base = validate_transform(camera_base)
    xy = np.asarray(xy, dtype=float)
    if xy.shape != (2,) or not np.all(np.isfinite(xy)) or not np.isfinite(yaw):
        raise ValueError('map position and yaw must be finite')
    up = unit(up_camera)
    if previous is None:
        r0 = level_rotation(up)
    else:
        previous = validate_transform(previous)
        established_up = previous[:3, :3].T @ [0., 0., 1.]
        if np.dot(established_up, up) < math.cos(math.radians(2.)):
            raise ValueError('gravity reference changed; stop navigation and reset alignment')
        r0 = previous[:3, :3]
    forward = (r0 @ camera_base[:3, :3])[:, 0]
    if np.linalg.norm(forward[:2]) < 1e-3:
        raise ValueError('base forward axis is vertical; map yaw is undefined')
    delta = yaw - math.atan2(forward[1], forward[0])
    c, s = math.cos(delta), math.sin(delta)
    r = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ r0
    result = np.eye(4)
    result[:3, :3] = r
    result[:2, 3] = xy - (r @ camera_base[:3, 3])[:2]
    if not np.isfinite(initial_base_height):
        raise ValueError('initial base height must be finite')
    result[2, 3] = (initial_base_height - (r @ camera_base[:3, 3])[2]
                    if previous is None else previous[2, 3])
    return result


def stationary_window(samples, reference_ns, now_ns, *, window_seconds=1., min_samples=20,
                      max_data_age=.5, pair_tolerance=.05, max_gyro=.08,
                      max_acc_cv=.03, max_direction_spread_deg=2.):
    """Return synchronized stationary IMU samples and specific-force UP in IMU frame.

    Magnitude stability is relative, supporting both g and m/s^2 driver units.
    A constant linear acceleration cannot be distinguished from gravity here;
    callers must also check TF motion and initialize while physically at rest.
    """
    if reference_ns <= 0 or now_ns <= 0 or not 0 <= now_ns-reference_ns <= max_data_age*1e9:
        raise ValueError('TF timestamp is stale, future or invalid')
    selected = [s for s in samples
                if reference_ns-window_seconds*1e9 <= s.stamp_ns <= reference_ns]
    if len(selected) < min_samples:
        raise ValueError('not enough IMU samples; remain still for the gravity window')
    stamps = [s.stamp_ns for s in selected]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('IMU timestamps must strictly increase')
    if max(b-a for a, b in zip(stamps, stamps[1:])) > min(.2, window_seconds/4)*1e9:
        raise ValueError('IMU data has a gap; wait for a continuous stationary window')
    if stamps[-1]-stamps[0] < .8*window_seconds*1e9:
        raise ValueError('IMU stationary window is too short')
    if reference_ns-stamps[-1] > pair_tolerance*1e9:
        raise ValueError('IMU and TF timestamps are too far apart')
    if now_ns-stamps[-1] > max_data_age*1e9:
        raise ValueError('raw IMU data is stale')
    if not selected[-1].frame or any(s.frame != selected[-1].frame for s in selected):
        raise ValueError('IMU frame is missing or changes in the gravity window')
    acc = np.asarray([s.acceleration for s in selected], dtype=float)
    gyro = np.asarray([s.angular_velocity for s in selected], dtype=float)
    if (acc.shape != (len(selected), 3) or gyro.shape != acc.shape or
            not np.all(np.isfinite(acc)) or not np.all(np.isfinite(gyro))):
        raise ValueError('IMU values must be finite three-dimensional vectors')
    norms = np.linalg.norm(acc, axis=1)
    if np.min(norms) < 1e-9:
        raise ValueError('IMU acceleration is zero')
    if np.max(np.linalg.norm(gyro, axis=1)) > max_gyro:
        raise ValueError('robot is rotating; remain still before initialization')
    if np.std(norms)/np.mean(norms) > max_acc_cv:
        raise ValueError('IMU acceleration is unstable; remain still')
    up = unit(np.mean(acc, axis=0))
    if np.min((acc/norms[:, None]) @ up) < math.cos(math.radians(max_direction_spread_deg)):
        raise ValueError('IMU direction is unstable; remain still')
    return selected, up
