"""Independent physical fixtures for map/gravity calibration."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import initial_pose_alignment as alignment


def pitch(degrees):
    a = math.radians(degrees)
    return np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])


def pose(rotation=None, xyz=(0.,0.,0.)):
    result = np.eye(4)
    result[:3,:3] = np.eye(3) if rotation is None else rotation
    result[:3,3] = xyz
    return result


def samples(acc=(0.,0.,9.81), gyro=(0.,0.,0.), scale=1.):
    return [alignment.ImuSample(int(i*1e7), 'imu', np.array(acc)*scale, np.array(gyro))
            for i in range(100,201)]


class AlignmentTest(unittest.TestCase):
    def test_body_pitch_does_not_tilt_horizontal_map(self):
        result = alignment.build_alignment(pose(pitch(10), (1.,2.,.5)),
                                           (3.,4.), 0., (0.,0.,1.))
        np.testing.assert_allclose(result[:3,:3], np.eye(3), atol=1e-12)
        mapped_body = result @ pose(pitch(10), (1.,2.,.5))
        np.testing.assert_allclose(mapped_body[:3,3], [3.,4.,0.], atol=1e-12)
        # The body remains pitched; the world floor remains horizontal.
        self.assertAlmostEqual(mapped_body[2,0], -math.sin(math.radians(10)))

    def test_tilted_camera_and_pitched_body_use_measured_up(self):
        camera_world = pitch(-34)
        body = pose(camera_world @ pitch(10), camera_world @ np.array([1.,2.,.5]))
        result = alignment.build_alignment(body, (3.,4.), 0., camera_world @ [0.,0.,1.])
        np.testing.assert_allclose(result[:3,:3], pitch(34), atol=1e-12)
        np.testing.assert_allclose((result @ body)[:3,:3], pitch(10), atol=1e-12)

    def test_reinitialization_preserves_up_and_height_reference(self):
        first = alignment.build_alignment(pose(pitch(10),(0.,0.,.5)),(1.,2.),0.,[0.,0.,1.])
        second = alignment.build_alignment(pose(pitch(-8),(2.,3.,.8)),(4.,5.),math.pi/2,
                                            [0.,0.,1.], previous=first)
        mapped = second @ pose(pitch(-8),(2.,3.,.8))
        np.testing.assert_allclose(mapped[:3,3], [4.,5.,.3], atol=1e-12)
        np.testing.assert_allclose(second[2,:], first[2,:], atol=1e-12)
        self.assertAlmostEqual(math.atan2(mapped[1,0],mapped[0,0]),math.pi/2)

    def test_explicit_initial_height_is_reference_not_ground_measurement(self):
        result = alignment.build_alignment(pose(xyz=(0.,0.,.8)),(0.,0.),0.,[0.,0.,1.],
                                           initial_base_height=.3)
        self.assertAlmostEqual((result @ [0.,0.,.8,1.])[2], .3)

    def test_invalid_and_vertical_forward_inputs_are_rejected(self):
        for up in [[0.,0.,0.],[float('nan'),0.,1.]]:
            with self.assertRaises(ValueError):
                alignment.build_alignment(pose(),(0.,0.),0.,up)
        with self.assertRaises(ValueError):
            alignment.build_alignment(pose(pitch(90)),(0.,0.),0.,[0.,0.,1.])

    def test_quaternion_validation_and_normalization(self):
        for q in [[0.,0.,0.,0.],[float('nan'),0.,0.,1.]]:
            with self.assertRaises(ValueError): alignment.quat_to_rot(*q)
        np.testing.assert_allclose(alignment.quat_to_rot(0.,0.,2.,2.),
                                   [[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]], atol=1e-12)

    def test_acceleration_units_do_not_change_up_direction(self):
        a = alignment.stationary_window(samples(), 2_000_000_000, 2_100_000_000)
        b = alignment.stationary_window(samples(scale=1/9.81), 2_000_000_000, 2_100_000_000)
        np.testing.assert_allclose(a[1], [0.,0.,1.])
        np.testing.assert_allclose(b[1], [0.,0.,1.])

    def test_raw_acceleration_is_rotated_out_of_tilted_imu(self):
        acc = pitch(-10) @ [0.,0.,9.81]
        _, up = alignment.stationary_window(samples(acc=acc), 2_000_000_000, 2_100_000_000)
        np.testing.assert_allclose(pitch(10) @ up, [0.,0.,1.], atol=1e-12)

    def test_rotating_or_insufficient_window_is_rejected(self):
        for data in [samples(gyro=(0.,0.,.2)), samples()[-10:]]:
            with self.assertRaises(ValueError):
                alignment.stationary_window(data,2_000_000_000,2_100_000_000)

    def test_stale_future_or_skewed_window_is_rejected(self):
        for ref,now in [(2_000_000_000,3_000_000_000),
                        (2_000_000_000,1_500_000_000),
                        (2_200_000_000,2_300_000_000)]:
            with self.assertRaises(ValueError): alignment.stationary_window(samples(),ref,now)

    def test_zero_noisy_or_mixed_frame_acceleration_is_rejected(self):
        noisy=samples()
        noisy[50]=alignment.ImuSample(noisy[50].stamp_ns,'imu',np.array([5.,0.,9.81]),np.zeros(3))
        mixed=samples()
        mixed[50]=alignment.ImuSample(mixed[50].stamp_ns,'other',np.array([0.,0.,9.81]),np.zeros(3))
        for data in [samples(acc=(0.,0.,0.)),noisy,mixed]:
            with self.assertRaises(ValueError):
                alignment.stationary_window(data,2_000_000_000,2_100_000_000)

    def test_opposite_up_vector_is_a_valid_rotation(self):
        result=alignment.build_alignment(pose(pitch(180)),(0.,0.),0.,[0.,0.,-1.])
        np.testing.assert_allclose(result[:3,:3] @ [0.,0.,-1.],[0.,0.,1.],atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(result[:3,:3]),1.)

    def test_large_imu_gap_does_not_count_as_continuous_stationary_window(self):
        data=samples()
        data=data[:35]+data[75:]
        with self.assertRaises(ValueError):
            alignment.stationary_window(data,2_000_000_000,2_100_000_000)


if __name__=='__main__': unittest.main()
