#!/usr/bin/env python3

import math
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor
from initial_pose_alignment import ImuSample, build_alignment, quat_to_rot, stationary_window


def rot_to_quat(R):
    tr = np.trace(R)

    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2,1] - R[1,2]) / S
        y = (R[0,2] - R[2,0]) / S
        z = (R[1,0] - R[0,1]) / S
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        w = (R[2,1] - R[1,2]) / S
        x = 0.25 * S
        y = (R[0,1] + R[1,0]) / S
        z = (R[0,2] + R[2,0]) / S
    elif R[1,1] > R[2,2]:
        S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        w = (R[0,2] - R[2,0]) / S
        x = (R[0,1] + R[1,0]) / S
        y = 0.25 * S
        z = (R[1,2] + R[2,1]) / S
    else:
        S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        w = (R[1,0] - R[0,1]) / S
        x = (R[0,2] + R[2,0]) / S
        y = (R[1,2] + R[2,1]) / S
        z = 0.25 * S

    return x, y, z, w


def tf_to_matrix(t):
    q = t.transform.rotation
    p = t.transform.translation

    T = np.eye(4)
    T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]

    return T


class InitialPoseTF(Node):

    def __init__(self):
        super().__init__('initial_pose_tf')

        defaults = dict(map_frame='map', odom_frame='camera_init', base_frame='base_link',
                        imu_topic='/livox/imu', max_data_age=.5, initial_pose_max_age=1.,
                        window_seconds=1., min_samples=20, pair_tolerance=.05, max_gyro=.08,
                        max_acc_cv=.03, max_direction_spread_deg=2., max_translation=.02,
                        max_rotation_deg=2., initial_base_height=0.)
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
            setattr(self, name, self.get_parameter(name).value)
        for name in ('map_frame', 'odom_frame', 'base_frame', 'imu_topic'):
            if not getattr(self, name):
                raise ValueError(f'{name} must not be empty')
        for name in ('max_data_age', 'initial_pose_max_age', 'window_seconds',
                     'pair_tolerance', 'max_gyro', 'max_acc_cv',
                     'max_direction_spread_deg', 'max_translation', 'max_rotation_deg'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.min_samples < 2 or not math.isfinite(self.initial_base_height):
            raise ValueError('min_samples must be >= 2 and initial_base_height must be finite')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.map_to_camera = None
        self.imu_samples = deque(maxlen=2000)
        self.last_clock_ns = 0
        self.last_imu_stamp_ns = 0
        self.imu_rejection_reason = ''

        # Only store bounded raw samples here; gravity/TF work happens on a click.
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reset', self.reset_callback)

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )

        # 数值保持固定，只是持续广播
        self.create_timer(0.1, self.publish_tf)

        self.get_logger().info(
            f'Waiting for initial pose in {self.map_frame}; remain still for '
            f'{self.window_seconds:.1f}s with fresh {self.imu_topic} and TF. '
            'Map up comes from gravity, not the current body pitch.'
        )

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def reset_alignment(self, reason):
        self.map_to_camera = None
        self.imu_samples.clear()
        self.last_imu_stamp_ns = 0
        self.imu_rejection_reason = 'waiting for new stationary IMU samples'
        self.get_logger().warning(f'Alignment reset: {reason}; stop navigation and initialize again')

    def reset_callback(self, request, response):
        self.reset_alignment('explicit reset requested')
        response.success = True
        response.message = 'Alignment cleared; wait for fresh stationary data and send initial pose again'
        return response

    def check_clock(self):
        now = self.get_clock().now().nanoseconds
        valid = not self.last_clock_ns or now >= self.last_clock_ns
        if not valid:
            self.reset_alignment('ROS clock moved backwards')
        self.last_clock_ns = now
        return valid

    def imu_callback(self, msg):
        if not self.check_clock():
            return
        stamp = self.stamp_ns(msg.header.stamp)
        now = self.get_clock().now().nanoseconds
        if stamp <= 0 or not 0 <= now-stamp <= self.max_data_age*1e9:
            self.imu_samples.clear()
            self.imu_rejection_reason = (
                f'raw IMU age {(now-stamp)*1e-9:.2f}s is invalid; '
                f'need 0..{self.max_data_age:.2f}s (check driver timestamps and queues)')
            return
        if not msg.header.frame_id:
            self.imu_samples.clear()
            self.imu_rejection_reason = 'raw IMU frame is empty'
            return
        if stamp <= self.last_imu_stamp_ns:
            if self.last_imu_stamp_ns-stamp > 100_000_000:
                self.reset_alignment('IMU timestamp moved backwards')
            else:
                return
        acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                                 msg.linear_acceleration.z])
        angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                                     msg.angular_velocity.z])
        if not np.all(np.isfinite(acceleration)) or not np.all(np.isfinite(angular_velocity)):
            self.imu_samples.clear()
            self.imu_rejection_reason = 'raw IMU contains non-finite values'
            return
        self.imu_samples.append(ImuSample(stamp, msg.header.frame_id, acceleration, angular_velocity))
        self.last_imu_stamp_ns = stamp
        self.imu_rejection_reason = ''

    def measured_gravity_and_pose(self, now):
        if not self.imu_samples and getattr(self, 'imu_rejection_reason', ''):
            raise ValueError(self.imu_rejection_reason)
        options = {name: getattr(self, name) for name in (
            'window_seconds', 'min_samples', 'max_data_age', 'pair_tolerance',
            'max_gyro', 'max_acc_cv', 'max_direction_spread_deg')}
        # The synchronized TF may lag current IMU. Motion that started after that
        # TF is still disqualifying: check the newest raw window before pairing.
        if self.imu_samples:
            stationary_window(self.imu_samples, self.imu_samples[-1].stamp_ns, now, **options)
        latest = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        samples, up_imu = stationary_window(
            self.imu_samples, self.stamp_ns(latest.header.stamp), now,
            **options)
        reference = Time(nanoseconds=samples[-1].stamp_ns, clock_type=self.get_clock().clock_type)
        camera_base = tf_to_matrix(self.tf_buffer.lookup_transform(
            self.odom_frame, self.base_frame, reference))
        camera_imu = tf_to_matrix(self.tf_buffer.lookup_transform(
            self.odom_frame, samples[-1].frame, reference))
        # Check translation and attitude throughout the same stationary window.
        for index in np.linspace(0, len(samples)-1, 5, dtype=int):
            stamp = Time(nanoseconds=samples[index].stamp_ns, clock_type=self.get_clock().clock_type)
            earlier = tf_to_matrix(self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, stamp))
            if np.linalg.norm(earlier[:3, 3]-camera_base[:3, 3]) > self.max_translation:
                raise ValueError('robot position is changing; remain still before initialization')
            cosine = np.clip((np.trace(earlier[:3, :3].T @ camera_base[:3, :3])-1)/2, -1., 1.)
            if math.acos(cosine) > math.radians(self.max_rotation_deg):
                raise ValueError('robot attitude is changing; remain still before initialization')
        return camera_imu[:3, :3] @ up_imu, camera_base

    def initial_pose_callback(self, msg):
        if not self.check_clock():
            return
        try:
            now = self.get_clock().now().nanoseconds
            if msg.header.frame_id != self.map_frame:
                raise ValueError(f'initial pose frame must be {self.map_frame}, got {msg.header.frame_id!r}')
            age = now-self.stamp_ns(msg.header.stamp)
            if self.stamp_ns(msg.header.stamp) <= 0 or not 0 <= age <= self.initial_pose_max_age*1e9:
                raise ValueError('initial pose timestamp is stale, future or invalid')
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            if not np.all(np.isfinite([p.x, p.y, p.z])):
                raise ValueError('initial position must be finite')
            desired_rotation = quat_to_rot(q.x, q.y, q.z, q.w)
            if np.linalg.norm(desired_rotation[:2, 0]) < 1e-3:
                raise ValueError('initial orientation has no horizontal heading')
            yaw = math.atan2(desired_rotation[1, 0], desired_rotation[0, 0])
            up_camera, camera_base = self.measured_gravity_and_pose(now)
            result = build_alignment(camera_base, [p.x, p.y], yaw, up_camera,
                                     previous=self.map_to_camera,
                                     initial_base_height=self.initial_base_height)
        except Exception as e:
            self.get_logger().error(f'Initial pose rejected: {e}')
            return
        self.map_to_camera = result
        self.get_logger().info(
            f'Initial pose accepted: x={p.x:.3f}, y={p.y:.3f}, yaw={math.degrees(yaw):.1f}deg; '
            'gravity-aligned map, full body roll/pitch retained'
        )

    def publish_tf(self):

        if not self.check_clock() or self.map_to_camera is None:
            return

        T = self.map_to_camera

        qx, qy, qz, qw = rot_to_quat(T[:3, :3])

        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame

        tf.transform.translation.x = float(T[0,3])
        tf.transform.translation.y = float(T[1,3])
        tf.transform.translation.z = float(T[2,3])

        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()

    node = InitialPoseTF()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
