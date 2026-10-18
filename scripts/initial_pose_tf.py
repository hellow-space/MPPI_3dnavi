#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
    ])


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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.map_to_camera = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )

        # 数值保持固定，只是持续广播
        self.create_timer(0.1, self.publish_tf)

        self.get_logger().info(
            'Waiting for RViz 2D Pose Estimate...'
        )

    def initial_pose_callback(self, msg):

        try:
            # T_camera_base
            tf = self.tf_buffer.lookup_transform(
                'camera_init',
                'base_link',
                Time()
            )
        except Exception as e:
            self.get_logger().error(
                f'Cannot get camera_init -> base_link: {e}'
            )
            return

        T_camera_base = tf_to_matrix(tf)

        # RViz 点出来的 T_map_base
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        T_map_base = np.eye(4)
        T_map_base[:3, :3] = quat_to_rot(
            q.x, q.y, q.z, q.w
        )

        # base_link 起点放在 map 地面 z=0
        T_map_base[:3, 3] = [
            p.x,
            p.y,
            0.0
        ]

        # 核心：
        # T_map_camera = T_map_base * inverse(T_camera_base)
        T_map_camera = (
            T_map_base @ np.linalg.inv(T_camera_base)
        )

        self.map_to_camera = T_map_camera

        self.get_logger().info(
            f'Initial pose set: x={p.x:.3f}, y={p.y:.3f}'
        )

    def publish_tf(self):

        if self.map_to_camera is None:
            return

        T = self.map_to_camera

        qx, qy, qz, qw = rot_to_quat(T[:3, :3])

        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'camera_init'

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