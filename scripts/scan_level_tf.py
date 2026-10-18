#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


class ScanLevelTF(Node):

    def __init__(self):
        super().__init__('scan_level_tf')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.publish_scan_level_tf
        )

    def publish_scan_level_tf(self):

        try:
            # 查询 line_frame 当前在 map 下的位置 + 姿态
            tf = self.tf_buffer.lookup_transform(
                'map',
                'line_frame',
                Time()
            )

        except Exception:
            return

        q = tf.transform.rotation

        # 只提取 yaw
        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        out = TransformStamped()

        out.header.stamp = self.get_clock().now().to_msg()

        out.header.frame_id = 'map'
        out.child_frame_id = 'scan_level_frame'

        # =============================
        # 位置：
        # 完全使用 line_frame 当前的位置
        # =============================

        out.transform.translation.x = \
            tf.transform.translation.x

        out.transform.translation.y = \
            tf.transform.translation.y

        out.transform.translation.z = \
            tf.transform.translation.z

        # =============================
        # 姿态：
        # roll  = 0
        # pitch = 0
        # yaw   = line_frame yaw
        # =============================

        out.transform.rotation.x = 0.0
        out.transform.rotation.y = 0.0

        out.transform.rotation.z = \
            math.sin(yaw / 2.0)

        out.transform.rotation.w = \
            math.cos(yaw / 2.0)

        self.tf_broadcaster.sendTransform(out)


def main():

    rclpy.init()

    node = ScanLevelTF()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
