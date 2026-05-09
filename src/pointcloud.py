import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

class FakeLocalPointCloud(Node):
    def __init__(self):
        super().__init__('fake_local_pointcloud')
        self.pub = self.create_publisher(PointCloud2, '/points', 10)
        self.timer = self.create_timer(0.1, self.pub_cloud)
        print("✅ 发布【安全局部点云】radar_link，不会碰撞！")

    def pub_cloud(self):
        points = []

        ###########################################################
        # ✅ 关键：生成在机器人【侧面/远处】，不会贴在车身上！
        ###########################################################
        # 右侧 1.5 米，一条线
        for i in range(30):
            x = (i/30 - 0.5) * 2.0
            y = 1.5
            z = 0.3
            points.append([x, y, z])

        # 左侧 1.5 米，一条线
        for i in range(30):
            x = (i/30 - 0.5) * 2.0
            y = -1.5
            z = 0.3
            points.append([x, y, z])

        header = PointCloud2().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'radar_link'   # ✅ 必须是雷达坐标系

        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pub.publish(cloud_msg)

def main():
    rclpy.init()
    rclpy.spin(FakeLocalPointCloud())
    rclpy.shutdown()
