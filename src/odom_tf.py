#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # 1. 订阅 Gazebo 发布的 /odom 话题
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)

        # 2. 创建 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("✅ Odom TF 搬运工已启动！正在监听 /odom ...")

    def listener_callback(self, msg):
        t = TransformStamped()

        # 3. 搬运时间戳 (这很重要，保证 TF 和话题时间一致)
        t.header.stamp = msg.header.stamp

        # 4. 强制修正名字 (这是解决问题的核心！)
        # 无论 Gazebo 内部叫什么，我们这里统统改成标准的 odom -> base_link
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link' 

        # 5. 搬运位置数据
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # 6. 搬运姿态数据
        t.transform.rotation = msg.pose.pose.orientation

        # 7. 广播出去
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()