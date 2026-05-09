#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeepAliveZero(Node):
    def __init__(self):
        super().__init__('keep_alive_zero')
        # 订阅导航实际发指令的话题（已经被 remap 到 /command/cmd_twist）
        self.sub = self.create_subscription(Twist, '/command/cmd_twist', self.cmd_callback, 10)
        # 发布到底盘监听的 /cmd_vel
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 定时器，以 20Hz 持续运行
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.last_msg_time = self.get_clock().now()
        self.has_command = False

    def cmd_callback(self, msg):
        # 收到任何指令，更新时间戳，并且直接转发（可选）
        self.last_msg_time = self.get_clock().now()
        self.has_command = True
        # 如果需要保持平滑器功能，可以不转发，让 smoother 继续工作
        # 但这里为了保险，直接转发原始指令（或稍作处理）
        self.pub.publish(msg)

    def timer_callback(self):
        now = self.get_clock().now()
        # 如果超过 0.5 秒没收到新指令，则持续发布零速
        if (now - self.last_msg_time).nanoseconds > 0.3e9:
            zero_msg = Twist()
            self.pub.publish(zero_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeepAliveZero()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()