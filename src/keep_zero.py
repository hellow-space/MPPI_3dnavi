#!/usr/bin/env python3
# 这一行表示：用系统环境里的 python3 来运行这个脚本

import math
import sys
# sys 用来在程序出错时退出程序

import traceback
# traceback 用来打印详细的报错信息，方便你看哪里错了

import rclpy
# rclpy 是 ROS2 的 Python 客户端库

from rclpy.node import Node
from rclpy.clock import Clock, ClockType
# Node 是 ROS2 节点的基类，写 ROS2 Python 节点一般都要继承它

from geometry_msgs.msg import Twist
# Twist 是 ROS2 里常用的速度消息类型
# 里面包括：
# linear.x  前后速度
# linear.y  左右速度
# angular.z 旋转速度


class KeepAliveZero(Node):
    # 定义一个 ROS2 节点类，名字叫 KeepAliveZero

    def __init__(self):
        # 初始化函数，节点一启动就会执行这里

        super().__init__('keep_alive_zero')
        # 初始化 ROS2 节点
        # 节点名字叫 /keep_alive_zero

        try:
            # try 里面放启动代码
            # 如果这里面出错，就会跳到 except，打印启动失败

            self.sub = self.create_subscription(
                Twist,
                '/command/cmd_twist_smoothed',
                self.cmd_callback,
                1
            )
            # 创建一个订阅者
            # 只订阅平滑后的速度，避免原始 MPPI 指令绕过平滑器
            # 消息类型是 Twist
            # 收到消息以后，会自动调用 self.cmd_callback 这个函数
            # 深度 1：只保留最新速度，不积压旧指令

            self.pub = self.create_publisher(
                Twist,
                '/cmd_vel',
                1
            )
            # 创建一个发布者
            # 发布的话题是 /cmd_vel
            # 消息类型也是 Twist
            # 也就是说，这个节点可以往 /cmd_vel 发速度

            # 超时判断和定时器都使用单调时钟，不受 ROS / 系统时间跳变影响。
            self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.last_msg_time = self.watchdog_clock.now()
            self.timer = self.create_timer(
                0.05, self.timer_callback, clock=self.watchdog_clock)
            # 创建一个定时器
            # 每 0.05 秒执行一次 self.timer_callback
            # 0.05 秒 = 20Hz
            # 也就是一秒执行 20 次

            self.get_logger().info("keep_alive_zero 启动成功")
            self.get_logger().info("订阅: /command/cmd_twist_smoothed")
            self.get_logger().info("发布: /cmd_vel")
            # 在终端打印日志，告诉你节点启动成功

        except Exception as e:
            # 如果 try 里面任何地方出错，就会进入这里

            self.get_logger().error(f"keep_alive_zero 启动失败: {e}")
            # 打印启动失败原因

            raise
            # 把错误继续抛出去，让外层 main 也能捕获到

    def cmd_callback(self, msg):
        # 这个函数会在收到平滑后的速度消息时自动执行
        # msg 就是收到的速度指令

        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(
                '忽略包含 NaN/Inf 的速度指令；不延长补零超时',
                throttle_duration_sec=1.0)
            return

        self.last_msg_time = self.watchdog_clock.now()
        # 更新“最后一次收到速度指令”的时间

        self.pub.publish(msg)
        # 把收到的速度指令原封不动转发到 /cmd_vel
        # 也就是：
        # /command/cmd_twist_smoothed  ---->  /cmd_vel

    def timer_callback(self):
        # 这个函数由定时器触发
        # 每 0.05 秒执行一次

        now = self.watchdog_clock.now()
        # 获取当前单调时间

        if (now - self.last_msg_time).nanoseconds > 0.3e9:
            # 判断距离上一次收到速度指令是否超过 0.3 秒
            # 0.3e9 纳秒 = 0.3 秒
            #
            # 如果超过 0.3 秒没收到有效的平滑速度，
            # 就认为平滑器的输出已经中断，
            # 这时候主动发零速度，防止底盘继续动

            zero_msg = Twist()
            # 创建一个空的 Twist 消息
            # 默认所有速度都是 0：
            # linear.x = 0
            # linear.y = 0
            # angular.z = 0

            self.pub.publish(zero_msg)
            # 发布零速度到 /cmd_vel
            # 让机器人停止


def main(args=None):
    # Python 程序入口函数

    try:
        # 用 try 包起来，方便捕获运行错误

        rclpy.init(args=args)
        # 初始化 ROS2 Python 系统
        # 所有 ROS2 Python 节点启动前都要先执行这个

        node = KeepAliveZero()
        # 创建 KeepAliveZero 节点对象
        # 创建时会自动执行 __init__()

        print("[OK] keep_zero.py 运行成功，节点已启动")
        # 如果能走到这里，说明节点创建成功

        rclpy.spin(node)
        # 让节点一直运行
        # 没有这一句，程序会直接退出
        #
        # spin 会一直等待：
        # 1. 订阅消息
        # 2. 定时器触发
        # 3. 其他 ROS 回调

    except KeyboardInterrupt:
        # 如果你按 Ctrl + C，就会进入这里

        print("[INFO] 手动退出 keep_zero.py")

    except Exception:
        # 如果程序运行过程中出现其他错误，就会进入这里

        print("[ERROR] keep_zero.py 运行失败")
        traceback.print_exc()
        # 打印完整错误信息，方便排查

        sys.exit(1)
        # 用错误码 1 退出程序，表示运行失败

    finally:
        # finally 里面的代码无论成功、失败、Ctrl+C 都会执行
        # 用来做清理工作

        try:
            node.destroy_node()
            # 销毁 ROS2 节点
        except Exception:
            pass
            # 如果 node 还没创建成功，这里可能会报错，忽略即可

        try:
            rclpy.shutdown()
            # 关闭 ROS2 Python 系统
        except Exception:
            pass
            # 如果 ROS2 已经关闭了，这里可能会报错，忽略即可


if __name__ == '__main__':
    # 只有直接运行这个 Python 文件时，才会执行 main()
    # 如果被别的文件 import，就不会自动执行

    main()
