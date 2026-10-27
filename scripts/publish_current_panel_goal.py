#!/usr/bin/env python3
"""Copy fresh panel FK into its preview-only target; never call an execution API."""
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy

def main():
    rclpy.init()
    node = rclpy.create_node('copy_current_panel_goal')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    received = []
    node.create_subscription(PoseStamped, '/manipulation/rm65b/current_pose', received.append, qos)
    pub = node.create_publisher(PoseStamped, '/manipulation/rm65b/set_goal_pose', 1)
    end = time.monotonic() + 10
    try:
        while time.monotonic() < end and (not received or pub.get_subscription_count() == 0):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not received or not pub.get_subscription_count():
            raise RuntimeError('未收到面板反馈或目标订阅，请先启动仿真与面板')
        # Panel's use_sim_time may differ from this CLI. Use the latest transform
        # and let the panel enforce fresh Gazebo joint feedback before accepting.
        msg = received[-1]
        msg.header.stamp.sec = msg.header.stamp.nanosec = 0
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)
        print('已复制当前 TCP 到面板预览目标，未执行运动。')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
