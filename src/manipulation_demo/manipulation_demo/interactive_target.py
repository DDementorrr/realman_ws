import math
import threading

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class InteractiveTargetPublisher(Node):
    def __init__(self):
        super().__init__("interactive_target_publisher")
        self.declare_parameter("topic", "/manipulation/rm65b/target_pose")
        self.declare_parameter("frame_id", "base_link")
        self.publisher = self.create_publisher(
            PoseStamped, self.get_parameter("topic").value, 10
        )

    def publish_values(self, values):
        x, y, z, roll, pitch, yaw = values
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published target xyz=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f})"
        )


def console_loop(node):
    prompt = "target> x y z roll pitch yaw (radians), or q: "
    while rclpy.ok():
        try:
            line = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line.lower() in {"q", "quit", "exit"}:
            break
        try:
            values = [float(item) for item in line.split()]
            if len(values) != 6:
                raise ValueError("expected six values")
            node.publish_values(values)
        except ValueError as exc:
            node.get_logger().error(f"Invalid input: {exc}")
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveTargetPublisher()
    thread = threading.Thread(target=console_loop, args=(node,), daemon=True)
    thread.start()
    rclpy.spin(node)
    thread.join(timeout=1.0)
    node.destroy_node()


if __name__ == "__main__":
    main()
