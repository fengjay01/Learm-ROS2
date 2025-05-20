# 话题发布数据
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class TrajectoryPrinter(Node):
    def __init__(self):
        super().__init__('trajectory_printer')

        # 订阅 MoveIt 执行的轨迹话题
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',  # ✅ 正确的轨迹话题
            self.trajectory_callback,
            10
        )
        self.get_logger().info("🟢 已启动，等待来自 MoveIt 的轨迹...")

    def trajectory_callback(self, msg):
        self.get_logger().info(f"📥 收到轨迹，共 {len(msg.points)} 个点")
        joint_names = msg.joint_names
        self.get_logger().info(f"关节名称: {joint_names}")

        for idx, point in enumerate(msg.points):
            positions = ", ".join(f"{p:.4f}" for p in point.positions)
            self.get_logger().info(f"  点 {idx + 1}: {positions}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
