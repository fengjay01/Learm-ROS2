# 监听轨迹

import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient, GoalResponse, CancelResponse
from rclpy.action import ActionServer
from rclpy.qos import qos_profile_system_default


class TrajectoryMonitor(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')

        # 订阅真实轨迹的 action goal 请求（模拟一个 fake action server，只监听）
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("✅ Trajectory Monitor 已启动，监听 MoveIt 执行的轨迹...")

    def goal_callback(self, goal_request):
        self.get_logger().info("📥 收到新轨迹目标（Goal）")
        joint_names = goal_request.trajectory.joint_names
        points = goal_request.trajectory.points

        self.get_logger().info(f"Joint Names: {joint_names}")
        for i, pt in enumerate(points):
            positions = ", ".join([f"{p:.3f}" for p in pt.positions])
            self.get_logger().info(f"  点 {i+1}: {positions}")
        # 不接受 goal，因为我们只是监听
        return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        return None


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
