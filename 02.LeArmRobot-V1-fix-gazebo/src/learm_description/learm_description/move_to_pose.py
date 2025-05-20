#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys


class MoveToPoseIKNode(Node):
    def __init__(self):
        super().__init__('move_to_pose_ik_node')

        # 创建 IK 服务客户端
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')

        # 创建轨迹控制器的动作客户端
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/learm_group_controller/follow_joint_trajectory')
        self.joint_names = ['Arm_Joint1', 'Arm_Joint2', 'Arm_Joint3',
                            'Arm_Joint4', 'Arm_Joint5', 'Grip_Joint']

    def move_to_pose(self, x, y, z):
        # 构造目标位姿
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.w = 1.0  # 简单设置单位四元数

        # 构造请求
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'learm_group'
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.timeout.sec = 1

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error("IK service call failed")
            return

        result = future.result()
        if not result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().error(
                f"IK failed with code: {result.error_code.val}")
            return

        # 解析结果：生成一个轨迹点
        joint_state = result.solution.joint_state
        positions = joint_state.position
        names = joint_state.name

        joint_positions = [0.0] * len(self.joint_names)
        for i, name in enumerate(self.joint_names):
            if name in names:
                index = names.index(name)
                joint_positions[i] = positions[index]

        # 构造轨迹消息
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 2
        traj.points.append(point)

        # 发送轨迹
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(
            f"Sending trajectory to reach ({x}, {y}, {z})...")
        self.action_client.wait_for_server()
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        if send_future.result().accepted:
            self.get_logger().info("Trajectory execution accepted.")
        else:
            self.get_logger().error("Trajectory execution rejected.")


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseIKNode()

    if len(sys.argv) < 4:
        node.get_logger().error("Usage: ros2 run your_package move_to_pose_ik.py x y z")
        rclpy.shutdown()
        return

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        node.move_to_pose(x, y, z)
        rclpy.spin_once(node, timeout_sec=5.0)
    except ValueError:
        node.get_logger().error("Invalid input coordinates")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
