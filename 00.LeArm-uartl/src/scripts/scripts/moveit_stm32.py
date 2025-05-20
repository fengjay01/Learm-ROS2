# 舵机6写死90度

import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import serial
import math


class TrajectoryRelay(Node):
    def __init__(self):
        super().__init__('trajectory_relay')

        # 初始化串口（根据实际修改端口号）
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1)
            self.get_logger().info("✅ 串口初始化成功：/dev/ttyUSB0 @9600")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败：{e}")
            self.ser = None

        # 创建 Action Server，仅监听
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("📡 Trajectory Relay 节点已启动，监听并转发轨迹给 STM32")

    def goal_callback(self, goal_request):
        joint_names = goal_request.trajectory.joint_names
        points = goal_request.trajectory.points

        if not points:
            self.get_logger().warn("⚠️ 收到空轨迹点，忽略")
            return GoalResponse.REJECT

        # 取最后一个轨迹点
        final_point = points[-1]
        joint_position_map = dict(zip(joint_names, final_point.positions))

        # 预定义目标关节顺序（必须与STM32舵机顺序一致）
        expected_joint_order = [
            'Arm_Joint1', 'Arm_Joint2', 'Arm_Joint3',
            'Arm_Joint4', 'Arm_Joint5', 'Grip_Joint'
        ]

        servo_angles = []
        for joint_name in expected_joint_order:
            if joint_name in joint_position_map:
                rad = joint_position_map[joint_name]
                angle = round(90 + rad * 180 / math.pi)  # round保持精度
            else:
                angle = 90  # 默认角度
            servo_angles.append(angle)

        # 固定第6个舵机角度为90°
        servo_angles[5] = 90

        # 构建串口指令字符串
        duration_ms = 1000
        servo_count = 6
        servo_ids = list(range(1, 7))

        parts = [str(servo_count), str(duration_ms)]
        for sid, angle in zip(servo_ids, servo_angles):
            parts.append(str(sid))
            parts.append(str(angle))
        command_str = " ".join(parts) + "\n"

        # 日志输出
        self.get_logger().info("📥 收到轨迹目标（原始弧度）：")
        for i, joint_name in enumerate(expected_joint_order):
            rad = joint_position_map.get(joint_name, 0.0)
            self.get_logger().info(f"  {joint_name} (ID {i+1}): {rad:.3f} rad")

        self.get_logger().info("🎯 转换为 STM32 角度（单位 °）：")
        for i, angle in enumerate(servo_angles):
            self.get_logger().info(f"  舵机 {i+1}: {angle}°")

        self.get_logger().info(f"➡️ 发送给 STM32：{command_str.strip()}")

        # 串口发送
        if self.ser:
            try:
                self.ser.write(command_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"❌ 串口发送失败：{e}")

        return GoalResponse.REJECT  # 只监听，不处理 goal 执行

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        return None

    def destroy_node(self):
        if self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
