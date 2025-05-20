# 提取的的数据与监听一致（6号也是固定，但不发送给stm32）

import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import serial
import math


class TrajectoryRelay(Node):
    def __init__(self):
        super().__init__('trajectory_relay')

        # 初始化串口（根据实际设备路径修改）
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1)
            self.get_logger().info("✅ 串口初始化成功：/dev/ttyUSB0 @9600")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败：{e}")
            self.ser = None

        # 创建 Action Server：监听来自 MoveIt 的轨迹
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

        # 只取最后一个轨迹点（目标点）
        final_point = points[-1]
        positions = final_point.positions

        # 角度转换：STM32角度 = 90 + 弧度 × 180 / π
        servo_angles = [int(90 + pos * 180 / math.pi) for pos in positions]

        # 固定第6个舵机角度为90°
        if len(servo_angles) >= 6:
            servo_angles[5] = 90

        duration_ms = 1000
        servo_count = len(servo_angles)
        servo_ids = list(range(1, servo_count + 1))

        # 构造发送字符串
        parts = [str(servo_count), str(duration_ms)]
        for sid, angle in zip(servo_ids, servo_angles):
            parts.append(str(sid))
            parts.append(str(angle))
        command_str = " ".join(parts) + "\n"

        # 控制台输出
        self.get_logger().info("📥 收到轨迹目标（原始弧度）：")
        for i, (name, rad) in enumerate(zip(joint_names, positions)):
            self.get_logger().info(f"  {name} (ID {i+1}): {rad:.3f} rad")

        self.get_logger().info("🎯 转换为 STM32 角度（单位 °）：")
        for i, angle in enumerate(servo_angles):
            self.get_logger().info(f"  舵机 {i+1}: {angle}°")

        self.get_logger().info(f"➡️ 发送给 STM32：{command_str.strip()}")

        # 发送串口指令
        if self.ser:
            try:
                self.ser.write(command_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"❌ 串口发送失败：{e}")

        return GoalResponse.REJECT  # 拒绝执行，只监听

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        # 我们不执行轨迹，只监听
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
