import rclpy
from rclpy.node import Node
from moveit_commander.robot_trajectory import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
import serial


class TrajectorySender(Node):
    def __init__(self):
        super().__init__('trajectory_sender')

        # 串口初始化，假设 STM32 通过 /dev/ttyUSB0 串口连接
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

        # 创建订阅器，监听 MoveIt 的轨迹
        self.trajectory_subscriber = self.create_subscription(
            JointTrajectory,
            '/move_group/follow_joint_trajectory/feedback',
            self.trajectory_callback,
            10
        )

    def trajectory_callback(self, msg):
        # 解析轨迹数据
        trajectory = msg
        joint_angles = trajectory.points[-1].positions  # 获取轨迹的最后一个点的角度

        # 将关节角度转换为适合 STM32 的格式（例如，将角度转为脉冲信号）
        pulse_data = self.convert_to_pulse(joint_angles)

        # 发送数据到 STM32
        self.send_to_stm32(pulse_data)

    def convert_to_pulse(self, joint_angles):
        # 将关节角度转换为 STM32 可理解的脉冲信号
        # 假设角度范围是 0 到 180 度，转换为脉冲范围 500 到 2500
        pulse_data = []
        for angle in joint_angles:
            pulse = int(500 + (angle / 180) * 2000)
            pulse_data.append(pulse)
        return pulse_data

    def send_to_stm32(self, pulse_data):
        # 发送数据给 STM32
        data_string = ' '.join(map(str, pulse_data)) + '\r\n'
        self.serial_port.write(data_string.encode())


def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySender()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
