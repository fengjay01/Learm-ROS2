import os
from launch import LaunchDescription  # 用于创建 ROS2 启动描述
# ExecuteProcess 用于执行外部进程，RegisterEventHandler 用于事件监听
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node  # 启动 ROS2 节点
from launch_ros.substitutions import FindPackageShare  # 用于获取 ROS2 包的路径

from launch.event_handlers import OnProcessExit  # 监听某个进程退出后的回调

import xacro  # 解析 Xacro 文件

import re  # 用于处理字符串


def remove_comments(text):
    """
    这个函数用于移除 XML（URDF/Xacro 文件）中的注释，以免影响 robot_description 参数
    """
    pattern = r'<!--(.*?)-->'  # 匹配 XML 格式的注释
    return re.sub(pattern, '', text, flags=re.DOTALL)  # 使用正则表达式删除注释


def generate_launch_description():
    """
    这个函数用于生成 ROS2 的 LaunchDescription，启动 Gazebo 并加载 URDF 机器人模型
    """

    # 机器人模型和包的信息
    robot_name_in_model = 'learm'  # 机器人在 Gazebo 中的名字
    package_name = 'learm_description'  # 存放 URDF/Xacro 文件的 ROS2 包
    urdf_name = "learm.urdf"  # 机器人描述文件名称

    # 获取 URDF 文件路径
    pkg_share = FindPackageShare(package=package_name).find(
        package_name)  # 找到 mybot_description 包的路径
    urdf_model_path = os.path.join(
        pkg_share, f'urdf/{urdf_name}')  # 拼接 URDF 文件的完整路径

    # 启动 Gazebo，并加载 ROS 插件
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],  # 加载 ROS-Gazebo 兼容插件
        output='screen'  # 让终端显示 Gazebo 启动日志
    )

    # 解析 Xacro 文件（如果是 Xacro 格式，也适用于 URDF）
    xacro_file = urdf_model_path  # 机器人 URDF 文件路径
    doc = xacro.parse(open(xacro_file))  # 解析 Xacro 文件
    xacro.process_doc(doc)  # 处理 Xacro，展开其中的宏定义
    params = {'robot_description': remove_comments(
        doc.toxml())}  # 生成 robot_description 参数，并移除注释

    # 启动 robot_state_publisher（用于解析 URDF，并发布 TF 坐标变换）
    node_robot_state_publisher = Node(
        package='robot_state_publisher',  # 该节点来自 robot_state_publisher 包
        executable='robot_state_publisher',  # 可执行文件
        parameters=[
            {'use_sim_time': True},  # 使用仿真时间
            params,  # 传入 robot_description
            {"publish_frequency": 15.0}  # 以 15Hz 频率发布 TF 变换
        ],
        output='screen'  # 输出到终端
    )

    # 在 Gazebo 里生成机器人
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',  # Gazebo 里的机器人生成脚本
        arguments=['-entity', robot_name_in_model,  '-topic',
                   'robot_description'],  # 机器人名称和 URDF 来源
        output='screen'
    )

    # 加载 joint_state_broadcaster 控制器（负责发布关节状态）
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],  # ROS2 控制器加载命令
        output='screen'
    )

    # 加载 my_group_controller（负责执行关节运动）
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'my_group_controller'],  # 轨迹控制器
        output='screen'
    )

    # 监听 spawn_entity.py 进程退出，等机器人插入 Gazebo 后再加载 joint_state_broadcaster
    close_evt1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_cmd,  # 监听 spawn_entity_cmd 进程
            # 机器人生成后，加载 joint_state_broadcaster
            on_exit=[load_joint_state_controller],
        )
    )

    # 监听 joint_state_broadcaster 进程退出，等它加载完成后再加载 my_group_controller
    close_evt2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,  # 监听 joint_state_broadcaster
            on_exit=[load_joint_trajectory_controller],  # 关节状态控制器启动后，加载轨迹控制器
        )
    )

    # 创建 ROS2 LaunchDescription
    ld = LaunchDescription()

    # 按顺序添加事件监听（确保依赖关系）
    ld.add_action(close_evt1)
    ld.add_action(close_evt2)

    # 添加启动 Gazebo、robot_state_publisher、spawn_entity.py 的动作
    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)

    return ld  # 返回完整的 LaunchDescription
