import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import OnProcessExit

import xacro
import re


def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)


def generate_launch_description():
    robot_name_in_model = 'learm'
    package_name = 'learm_description'
    urdf_name = "gazebo_learm.urdf.xacro"

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    learm_moveit_config = FindPackageShare(
        package='moveit_uartl').find('moveit_uartl')

    controller_config = PathJoinSubstitution(
        [learm_moveit_config, 'config', 'ros2_controllers.yaml']
    )

    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # 因为 urdf文件中有一句 $(find dummy_moveit_config) 需要用xacro进行编译一下才行
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}
    params = {'robot_description': remove_comments(doc.toxml())}

    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容
    # 并且会订阅 /joint_states 话题，获取关节的数据，然后发布tf和tf_static话题.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True},
                    params, {"publish_frequency": 15.0}],
        output='screen'
    )

    # 非必要，只是之前在修改的过程中验证的时候老是出现找不到控制器配置文件ros2_controller.yaml就这这里再加了一点，但根本原因不在于没加这一段但是懒得删了
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )

    # Launch the robot, 通过robot_description话题进行模型内容获取从而在gazebo中生成模型
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-topic', 'robot_description'], output='screen')

    # gazebo在加载urdf时，根据urdf的设定，会启动一个joint_states节点
    # 关节状态发布器
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # 路径执行控制器，这个控制器名字不一样的要注意进行更换哦，控制器名字可以在类似ros2_controllers.yaml的文件里找到
    load_joint_trajectory_controller_arm = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    # 爪子执行控制器，这个控制器名字不一样的要注意进行更换哦，控制器名字可以在类似ros2_controllers.yaml的文件里找到
    load_joint_trajectory_controller_gripper = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'hand_controller'],
        output='screen'
    )

    # 控制好各个节点的启动顺序
    close_evt1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[load_joint_state_controller],
        )
    )
    # 监听 load_joint_state_controller,加载完成后启动arm控制器
    close_evt2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_trajectory_controller_arm],
        )
    )
    # 监听 load_joint_state_controller，加载后启动gripper控制器
    close_evt3 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_trajectory_controller_gripper],
        )
    )

    ld = LaunchDescription()

    ld.add_action(close_evt1)
    ld.add_action(close_evt2)
    ld.add_action(close_evt3)

    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(spawn_entity_cmd)

    return ld
