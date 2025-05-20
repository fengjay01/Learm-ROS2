import os  # 导入os模块，用于与操作系统进行交互

# 从ament_index_python包导入获取包共享目录的函数
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription  # 从launch包导入LaunchDescription类，表示一个启动描述
# 从actions模块导入声明启动参数、执行进程和包含启动描述的动作
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition  # 从conditions模块导入IfCondition类，用于条件执行
# 从launch_description_sources模块导入PythonLaunchDescriptionSource类
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 从substitutions模块导入LaunchConfiguration和PythonExpression类
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node  # 从launch_ros.actions模块导入Node类，表示一个ROS节点的启动


def generate_launch_description():
    # 获取启动目录
    bringup_dir = get_package_share_directory(
        'dofbot_description')  # 获取test_urdf包的共享目录
    launch_dir = os.path.join(bringup_dir, 'launch')  # 拼接得到launch目录的路径

    # 启动时特定于仿真的配置变量
    rviz_config_file = LaunchConfiguration(
        'rviz_config_file')  # 创建一个用于RVIZ配置文件的启动配置变量
    use_robot_state_pub = LaunchConfiguration(
        'use_robot_state_pub')  # 创建一个用于是否使用机器人状态发布器的启动配置变量
    use_joint_state_pub = LaunchConfiguration(
        'use_joint_state_pub')  # 创建一个用于是否使用关节状态发布器的启动配置变量
    use_rviz = LaunchConfiguration('use_rviz')  # 创建一个用于是否使用RVIZ的启动配置变量
    urdf_file = LaunchConfiguration('urdf_file')  # 创建一个用于URDF文件路径的启动配置变量

    # 声明RVIZ配置文件的启动参数
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',  # 参数名称
        default_value=os.path.join(
            bringup_dir, 'rviz', 'view.rviz'),  # 默认值为RVIZ配置文件的路径
        description='Full path to the RVIZ config file to use'  # 参数描述
    )

    # 声明是否使用机器人状态发布器的启动参数
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',  # 默认值为True
        description='Whether to start the robot state publisher'  # 参数描述
    )

    # 声明是否使用关节状态发布器的启动参数
    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='True',  # 默认值为True
        description='Whether to start the joint state publisher'  # 参数描述
    )

    # 声明是否使用RVIZ的启动参数
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',  # 默认值为True
        description='Whether to start RVIZ'  # 参数描述
    )

    # 声明URDF文件的启动参数
    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(
            bringup_dir, 'urdf', 'dofbot.urdf'),  # 默认值为URDF文件的路径
        description='Whether to start RVIZ'  # 参数描述（这里描述有误，可能应该是URDF文件描述）
    )

    # 创建机器人状态发布器节点的命令
    start_robot_state_publisher_cmd = Node(
        # 仅在use_robot_state_pub为True时启动
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',  # 指定使用的包
        executable='robot_state_publisher',  # 指定可执行文件
        name='robot_state_publisher',  # 节点名称
        output='screen',  # 将输出到屏幕
        # parameters=[{'use_sim_time': use_sim_time}],  # 可选的参数（注释掉了）
        arguments=[urdf_file]  # 将URDF文件作为参数传递
    )

    # 创建关节状态发布器节点的命令
    start_joint_state_publisher_cmd = Node(
        # 仅在use_joint_state_pub为True时启动
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher_gui',  # 指定使用的包
        executable='joint_state_publisher_gui',  # 指定可执行文件
        name='joint_state_publisher_gui',  # 节点名称
        output='screen',  # 将输出到屏幕
        arguments=[urdf_file]  # 将URDF文件作为参数传递
    )

    # 创建RVIZ节点的命令
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),  # 仅在use_rviz为True时启动
        package='rviz2',  # 指定使用的包
        executable='rviz2',  # 指定可执行文件
        name='rviz2',  # 节点名称
        arguments=['-d', rviz_config_file],  # 传递RVIZ配置文件参数
        output='screen'  # 将输出到屏幕
    )

    # 创建启动描述并填充
    ld = LaunchDescription()  # 新建一个启动描述对象

    # 声明启动选项
    ld.add_action(declare_rviz_config_file_cmd)  # 添加RVIZ配置文件声明
    ld.add_action(declare_urdf_cmd)  # 添加URDF声明
    ld.add_action(declare_use_robot_state_pub_cmd)  # 添加机器人状态发布器声明
    ld.add_action(declare_use_joint_state_pub_cmd)  # 添加关节状态发布器声明
    ld.add_action(declare_use_rviz_cmd)  # 添加RVIZ声明

    # 添加带条件的动作
    ld.add_action(start_joint_state_publisher_cmd)  # 添加关节状态发布器启动命令
    ld.add_action(start_robot_state_publisher_cmd)  # 添加机器人状态发布器启动命令
    ld.add_action(rviz_cmd)  # 添加RVIZ启动命令

    return ld  # 返回启动描述对象
