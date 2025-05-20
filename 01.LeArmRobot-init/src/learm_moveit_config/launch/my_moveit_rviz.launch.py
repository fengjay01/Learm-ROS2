# 引入 MoveIt 的配置工具，用于构建 MoveIt 配置
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

# 引入 ROS2 的 Launch 相关模块
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,  # 声明启动参数
    IncludeLaunchDescription,  # 包含其他启动文件
)
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,  # 添加可调试的节点
    DeclareBooleanLaunchArg,  # 声明布尔类型的启动参数
)
from launch.substitutions import LaunchConfiguration  # 允许在启动时动态获取参数值
from launch_ros.parameter_descriptions import ParameterValue  # 允许参数为空值并转换为指定类型


def generate_launch_description():
    """
    生成 MoveIt 相关的启动文件，包含 move_group 和 RViz 的启动。
    """

    # 构建 MoveIt 配置，"learm" 是机器人名称，"mybot" 是 ROS2 包名称
    moveit_config = MoveItConfigsBuilder(
        "learm", package_name="learm_moveit_config").to_moveit_configs()

    # 创建一个 LaunchDescription（启动描述）对象
    ld = LaunchDescription()

    # 启动 move_group 组件
    my_generate_move_group_launch(ld, moveit_config)
    # 启动 RViz 可视化工具
    my_generate_moveit_rviz_launch(ld, moveit_config)

    return ld


def my_generate_move_group_launch(ld, moveit_config):
    """
    启动 move_group 组件，该组件负责规划和执行运动轨迹。
    """

    # 添加布尔类型的启动参数
    ld.add_action(DeclareBooleanLaunchArg(
        "debug", default_value=False))  # 是否开启调试模式
    ld.add_action(DeclareBooleanLaunchArg(
        "allow_trajectory_execution", default_value=True))  # 是否允许轨迹执行
    ld.add_action(DeclareBooleanLaunchArg(
        "publish_monitored_planning_scene", default_value=True))  # 是否发布规划场景信息

    # 额外的可选参数：
    # capabilities：启用的 MoveGroup 额外功能
    # disable_capabilities：禁止的 MoveGroup 功能
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument(
        "disable_capabilities", default_value=""))

    # 是否监视动力学信息，默认为 False，因为大部分 MoveIt 组件不依赖此信息
    ld.add_action(DeclareBooleanLaunchArg(
        "monitor_dynamics", default_value=False))

    # 获取是否需要发布规划场景的参数值
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    # 配置 move_group 启动时的参数
    move_group_configuration = {
        "publish_robot_description_semantic": True,  # 发布机器人语义描述信息
        # 允许轨迹执行
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),

        # 允许 capabilities 和 disable_capabilities 为空字符串
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),

        # 发布真实机器人状态到 RViz，以便进行监控
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,  # 默认关闭动力学监测
    }

    # 将 MoveIt 配置和 move_group 相关的参数组合到参数列表
    move_group_params = [
        moveit_config.to_dict(),  # MoveIt 配置参数
        move_group_configuration,  # MoveGroup 配置参数
    ]
    move_group_params.append({"use_sim_time": True})  # 使用仿真时间，以确保仿真环境中的时间同步

    # 添加 move_group 可调试节点
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",  # move_group 所在的 ROS2 包
        executable="move_group",  # move_group 的可执行文件
        commands_file=str(moveit_config.package_path /
                          "launch" / "gdb_settings.gdb"),  # 调试模式下的 gdb 配置文件
        output="screen",  # 输出到终端
        parameters=move_group_params,  # 传入 move_group 的参数
        extra_debug_args=["--debug"],  # 额外的调试参数
        additional_env={"DISPLAY": ":0"},  # 设置显示环境变量，以便内部可能使用 OpenGL 渲染
    )

    return ld


def my_generate_moveit_rviz_launch(ld, moveit_config):
    """
    启动 RViz，并加载 MoveIt 配置，用于可视化和交互控制。
    """

    # 添加布尔类型的启动参数
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))

    # 添加 RViz 配置文件路径参数
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            # 默认使用 MoveIt 相关的 RViz 配置文件
            default_value=str(moveit_config.package_path /
                              "config/moveit.rviz"),
        )
    )

    # RViz 需要的参数
    rviz_parameters = [
        moveit_config.planning_pipelines,  # 规划管道配置
        moveit_config.robot_description_kinematics,  # 机器人运动学配置
    ]
    rviz_parameters.append({"use_sim_time": True})  # 使用仿真时间

    # 添加可调试的 RViz2 节点
    add_debuggable_node(
        ld,
        package="rviz2",  # RViz2 相关的 ROS2 包
        executable="rviz2",  # RViz2 可执行文件
        output="log",  # 以日志形式输出
        respawn=False,  # 若进程崩溃，则不重新启动
        arguments=["-d", LaunchConfiguration("rviz_config")],  # 加载 RViz 配置文件
        parameters=rviz_parameters,  # 传入参数
    )

    return ld
