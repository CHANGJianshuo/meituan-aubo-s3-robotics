"""启动 mtc_moveit_planner 的离线规划工具（只规划，不执行）。

前置：Fake Hardware 控制 launch 与 aubo_moveit.launch.py（move_group）已在同一 ROS 域运行。
本 launch 只给工具节点注入与 move_group 相同的 MoveIt 配置（URDF/SRDF/kinematics），
不启动 controller_manager / move_group，也不连接任何真实设备。

用法：
  ros2 launch mtc_moveit_planner plan_tool.launch.py program:=plan_fixed_pose
  ros2 launch mtc_moveit_planner plan_tool.launch.py program:=plan_battery_sequence
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    program = LaunchConfiguration("program").perform(context)
    aubo_type = LaunchConfiguration("aubo_type").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    output_dir = LaunchConfiguration("output_dir").perform(context)
    extra_params_file = LaunchConfiguration("extra_params_file").perform(context)
    if program not in ("plan_fixed_pose", "plan_battery_sequence"):
        raise RuntimeError(f"unknown program: {program}")
    for path in [params_file] + ([extra_params_file] if extra_params_file else []):
        if not os.path.isfile(path):
            # launch_ros 对缺失的参数文件只告警并继续，会让证据静默失真，这里直接拒绝。
            raise RuntimeError(f"parameter file not found: {path}")

    description_share = get_package_share_directory("aubo_description")
    moveit_share = get_package_share_directory("aubo_moveit_config")
    generated = os.path.join(description_share, "urdf", f"{aubo_type}.urdf.xacro")
    if not os.path.isfile(generated):
        raise RuntimeError(
            f"missing {generated}; start the Fake Hardware control launch first "
            "(it generates the model copy) instead of generating it here")

    # 与 aubo_moveit_config/launch/aubo_moveit.launch.py 相同的配置来源。
    moveit_config = (
        MoveItConfigsBuilder("aubo", package_name="aubo_moveit_config")
        .robot_description(
            file_path=os.path.join(description_share, "urdf/xacro/inc", "aubo_ros2.xacro"),
            mappings={"aubo_type": aubo_type},
        )
        .robot_description_semantic(
            file_path=os.path.join(moveit_share, "config/aubo_robot.srdf"))
        .robot_description_kinematics(
            file_path=os.path.join(moveit_share, "config", "kinematics.yaml"))
        .joint_limits(file_path=os.path.join(moveit_share, "config", "joint_limits.yaml"))
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    return [
        Node(
            package="mtc_moveit_planner",
            executable=program,
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                params_file,
                *([extra_params_file] if extra_params_file else []),
                {"output_dir": output_dir},
            ],
        )
    ]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("mtc_moveit_planner"), "config", "planner.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("program", default_value="plan_fixed_pose"),
        DeclareLaunchArgument("aubo_type", default_value="aubo_S3"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("extra_params_file", default_value="",
                              description="Optional YAML layered over params_file"),
        DeclareLaunchArgument("output_dir", default_value=os.path.join(os.getcwd(), "planning_evidence")),
        OpaqueFunction(function=launch_setup),
    ])
