"""Load the exact same generated model in planning and physical execution."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    run=Path(LaunchConfiguration('run_dir').perform(context))
    mode=LaunchConfiguration('mode').perform(context)
    share=Path(get_package_share_directory('aubo_moveit_config'))/'config'
    params={'use_sim_time':True,'robot_description':(run/'robot.urdf').read_text(),
            'robot_description_semantic':(run/'robot.srdf').read_text(),
            'robot_description_kinematics':yaml.safe_load((share/'kinematics.yaml').read_text()),
            'robot_description_planning':yaml.safe_load((share/'joint_limits.yaml').read_text()),
            'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl',
            'ompl':yaml.safe_load((share/'ompl_planning.yaml').read_text()),
            'allow_trajectory_execution':False,'trajectory_execution.manage_controllers':False,
            'moveit_controller_manager':'mtc_simulation/PlanningOnly',
            'publish_robot_description_semantic':True}
    if mode=='planner':
        env={}
        if LaunchConfiguration('pin_plugins').perform(context).lower()=='true':
            libs=[Path(get_package_prefix('mtc_simulation'))/'lib/libmtc_sim_planning_only.so',
                  Path(get_package_prefix('moveit_ros_move_group'))/'lib/libmoveit_move_group_default_capabilities.so']
            if not all(p.is_file() for p in libs):raise RuntimeError('MoveIt plugin pinning library missing')
            env['LD_PRELOAD']=':'.join(map(str,libs))
        return [Node(package='moveit_ros_move_group',executable='move_group',parameters=[params],additional_env=env,output='screen')]
    if mode!='task':raise ValueError(mode)
    params.update(scenario_file=str(run/'scenario.json'),output_dir=str(run),
                  order=LaunchConfiguration('order').perform(context))
    return [Node(package='mtc_simulation',executable='v1_task',parameters=[params],output='screen')]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_dir'),DeclareLaunchArgument('mode',default_value='planner'),DeclareLaunchArgument('pin_plugins',default_value='false'),
                              DeclareLaunchArgument('order',default_value='A'),OpaqueFunction(function=setup)])
