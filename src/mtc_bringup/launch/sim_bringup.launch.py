"""Run the isolated V1 Gazebo simulation; never starts a real robot driver."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def setup(context):
    command=['ros2','run','mtc_simulation','run_v1.py']
    for key in ('task','order','output','domain','timeout'):
        value=LaunchConfiguration(key).perform(context)
        if value:command.extend(['--'+key,value])
    for key in ('gui','observe_only','pin_moveit_plugins'):
        if LaunchConfiguration(key).perform(context).lower()=='true':command.append('--'+key.replace('_','-'))
    return [ExecuteProcess(cmd=command,output='screen',sigterm_timeout='15')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('task',default_value='basic'),
        DeclareLaunchArgument('order',default_value=''),
        DeclareLaunchArgument('output',description='New output directory for logs, video and acceptance'),
        DeclareLaunchArgument('domain',default_value='86'),
        DeclareLaunchArgument('timeout',default_value='1800'),
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('observe_only',default_value='false'),
        DeclareLaunchArgument('pin_moveit_plugins',default_value='false'),
        OpaqueFunction(function=setup)])
