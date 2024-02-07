#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='rb10',
            description='Robot model'
        ),
        DeclareLaunchArgument(
            'ip', default_value='192.168.1.139',
            description='IP address for the rb_connector'
        ),
        Node(
            package='rb_connector',
            executable='rb_connector',
            name='rb_connector',
            parameters=[{'ip': LaunchConfiguration('ip')}],
            output='screen'
        )
    ])
