from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, ThisLaunchFileDir, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ip', default_value='10.0.2.7',
            description='IP address for the rb_connector'),

        # Set robot_description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'), 
                    ' ', 
                    PathJoinSubstitution([
                        FindPackageShare('rb_moveit_config'), 'config', 'rb10.urdf.xacro'
                    ])
                ])
            }],
            output='screen'),

        # rb_connector Node
        Node(
            package='rb_connector',
            executable='rb_connector_node',
            name='rb_connector',
            parameters=[{
                'ip': LaunchConfiguration('ip')
            }],
            output='screen'),

        # Include move_group launch file from rb10_moveit_config package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rb_moveit_config'), 'launch', 'move_group.launch.py'
                ])
            ]),
            launch_arguments={
                'allow_trajectory_execution': 'true',
                'fake_execution': 'false',
                'info': 'true',
                'publish_monitored_planning_scene': 'true',
            }.items()),

        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('rb_moveit_config'), 'launch', 'moveit.rviz'])],
            output='screen'),
    ])
