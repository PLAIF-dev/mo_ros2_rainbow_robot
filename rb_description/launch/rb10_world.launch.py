from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'paused', default_value='false',
            description='Start simulation in paused mode'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Launch GUI'),

        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run in headless mode'),

        DeclareLaunchArgument(
            'debug', default_value='false',
            description='Run in debug mode'),

        # Load the URDF into the ROS Parameter Server
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    PathJoinSubstitution([FindExecutable(name='xacro')]),
                    ' ',
                    PathJoinSubstitution([FindPackageShare('rb_description'), 'urdf', 'rb10_moveit.xacro']),
                ])
            }],
            output='screen'),

        # Joint State Publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=LaunchConfiguration('gui')),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'),

        # Rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('rb_description'), 'launch', 'rb10.rviz'])],
            output='screen'),
    ])
