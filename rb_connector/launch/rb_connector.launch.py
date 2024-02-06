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
            'pipeline', default_value='ompl',
            description='Planning pipeline'
        ),
        DeclareLaunchArgument(
            'load_robot_description', default_value='true',
            description='Load robot description'
        ),
        DeclareLaunchArgument(
            'ip', default_value='192.168.1.239',
            description='IP address for the rb_connector'
        ),

        GroupAction([
            SetLaunchConfiguration('robot_description_file', PathJoinSubstitution([
                FindPackageShare('rb_description'), 'urdf', LaunchConfiguration('model') + '_moveit.xacro'
            ])),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{
                    'robot_description': Command([
                        FindExecutable(name='xacro'), ' ',
                        LaunchConfiguration('robot_description_file')
                    ])
                }],
                output='screen',
                condition=IfCondition(LaunchConfiguration('load_robot_description'))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare(LaunchConfiguration('model') + '_moveit_config'), 'launch', 'move_group.launch.py'
                ])),
                launch_arguments={
                    'allow_trajectory_execution': 'true',
                    'fake_execution': 'false',
                    'info': 'true',
                    'pipeline': LaunchConfiguration('pipeline'),
                    'load_robot_description': LaunchConfiguration('load_robot_description'),
                    'publish_monitored_planning_scene': 'true',
                }.items(),
                condition=IfCondition(LaunchConfiguration('load_robot_description'))
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare(LaunchConfiguration('model') + '_moveit_config'), 'launch', 'moveit.rviz'
                ])],
                condition=IfCondition(LaunchConfiguration('load_robot_description'))
            )
        ], condition=IfCondition(LaunchConfiguration('load_robot_description'))),

        Node(
            package='rb_connector',
            executable='rb_connector',
            name='rb_connector',
            parameters=[{'ip': LaunchConfiguration('ip')}],
            output='screen'
        )
    ])
