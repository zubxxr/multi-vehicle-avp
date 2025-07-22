
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    avp_file = LaunchConfiguration('avp_file')
    vehicle_id = LaunchConfiguration('vehicle_id')
    enable_managers = LaunchConfiguration('enable_managers')
    
    script_path = PathJoinSubstitution([
        FindPackageShare('avp_node'),
        'scripts',
        'echo_avp_topics.sh'
    ])

    return LaunchDescription([

        DeclareLaunchArgument('avp_file', default_value='avp_node', description='Which AVP executable to run'),
        DeclareLaunchArgument('vehicle_id', default_value='1', description='Vehicle ID to pass to the AVP script'),
        DeclareLaunchArgument('enable_managers', default_value='false', description='Enable manager nodes'),
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode (simulator use)'),
        DeclareLaunchArgument('namespaces', default_value="[]", description='List of namespaces'),

        TimerAction(
            period=0.5,
            actions=[
                ExecuteProcess(
                    cmd=['bash', script_path],
                    output='screen',
                )
            ]
        ),

        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='avp_managers',
                    executable='drop_off_zone_queue_manager',
                    name='drop_off_zone_queue_manager',
                    output='screen',
                    parameters=[{
                        'namespaces': ParameterValue(LaunchConfiguration('namespaces'), value_type=str)
                    }],
                    condition=IfCondition(enable_managers)
                )
            ]
        ),
        TimerAction(
            period=2.5,
            actions=[
                Node(
                    package='avp_managers',
                    executable='parking_spot_reservation_manager',
                    name='parking_spot_reservation_manager',
                    output='screen',
                    parameters=[{
                        'namespaces': ParameterValue(LaunchConfiguration('namespaces'), value_type=str)
                    }],
                    condition=IfCondition(enable_managers)
                )
            ]
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='avp_managers',
                    executable='vehicle_count_manager',
                    name='vehicle_count_manager',
                    output='screen',
                    parameters=[{
                        'namespaces': ParameterValue(LaunchConfiguration('namespaces'), value_type=str)
                    }],
                    condition=IfCondition(enable_managers)
                )
            ]
        ),
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='avp_managers',
                    executable='vehicle_status_manager',
                    name='vehicle_status_manager',
                    output='screen',
                    parameters=[{
                        'namespaces': ParameterValue(LaunchConfiguration('namespaces'), value_type=str)
                    }],
                    condition=IfCondition(enable_managers)
                )
            ]
        ),

        # AVP Node
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='avp_node',
                    executable=avp_file,
                    output='screen',
                    arguments=[
                        '--vehicle_id', vehicle_id,
                        '--debug', LaunchConfiguration('debug')
                    ],
                )
            ]
        ),
    ])