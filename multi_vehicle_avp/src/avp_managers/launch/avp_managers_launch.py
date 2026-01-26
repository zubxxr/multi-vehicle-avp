from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    namespaces = LaunchConfiguration('namespaces')
    enable_managers = LaunchConfiguration('enable_managers')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespaces',
            default_value="[]",
            description='List of vehicle namespaces'
        ),

        DeclareLaunchArgument(
            'enable_managers',
            default_value='true',
            description='Enable AVP manager nodes'
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
                        'namespaces': ParameterValue(namespaces, value_type=str)
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
                        'namespaces': ParameterValue(namespaces, value_type=str)
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
                        'namespaces': ParameterValue(namespaces, value_type=str)
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
                        'namespaces': ParameterValue(namespaces, value_type=str)
                    }],
                    condition=IfCondition(enable_managers)
                )
            ]
        ),
    ])
