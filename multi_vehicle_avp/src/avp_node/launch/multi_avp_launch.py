
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    avp_file = LaunchConfiguration('avp_file')
    vehicle_id = LaunchConfiguration('vehicle_id')
    enable_managers = LaunchConfiguration('enable_managers')
    echo_avp = LaunchConfiguration('echo_avp')

    # Condition: echo if managers are disabled AND echo_avp is not explicitly false
    echo_condition = IfCondition(
        PythonExpression([
            '"', enable_managers, '" == "false" and ("', echo_avp, '" == "true" or "', echo_avp, '" == "auto")'
        ])
    )

    script_path = PathJoinSubstitution([
        FindPackageShare('avp_node'),
        'scripts',
        'echo_avp_topics.sh'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'avp_file',
            default_value='avp_node',
            description='Which AVP file to run'
        ),
        DeclareLaunchArgument(
            'vehicle_id',
            default_value='1',
            description='Vehicle ID to pass to the AVP script'
        ),
        DeclareLaunchArgument(
            'enable_managers',
            default_value='true',
            description='Set to false to disable launching manager nodes'
        ),
        DeclareLaunchArgument(
            'echo_avp',
            default_value='auto',
            description='Echo AVP-related topics if needed (true, false, or auto)'
        ),
        DeclareLaunchArgument(
            'namespaces',
            default_value='[main, vehicle2]',
            description='List of namespaces to manage'
        ),

        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Set to true to use planning simulator for debugging'
        ),

        # Only runs if enable_managers is false and echo_avp is true or auto
        ExecuteProcess(
            cmd=['bash', script_path],
            output='screen',
            condition=echo_condition
        ),

        # Manager Nodes
        Node(
            package='avp_managers',
            executable='drop_off_zone_queue_manager',
            name='drop_off_zone_queue_manager',
            output='screen',
            arguments=['--ros-args', '-p', ['namespaces:=', LaunchConfiguration('namespaces')]],
            condition=IfCondition(enable_managers)
        ),
        Node(
            package='avp_managers',
            executable='parking_spot_reservation_manager',
            name='parking_spot_reservation_manager',
            output='screen',
            arguments=['--ros-args', '-p', ['namespaces:=', LaunchConfiguration('namespaces')]],
            condition=IfCondition(enable_managers)
        ),
        Node(
            package='avp_managers',
            executable='vehicle_count_manager',
            name='vehicle_count_manager',
            output='screen',
            arguments=['--ros-args', '-p', ['namespaces:=', LaunchConfiguration('namespaces')]],
            condition=IfCondition(enable_managers)
        ),

        Node(
            package='avp_managers',
            executable='vehicle_status_manager',
            name='vehicle_status_manager',
            output='screen',
            arguments=['--ros-args', '-p', ['namespaces:=', LaunchConfiguration('namespaces')]],
            condition=IfCondition(enable_managers)
        ),

        # AVP Script Node
        Node(
            package='avp_node',
            executable=avp_file,
            output='screen',
            arguments=[
                '--vehicle_id', vehicle_id,
                '--debug', LaunchConfiguration('debug')
            ],
        ),
    ])