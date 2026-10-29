from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    nodes = LaunchConfiguration('nodes')
    auto_start = LaunchConfiguration('auto_start')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Flag to enable use_sim_time',
    ))
    ld.add_action(DeclareLaunchArgument(
        'nodes',
        default_value='all',
        description='Nodes to start: all, or a comma-separated list of '
                    'system_control, husky, parrot, terrain_map',
    ))
    ld.add_action(DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Start the drone area search automatically once it has climbed '
                    '(otherwise: ros2 service call /mission/start std_srvs/srv/Trigger)',
    ))

    for name in ('system_control', 'husky', 'parrot', 'terrain_map'):
        params = {'use_sim_time': use_sim_time}
        if name == 'system_control':
            params['auto_start'] = ParameterValue(auto_start, value_type=bool)
        ld.add_action(Node(
            package='control_template',
            executable=name + '_node',
            parameters=[params],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", nodes, "' == 'all' or '", name, "' in '", nodes, "'",
            ])),
        ))

    return ld
