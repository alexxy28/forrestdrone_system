from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    nodes = LaunchConfiguration('nodes')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Flag to enable use_sim_time',
    ))
    ld.add_action(DeclareLaunchArgument(
        'nodes',
        default_value='all',
        description='Nodes to start: all, or a comma-separated list of '
                    'system_control, husky, parrot',
    ))

    for name in ('system_control', 'husky', 'parrot'):
        ld.add_action(Node(
            package='control_template',
            executable=name + '_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", nodes, "' == 'all' or '", name, "' in '", nodes, "'",
            ])),
        ))

    return ld
