import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _preload_height_map(context, *_args, **_kwargs):
    # Plays a saved terrain_map bag onto /parrot1/terrain_seed. terrain_map_node
    # keeps the first cloud and ignores the rest, so --loop removes any
    # start-order race between the bag and the node.
    height_map = LaunchConfiguration('height_map').perform(context).strip()
    if not height_map:
        return []

    path = os.path.expanduser(height_map)
    if not os.path.isdir(path):
        path = os.path.join(get_package_share_directory('41068_ignition_bringup'),
                            'pre_scanned_height_maps', height_map)
    if not os.path.isfile(os.path.join(path, 'metadata.yaml')):
        raise RuntimeError(
            f"height_map '{height_map}' is not a bag folder or a name in "
            f"pre_scanned_height_maps/ (looked for {path}/metadata.yaml)")

    return [
        LogInfo(msg=f'Preloading height map from {path}'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', path, '--loop',
                 '--remap', '/parrot1/terrain_points:=/parrot1/terrain_seed'],
            output='log',
        ),
    ]


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
    ld.add_action(DeclareLaunchArgument(
        'height_map',
        default_value='',
        description='Saved terrain map to preload into terrain_map_node: a name in '
                    'pre_scanned_height_maps/ (e.g. terrain_demo) or a bag folder path. '
                    'Empty = start with a blank map',
    ))

    # Before the nodes, so a bad height_map aborts the launch before anything starts.
    ld.add_action(OpaqueFunction(function=_preload_height_map))

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
