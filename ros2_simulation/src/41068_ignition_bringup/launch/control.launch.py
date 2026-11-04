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
        'parrot_start_paused',
        default_value='false',
        description='Keep the drone on the ground (PAUSED) until Start Parrot is pressed '
                    '(or: ros2 topic pub --once /mission_control/parrot1/estop '
                    'std_msgs/msg/Bool "{data: false}")',
    ))
    ld.add_action(DeclareLaunchArgument(
        'height_map',
        default_value='',
        description='Saved terrain map to preload into terrain_map_node: a name in '
                    'pre_scanned_height_maps/ (e.g. terrain_demo) or a bag folder path. '
                    'Empty = start with a blank map',
    ))

    ld.add_action(DeclareLaunchArgument(
        'parrot2',
        default_value='false',
        description='Second drone: start a parrot_node for /parrot2, split the search '
                    'area between the drones and map with both LiDARs',
    ))
    second_drone = ParameterValue(LaunchConfiguration('parrot2'), value_type=bool)
    ld.add_action(DeclareLaunchArgument(
        'drone_search',
        default_value='race',
        description='With parrot2: race = both drones search the whole area on different paths, '
                    'split = each drone searches half',
    ))

    # Before the nodes, so a bad height_map aborts the launch before anything starts.
    ld.add_action(OpaqueFunction(function=_preload_height_map))

    for name in ('system_control', 'husky', 'parrot', 'terrain_map'):
        params = {'use_sim_time': use_sim_time}
        if name == 'system_control':
            params['auto_start'] = ParameterValue(auto_start, value_type=bool)
            params['second_drone'] = second_drone
            params['drone_search'] = LaunchConfiguration('drone_search')
        if name == 'terrain_map':
            params['second_drone'] = second_drone
        if name == 'parrot':
            params['start_paused'] = ParameterValue(
                LaunchConfiguration('parrot_start_paused'), value_type=bool)
            params['multi_drone'] = second_drone
        ld.add_action(Node(
            package='control_template',
            executable=name + '_node',
            parameters=[params],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", nodes, "' == 'all' or '", name, "' in '", nodes, "'",
            ])),
        ))

    # Second drone: the same parrot_node program, renamed and pointed at the
    # /parrot2 topics. Every topic it uses is a parameter, so no code is copied.
    ld.add_action(Node(
        package='control_template',
        executable='parrot_node',
        name='parrot2_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'start_paused': ParameterValue(
                LaunchConfiguration('parrot_start_paused'), value_type=bool),
            'multi_drone': True,
            # ParrotNode
            'mission_path_topic': '/parrot2/mission/path',
            'mission_status_topic': '/parrot2/mission/status',
            'stop_topic': '/parrot2/estop',
            # Parrot (flight)
            'odom_topic': '/parrot2/odometry',
            'cmd_topic': '/parrot2/cmd_vel',
            'scan_topic': '/parrot2/obstacle_scan',
            'agl_cloud_topic': '/parrot2/scan/points',
            'marker_topic': '/parrot2/obstacle_markers',
            # ThermalCamera (detection)
            'thermal_topic': '/parrot2/thermal_camera',
            'depth_topic': '/parrot2/camera/depth/image',
            'detection_odometry_topic': '/parrot2/odometry',
        }],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('parrot2'), "'.lower() in ['true', '1', 'yes', 'on'] and ('",
            nodes, "' == 'all' or 'parrot' in '", nodes, "')",
        ])),
    ))

    return ld
