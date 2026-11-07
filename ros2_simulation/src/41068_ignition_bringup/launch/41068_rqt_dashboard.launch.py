from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # The terrain map shown here is published by terrain_map_node, which
    # control.launch.py starts.
    pkg_share = FindPackageShare('41068_ignition_bringup')

    ld = LaunchDescription()

    # Second drone: the same dashboard plus Parrot 2 RGB and Parrot 2 Heat panels.
    ld.add_action(DeclareLaunchArgument(
        'parrot2',
        default_value='false',
        description='Use the two-drone layout (adds the Parrot 2 camera panels)',
    ))
    second_drone = PythonExpression([
        "'", LaunchConfiguration('parrot2'), "'.lower() in ['true', '1', 'yes', 'on']"])
    perspective_file = PathJoinSubstitution([
        pkg_share,
        'config',
        PythonExpression([
            "'my_dashboard_two_drones.perspective' if ", second_drone,
            " else 'my_dashboard.perspective'"]),
    ])

    ld.add_action(DeclareLaunchArgument(
        'perspective_file',
        default_value=perspective_file,
        description='Path to the saved rqt perspective file',
    ))

    # False-colour thermal images for the "Heat" panels (the raw mono16 image
    # looks almost uniformly grey in rqt Image View).
    ld.add_action(Node(
        package='41068_ignition_bringup',
        executable='thermal_view.py',
        name='thermal_view',
        output='screen',
    ))
    ld.add_action(Node(
        package='41068_ignition_bringup',
        executable='thermal_view.py',
        name='thermal_view_parrot2',
        parameters=[{'cameras': ['parrot2']}],
        output='screen',
        condition=IfCondition(second_drone),
    ))

    ld.add_action(ExecuteProcess(
        cmd=[
            'rqt',
            '--clear-config',
            '--force-discover',
            '--perspective-file',
            LaunchConfiguration('perspective_file'),
        ],
        output='screen',
    ))

    return ld
