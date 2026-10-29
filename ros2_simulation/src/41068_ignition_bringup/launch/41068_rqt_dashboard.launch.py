from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # The terrain map shown here is published by terrain_map_node, which
    # control.launch.py starts.
    pkg_share = FindPackageShare('41068_ignition_bringup')
    perspective_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'my_dashboard.perspective',
    ])

    ld = LaunchDescription()

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

    ld.add_action(ExecuteProcess(
        cmd=['rqt', '--perspective-file', LaunchConfiguration('perspective_file')],
        output='screen',
    ))

    return ld
