from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    ld.add_action(DeclareLaunchArgument(
        'namespace',
        default_value='parrot1',
        description='Robot namespace for the terrain map builder',
    ))
    ld.add_action(DeclareLaunchArgument(
        'robot_name',
        default_value='parrot1',
        description='Robot name used for frame naming',
    ))
    ld.add_action(DeclareLaunchArgument(
        'map_frame',
        default_value='parrot1_odom',
        description='Frame in which the accumulated terrain points are published',
    ))
    ld.add_action(DeclareLaunchArgument(
        'odom_topic',
        default_value='odometry',
        description='Odometry topic used to confirm the drone pose',
    ))
    ld.add_action(DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='scan/points',
        description='LiDAR point cloud topic to accumulate',
    ))
    ld.add_action(DeclareLaunchArgument(
        'output_topic',
        default_value='terrain_points',
        description='Topic name for the accumulated point cloud output',
    ))

    ld.add_action(Node(
        package='41068_ignition_bringup',
        executable='terrain_map_builder.py',
        name='terrain_map_builder',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name'),
            'map_frame': LaunchConfiguration('map_frame'),
            'odometry_topic': LaunchConfiguration('odom_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
        }],
        output='screen',
    ))

    ld.add_action(ExecuteProcess(
        cmd=['rqt', '--perspective-file', LaunchConfiguration('perspective_file')],
        output='screen',
    ))

    return ld
