from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='parrot1',
        description='Robot namespace for the terrain map builder',
    )
    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='parrot1',
        description='Robot name used for frame naming',
    )
    map_frame = DeclareLaunchArgument(
        'map_frame',
        default_value='parrot1_odom',
        description='Frame in which the accumulated terrain points are published',
    )
    odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='odometry',
        description='Odometry topic used to confirm the drone pose',
    )
    pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='scan/points',
        description='LiDAR point cloud topic to accumulate',
    )
    output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='terrain_points',
        description='Topic name for the accumulated point cloud output',
    )

    terrain_node = Node(
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
    )

    ld.add_action(namespace)
    ld.add_action(robot_name)
    ld.add_action(map_frame)
    ld.add_action(odom_topic)
    ld.add_action(pointcloud_topic)
    ld.add_action(output_topic)
    ld.add_action(terrain_node)

    return ld
