from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bringup = FindPackageShare('41068_ignition_bringup')

    # ---------------------------------------------------------
    # 1. Complete simulation
    #    terrain_demo + Husky + Parrot + Husky Nav2/SLAM + RViz
    # ---------------------------------------------------------
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup,
                'launch',
                '41068_ignition.launch.py'
            ])
        ),
        launch_arguments={
            'world': 'terrain_demo',
            'husky': 'true',
            'parrot': 'true',
            'rviz': 'true',
            'nav2': 'false',
            'slam': 'false',
        }.items()
    )

    # ---------------------------------------------------------
    # 2. TRACE dashboard + Parrot terrain mapping
    # ---------------------------------------------------------
    dashboard = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        bringup,
                        'launch',
                        '41068_rqt_dashboard.launch.py'
                    ])
                )
            )
        ]
    )

    # ---------------------------------------------------------
    # 3. Thermal casualty candidate detector
    # ---------------------------------------------------------
    thermal_detector = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='control_template',
                executable='thermal_casualty_detector',
                name='thermal_casualty_detector',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        simulation,
        dashboard,
        thermal_detector,
    ])
