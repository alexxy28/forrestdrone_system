"""TRACE demo: everything for one rescue mission from a single command.

    ros2 launch 41068_ignition_bringup trace_demo.launch.py

Starts, in order:
  1. Gazebo (terrain_demo, random casualty pose), Husky + Parrot, SLAM, Husky Nav2, RViz
  2. after `control_delay` s: parrot_node, terrain_map_node, system_control_node,
     husky_node. The drone waits on the ground (PAUSED) until the operator
     presses Start Parrot on the dashboard; it then climbs and (auto_start)
     begins its area search
  3. the rqt operator dashboard (terrain map, camera feeds, confirm/reject casualty,
     stop buttons, mission state)
  4. mission_logger (log_results:=true): prints and saves a scored result per run
     to ~/trace_results/results.csv (person's true position from Gazebo, for
     evaluation only; the robots never see it)

Useful arguments: world:=terrain_demo rviz:=true rqt:=true auto_start:=true gui:=true
                  control_delay:=20.0 human_pose:=0..5 log_results:=true
                  hold_check_s:=120 parrot_start_paused:=true
                  parrot2:=true  (second drone; drone_search:=race or split)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('41068_ignition_bringup')

    def launch_file(name):
        return PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', name]))

    args = [
        DeclareLaunchArgument('world', default_value='terrain_demo',
                              description='terrain_demo contains the heat-signature casualty'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('human_pose', default_value='random',
                              description='Casualty spawn: random, or 0-5 to repeat a scenario'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo window (false = server only, less CPU)'),
        DeclareLaunchArgument('rqt', default_value='true',
                              description='Open the operator dashboard'),
        DeclareLaunchArgument('auto_start', default_value='true',
                              description='Drone starts searching without a service call'),
        DeclareLaunchArgument('parrot_start_paused', default_value='true',
                              description='Drone waits on the ground until Start Parrot '
                                          'is pressed on the dashboard'),
        DeclareLaunchArgument('parrot2', default_value='false',
                              description='Add a second drone (/parrot2)'),
        DeclareLaunchArgument('drone_search', default_value='race',
                              description='With parrot2: race = both drones search the whole '
                                          'area on different paths, split = each searches half'),
        DeclareLaunchArgument('control_delay', default_value='20.0',
                              description='Seconds to wait for Gazebo/Nav2 before the mission nodes'),
        DeclareLaunchArgument('hold_check_s', default_value='120.0',
                              description='Test logger: seconds (sim) to keep watching the '
                                          'Husky after DONE before scoring the run'),
        DeclareLaunchArgument('log_results', default_value='true',
                              description='Score the run against Gazebo ground truth '
                                          '(~/trace_results/results.csv)'),
    ]

    sim = IncludeLaunchDescription(
        launch_file('41068_ignition.launch.py'),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'husky': 'true',
            'parrot': 'true',
            'parrot2': LaunchConfiguration('parrot2'),
            'slam': 'true',
            'nav2': 'true',          # Husky Nav2 only; parrot_nav2 defaults to false
            'rviz': LaunchConfiguration('rviz'),
            'gui': LaunchConfiguration('gui'),
            'human_pose': LaunchConfiguration('human_pose'),
        }.items(),
    )

    control = TimerAction(
        period=LaunchConfiguration('control_delay'),
        actions=[IncludeLaunchDescription(
            launch_file('control.launch.py'),
            launch_arguments={
                'auto_start': LaunchConfiguration('auto_start'),
                'parrot_start_paused': LaunchConfiguration('parrot_start_paused'),
                'parrot2': LaunchConfiguration('parrot2'),
                'drone_search': LaunchConfiguration('drone_search'),
            }.items(),
        )],
    )

    dashboard = TimerAction(
        period=LaunchConfiguration('control_delay'),
        actions=[IncludeLaunchDescription(
            launch_file('41068_rqt_dashboard.launch.py'),
            launch_arguments={'parrot2': LaunchConfiguration('parrot2')}.items(),
            condition=IfCondition(LaunchConfiguration('rqt')),
        )],
    )

    logger = Node(
        package='41068_ignition_bringup',
        executable='mission_logger.py',
        name='mission_logger',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'world': LaunchConfiguration('world'),
                     # a string label ("0".."5" or "random"), not an integer
                     'human_pose': ParameterValue(LaunchConfiguration('human_pose'),
                                                  value_type=str),
                     'hold_check_s': ParameterValue(LaunchConfiguration('hold_check_s'),
                                                    value_type=float)}],
        condition=IfCondition(LaunchConfiguration('log_results')),
    )

    return LaunchDescription(args + [sim, control, dashboard, logger])
