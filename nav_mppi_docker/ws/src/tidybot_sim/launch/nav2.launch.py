"""Nav2 baseline stack on top of sim.launch.py. No AMCL: odometry is ground truth
and map == odom, so the static map->odom in sim.launch.py is the whole localiser.

    ros2 run rmw_zenoh_cpp rmw_zenohd            # terminal 1, first, always
    ros2 launch tidybot_sim nav2.launch.py scenario:=doorway

Then drop a "2D Goal Pose" in RViz, or:

    python3 scripts/send_goal.py doorway

nav2_bringup's navigation_launch.py is deliberately not included: stage 3 has to
drop controller_server from the lifecycle list and swap in its own FollowPath
action server, and that list is hard-coded there. Here it is one line.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Brought up in this order; each must reach `active` before the next is configured.
LIFECYCLE = ['map_server', 'planner_server', 'controller_server',
             'behavior_server', 'bt_navigator']

SERVERS = [
    ('nav2_map_server', 'map_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_controller', 'controller_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
]


def _setup(context, *_a, **_k):
    share = Path(get_package_share_directory('tidybot_sim'))
    params = str(share / 'config' / 'nav2_params.yaml')
    # Overrides the empty yaml_filename in the params file. Two entries in
    # `parameters` merge with the later one winning, so no RewrittenYaml needed.
    map_yaml = {'yaml_filename': str(share / 'maps' / 'tidy.yaml')}

    nodes = [
        Node(package=pkg, executable=exe, name=exe, output='screen',
             parameters=[params] + ([map_yaml] if exe == 'map_server' else []))
        for pkg, exe in SERVERS
    ]
    nodes.append(
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_nav', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': LIFECYCLE}]))
    return nodes


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('tidybot_sim')) / 'launch' / 'sim.launch.py')),
        condition=IfCondition(LaunchConfiguration('sim')),
        launch_arguments={'scenario': LaunchConfiguration('scenario'),
                          'rviz': LaunchConfiguration('rviz'),
                          'headless': LaunchConfiguration('headless')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='corridor'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='false to attach Nav2 to an already running sim'),
        sim,
        OpaqueFunction(function=_setup),
    ])
