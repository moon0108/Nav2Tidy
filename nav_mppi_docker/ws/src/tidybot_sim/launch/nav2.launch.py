"""Nav2 baseline stack on top of sim.launch.py. No AMCL: odometry is ground truth
and map == odom, so the static map->odom in sim.launch.py is the whole localiser.

    ros2 run rmw_zenoh_cpp rmw_zenohd            # terminal 1, first, always
    ros2 launch tidybot_sim nav2.launch.py scenario:=doorway

Then drop a "2D Goal Pose" in RViz, or:

    python3 scripts/send_goal.py doorway

    ros2 launch tidybot_sim nav2.launch.py controller:=mppi scenario:=utrap

nav2_bringup's navigation_launch.py is deliberately not included: stage 3 swaps
the FollowPath action server, and the node list there is hard-coded.

With controller:=mppi, controller_server keeps running but its follow_path
action is remapped to follow_path_unused. It stays because it owns
local_costmap, published_footprint and clear_entirely_local_costmap - the local
costmap the challenger reads, the footprint behavior_server needs, and the
service the BT's recovery branch calls. With nothing ever sending it a goal its
own MPPI never runs. Dropping the node entirely would mean standing up
nav2_costmap_2d instead, whose node name is hard-coded to "costmap" and which
overrides __ns with its own name, so it would take five remappings and a
duplicated params section to impersonate local_costmap.
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

# Remapping the bare action name does nothing: rcl_action builds
# "<action>/_action/<entity>" and remaps that, and a rule for "follow_path" does
# not match it. Verified - with -r follow_path:=follow_path_unused in place,
# `ros2 action info /follow_path` still listed two servers. So each of the five
# sub-entities is moved by name.
FOLLOW_PATH_AWAY = [(f'follow_path/_action/{e}', f'follow_path_unused/_action/{e}')
                    for e in ('send_goal', 'cancel_goal', 'get_result',
                              'feedback', 'status')]

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
    mppi = LaunchConfiguration('controller').perform(context) == 'mppi'
    # Overrides the empty yaml_filename in the params file. Two entries in
    # `parameters` merge with the later one winning, so no RewrittenYaml needed.
    map_yaml = {'yaml_filename': str(share / 'maps' / 'tidy.yaml')}

    nodes = [
        Node(package=pkg, executable=exe, name=exe, output='screen',
             parameters=[params] + ([map_yaml] if exe == 'map_server' else []),
             remappings=(FOLLOW_PATH_AWAY
                         if mppi and exe == 'controller_server' else []))
        for pkg, exe in SERVERS
    ]
    if mppi:
        nodes.append(Node(
            package='tidybot_sim', executable='mppi_controller.py',
            name='mppi_controller', output='screen',
            parameters=[params, {'mppi_overrides':
                                 LaunchConfiguration('mppi_overrides').perform(context)}]))
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
        DeclareLaunchArgument('controller', default_value='nav2',
                              choices=['nav2', 'mppi'],
                              description='nav2_mppi_controller, or nav/mppi.py'),
        DeclareLaunchArgument('mppi_overrides', default_value='',
                              description='MPPIConfig overrides, e.g. "lam=10.0"'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='false to attach Nav2 to an already running sim'),
        sim,
        OpaqueFunction(function=_setup),
    ])
