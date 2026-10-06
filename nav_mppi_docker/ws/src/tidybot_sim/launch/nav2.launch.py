"""Nav2 baseline stack on top of sim.launch.py. No AMCL: odometry is ground truth
and map == odom, so the static map->odom in sim.launch.py is the whole localiser.

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

# The two servers team mode keeps. Named rather than sliced so reordering
# LIFECYCLE or SERVERS cannot quietly change which nodes team mode brings up.
TEAM = ('map_server', 'planner_server')


SERVERS = [
    ('nav2_map_server', 'map_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_controller', 'controller_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
]


def _setup(context, *_a, **_k):
    share = Path(get_package_share_directory('tidybot_sim'))
    mode = LaunchConfiguration('controller').perform(context)
    loc = LaunchConfiguration('localization').perform(context)
    # team 모드는 전용 파일을 읽는다. local costmap 의 global_frame(map vs odom),
    # global costmap 의 plugins, robot_radius(0.37 vs 0.35)가 동시에 다르고, 한 파일
    # 안에서 모드별로 바꾸는 방법이 이 프로젝트에서 둘 다 막혔기 때문이다. 파일이
    # 나뉘어 있으면 기준선이 구조적으로 드리프트할 수 없다.
    # team 모드의 params 파일은 인자로 바꿀 수 있다. Biased-MPPI A/B 의 off-wall /
    # on-wall 설정이 inflation_radius 만 다른 사본(nav2_params_team_wall.yaml)을 쓰기
    # 위한 것이다 -- 원본을 임시로 고치면 A/B 가 중단될 때 틀린 값이 남는다.
    team_params = LaunchConfiguration('team_params').perform(context)
    params = str(share / 'config' /
                 (team_params if mode == 'team' else 'nav2_params.yaml'))
    assert loc == 'gt' or mode == 'team', (
        f'localization:={loc} needs controller:=team; the nav2/mppi baselines run on '
        'Gazebo ground truth and their measured numbers assume it')
    mppi = mode == 'mppi'
    # team mode lifts the BT stack out entirely. The exploration system asks
    # planner_server for a path itself and then calls NavigateToGoal directly,
    # so bt_navigator, behavior_server and controller_server have nothing to do.
    # controller_server 를 되살리지 않는 이유는 goal 이 없어도 /cmd_vel 발행자를
    # 만들기 때문이다 -- "발행자 1개" 조건이 깨진다. local costmap 은 standalone
    # nav2_costmap_2d 가 대신 낸다. Recovery 도 사라진다; 재시도와 재계획은 탐색
    # 시스템의 일이다.
    servers = [x for x in SERVERS if x[1] in TEAM] if mode == 'team' else SERVERS
    # standalone costmap 도 라이프사이클 노드다. 완전수식 이름으로 적는다 --
    # 이름이 /costmap/costmap 으로 고정이라 상대 이름으로는 안 잡힌다.
    lifecycle = ([n for n in LIFECYCLE if n in TEAM] + ['/costmap/costmap']
                 if mode == 'team' else LIFECYCLE)
    # Overrides the empty yaml_filename in the params file. Two entries in
    # `parameters` merge with the later one winning, so no RewrittenYaml needed.
    map_yaml = {'yaml_filename': str(share / 'maps' / 'tidy.yaml')}

    nodes = [
        Node(package=pkg, executable=exe, name=exe, output='screen',
             parameters=[params] + ([map_yaml] if exe == 'map_server' else []),
             remappings=(FOLLOW_PATH_AWAY
                         if mppi and exe == 'controller_server' else []))
        for pkg, exe in servers
    ]
    if mppi:
        nodes.append(Node(
            package='tidybot_sim', executable='mppi_controller.py',
            name='mppi_controller', output='screen',
            parameters=[params, {'mppi_overrides':
                                 LaunchConfiguration('mppi_overrides').perform(context)}]))
    elif mode == 'team':
        # map_version 은 gen_world.py 가 maps/tidy.version 에 쓴 값이다. 노드는
        # 문자열 두 개를 비교할 뿐이라, 해시 방식이 바뀌어도 코드는 그대로다.
        nodes.append(Node(
            package='tidybot_sim', executable='navigate_to_goal.py',
            name='navigate_to_goal', output='screen',
            parameters=[params, {
                'map_version': (share / 'maps' / 'tidy.version').read_text().strip(),
                # gt 실행과 fastlio_mock 실행의 CSV 를 파일명으로 구분한다.
                'log_prefix': loc,
                'mppi_overrides':
                    LaunchConfiguration('mppi_overrides').perform(context)}]))
        # local costmap. Costmap2DROS 가 노드 이름과 네임스페이스를 /costmap/costmap
        # 으로 고정하고 __ns, __node 리매핑을 무시한다 -- 그래서 인스턴스는 하나만
        # 띄울 수 있고, 파라미터 경로도 costmap.costmap 이다. 토픽 리매핑은 통하므로
        # 실기와 같은 이름으로 내보낸다. /cmd_vel 발행자를 만들지 않는 노드라
        # "발행자 1개" 조건을 깨지 않는다.
        nodes.append(Node(
            package='nav2_costmap_2d', executable='nav2_costmap_2d',
            name='costmap', output='screen', parameters=[params],
            remappings=[('costmap', '/local_costmap/costmap'),
                        ('costmap_raw', '/local_costmap/costmap_raw'),
                        ('costmap_updates', '/local_costmap/costmap_updates')]))
    nodes.append(
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_nav', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': lifecycle}]))
    return nodes


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('tidybot_sim')) / 'launch' / 'sim.launch.py')),
        condition=IfCondition(LaunchConfiguration('sim')),
        launch_arguments={'scenario': LaunchConfiguration('scenario'),
                          'rviz': LaunchConfiguration('rviz'),
                          'headless': LaunchConfiguration('headless'),
                          'localization': LaunchConfiguration('localization')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='corridor'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        # team 모드 전용. 기본값이 원래 파일이라 평소 동작은 그대로다.
        DeclareLaunchArgument('team_params', default_value='nav2_params_team.yaml',
                              description='config/ 안의 team 모드 params 파일 이름'),
        # One knob, three stack shapes. It stays named `controller` because
        # benchmark.py passes controller:=<name> and the two baselines must keep
        # working unchanged; `team` is the third value rather than a second
        # argument so there is only ever one thing to read.
        DeclareLaunchArgument('controller', default_value='nav2',
                              choices=['nav2', 'mppi', 'team'],
                              description='nav2_mppi_controller | nav/mppi.py under the BT '
                                          '| team NavigateToGoal stack (no BT)'),
        DeclareLaunchArgument('mppi_overrides', default_value='',
                              description='MPPIConfig overrides, e.g. "lam=10.0"'),
        DeclareLaunchArgument('localization', default_value='gt',
                              choices=['gt', 'fastlio_mock'],
                              description='Gazebo ground truth, or the FAST-LIO mock TF '
                                          'chain (controller:=team only)'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='false to attach Nav2 to an already running sim'),
        sim,
        OpaqueFunction(function=_setup),
    ])
