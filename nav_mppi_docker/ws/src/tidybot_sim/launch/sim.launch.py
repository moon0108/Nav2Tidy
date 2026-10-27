"""Gazebo Fortress + the TidyBot++ base + the ros_gz bridge.

    ros2 launch tidybot_sim sim.launch.py scenario:=doorway

RMW 는 rmw_cyclonedds_cpp 이고 라우터가 필요 없다 (팀 표준). 기본 실행은
ROS_LOCALHOST_ONLY=1, ROS_DOMAIN_ID=42 라 PC 밖으로 나가지 않는다 -- compose.yaml 참조.

`scenario` only picks the spawn pose. All four regions live in the one world, so
switching scenarios never needs a Gazebo restart - the benchmark in stage 4
teleports between them instead.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nav import scene

# Fortress message types are ignition.msgs.*; Harmonic's are gz.msgs.*.
# '[' is gz->ros, ']' is ros->gz.
BRIDGE = [
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
    '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
    '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
]


def _setup(context, *_a, **_k):
    share = Path(get_package_share_directory('tidybot_sim'))
    (x, y, yaw), _goal = scene.SCENARIOS[LaunchConfiguration('scenario').perform(context)]
    sim_time = {'use_sim_time': True}

    # localization:=fastlio_mock 은 Gazebo ground truth 대신 FAST-LIO 의 TF 체인을
    # 흉내 낸다: map -> camera_init -> imu_link -> base_link. 그러려면 Gazebo 의
    # odom->base_link TF 를 꺼야 한다 - 켜 두면 base_link 에 부모가 둘 생긴다.
    # /odom 토픽 자체는 mock 의 입력이므로 그대로 둔다.
    fastlio = LaunchConfiguration('localization').perform(context) == 'fastlio_mock'
    bridge = [b for b in BRIDGE if not (fastlio and b.startswith('/tf@'))]

    nodes = [
        # -s is server-only. The Sensors system still renders gpu_lidar, so the
        # headless benchmark in stage 4 and a GUI run see the same scan.
        ExecuteProcess(output='screen', cmd=(
            ['ign', 'gazebo', '-r', '-v', '3']
            + (['-s'] if LaunchConfiguration('headless').perform(context) == 'true' else [])
            + [str(share / 'worlds' / 'tidy.sdf')])),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[
                 {'robot_description': (share / 'urdf' / 'tidybot.urdf').read_text()},
                 sim_time]),

        # create calls /world/tidy/create, which does not exist until the server
        # is up, and it does not retry. The delay is the whole fix.
        TimerAction(period=4.0, actions=[
            Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
                '-topic', 'robot_description', '-name', 'tidybot',
                # z = 0.01: the caster spheres bottom out at z = 0, so this drops
                # the base 1 cm onto the floor instead of starting it interpenetrated.
                '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw)])]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=bridge, parameters=[sim_time]),

        # gt 에서만 띄운다: 오도메트리가 ground truth 라 map 과 odom 이 같은
        # 프레임이다. AMCL 없음.
        #
        # fastlio_mock 에서는 map -> camera_init 을 fastlio_mock 노드가 직접 낸다
        # (10 Hz 동적, 실기의 transform_fusion 자리). 여기서 static 으로도 내면
        # camera_init 에 부모가 둘 생긴다.
        *([] if fastlio else [
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='map_to_odom',
                 arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
                 parameters=[sim_time])]),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             # nav2_bringup already ships a view with TF, LaserScan, Map, RobotModel
             # and the costmap/plan displays stage 2 will fill in. Those read
             # "no messages" until then; nothing else here needs an .rviz file.
             arguments=['-d', str(Path(get_package_share_directory('nav2_bringup'))
                                  / 'rviz' / 'nav2_default_view.rviz')],
             parameters=[sim_time]),
    ]
    if fastlio:
        # team 전용 파일을 읽는다. fastlio_mock 섹션이 거기 있다 -- 기준선 파일을
        # 넘기면 섹션이 없어 **코드 기본값으로 조용히 돈다**. 실제로 당했다:
        # require_initialpose 가 무시되어 테스트가 통과해 버렸다.
        nodes.append(Node(package='tidybot_sim', executable='fastlio_mock.py',
                          name='fastlio_mock', output='screen',
                          parameters=[str(share / 'config' / 'nav2_params_team.yaml')]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='corridor',
                              choices=sorted(scene.SCENARIOS),
                              description='spawn pose, from nav/scene.py SCENARIOS'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false',
                              choices=['true', 'false']),
        # gt 는 지금까지의 동작 그대로 -- Gazebo 의 odom->base_link TF 와 map->odom
        # 항등. fastlio_mock 은 실제 로봇의 TF 체인을 흉내 낸다. team 모드 전용이고,
        # nav2.launch.py 가 그것을 강제한다.
        DeclareLaunchArgument('localization', default_value='gt',
                              choices=['gt', 'fastlio_mock'],
                              description='Gazebo ground truth, or the FAST-LIO mock chain'),
        OpaqueFunction(function=_setup),
    ])
