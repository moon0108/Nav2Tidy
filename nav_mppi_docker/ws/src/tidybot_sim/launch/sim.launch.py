"""Gazebo Fortress + the TidyBot++ base + the ros_gz bridge.

Start the zenoh router FIRST, in its own terminal, or no two nodes will see each
other (rmw_zenoh_cpp has no discovery without it):

    ros2 run rmw_zenoh_cpp rmw_zenohd

It is deliberately not launched here: a second router on a host that already has
one just fights the first.

    ros2 launch tidybot_sim sim.launch.py scenario:=doorway

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

    return [
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
             arguments=BRIDGE, parameters=[sim_time]),

        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
             # Ground-truth odometry, so map and odom are the same frame. No AMCL.
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[sim_time]),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             # nav2_bringup already ships a view with TF, LaserScan, Map, RobotModel
             # and the costmap/plan displays stage 2 will fill in. Those read
             # "no messages" until then; nothing else here needs an .rviz file.
             arguments=['-d', str(Path(get_package_share_directory('nav2_bringup'))
                                  / 'rviz' / 'nav2_default_view.rviz')],
             parameters=[sim_time]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='corridor',
                              choices=sorted(scene.SCENARIOS),
                              description='spawn pose, from nav/scene.py SCENARIOS'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false',
                              choices=['true', 'false']),
        OpaqueFunction(function=_setup),
    ])
