"""실기 전체 스택: planner_server + MPPI 로컬 플래너를 한 번에 띄운다.

    source scripts/env_team.sh
    ros2 launch tidybot_sim real.launch.py cmd_vel_topic:=/cmd_vel

호스트에서 한 줄로 띄우려면 nav_mppi_docker/run_real.sh 를 쓴다.

cmd_vel_topic 기본값은 /cmd_vel_dryrun 이다 -- 팀 로봇이 같은 도메인(25)에 떠 있을
수 있어서 무장은 항상 명시적이어야 한다.

목표를 **보내는** 쪽(동료의 탐색 서버, 또는 mock_explorer.py)은 여기 없다.
이 launch 가 띄우는 것은 받는 쪽뿐이다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('tidybot_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_dryrun',
                              description='무장하려면 /cmd_vel. 기본값은 건식이다'),
        DeclareLaunchArgument('params',
                              default_value=str(share / 'config' / 'team_real.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / 'launch' / 'real_planner.launch.py'))),
        Node(package='tidybot_sim', executable='navigate_to_goal.py',
             name='navigate_to_goal', output='screen',
             parameters=[LaunchConfiguration('params'),
                         {'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')}]),
    ])
