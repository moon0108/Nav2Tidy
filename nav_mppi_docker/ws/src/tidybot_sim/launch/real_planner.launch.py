"""실기 모니터링 모드: planner_server 하나만 띄운다.

    source scripts/env_team.sh
    ros2 launch tidybot_sim real_planner.launch.py

동료 PC 의 SLAM/costmap/구동 노드가 이미 도메인 25 에 떠 있는 상태를 전제한다.
여기서 띄우는 것은 A* 경로를 만들 planner 뿐이고, /cmd_vel 발행자는 만들지 않는다
(controller_server 와 bt_navigator 를 일부러 제외한다 -- 그 둘은 goal 이 없어도
/cmd_vel 발행자를 만들어 "발행자 1개" 조건을 깬다).

map_server 도 띄우지 않는다. 동료의 /map 을 그대로 구독한다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = str(Path(get_package_share_directory('tidybot_sim'))
                 / 'config' / 'real_planner.yaml')
    return LaunchDescription([
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_nav', output='screen',
             # 실기에는 /clock 이 없다. use_sim_time 을 켜면 전이가 영영 안 끝난다.
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['planner_server']}]),
    ])
