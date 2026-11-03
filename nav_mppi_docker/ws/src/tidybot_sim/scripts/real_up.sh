#!/usr/bin/env bash
# 실기 스택(받는 쪽)을 컨테이너 안에서 한 번에 띄운다. planner_server + MPPI.
# 호스트에서는 nav_mppi_docker/run_real.sh 가 이 파일을 docker exec 로 부른다.
#
#   real_up.sh         -> /cmd_vel_dryrun (건식. 로봇은 움직이지 않는다)
#   real_up.sh --arm   -> /cmd_vel (실제로 움직인다)
set -euo pipefail

TOPIC=/cmd_vel_dryrun
[ "${1:-}" = "--arm" ] && TOPIC=/cmd_vel

# ROS 의 setup.bash 는 nounset 에 안전하지 않다 (/opt/ros/humble/setup.bash:8 이
# AMENT_TRACE_SETUP_FILES 를 기본값 없이 읽는다). source 구간만 -u 를 끈다.
set +u
. /home/moon/ws/src/tidybot_sim/scripts/env_team.sh
set -u

# install/ 은 파일 단위 심링크다 (--symlink-install). 기존 파일을 고치는 데는
# 재빌드가 필요 없지만 **새 파일**은 한 번 빌드해야 share 에 나타난다. 안 하면
# "file 'real.launch.py' was not found in the share directory" 로 죽는데, 그
# 메시지만 보고 colcon build 를 떠올리기는 어렵다 -- 그래서 여기서 해 준다.
SHARE=/home/moon/ws/install/tidybot_sim/share/tidybot_sim
if [ ! -e "$SHARE/launch/real.launch.py" ]; then
    echo "real.launch.py 가 install 에 없다 -- colcon build 를 한 번 돌린다"
    (cd /home/moon/ws && colcon build --symlink-install --packages-select tidybot_sim)
    set +u
    . /home/moon/ws/install/setup.bash
    set -u
fi

# zenoh 라우터는 **동료 스택이 띄운다.** 여기서는 띄우지 않고 있는지만 본다.
# 이 이미지의 기본 세션 설정은 mode: "peer" 지만 scouting.multicast.enabled 가
# false 이고 gossip.target.peer 가 ["router"] 라, 라우터가 없으면 PC 한 대 안에서도
# 노드끼리 서로를 못 본다. 그 증상은 "조용히 아무것도 안 보임"이라 제일 찾기 나쁘다
# -- 그래서 그냥 뜨지 않고 여기서 멈춘다. (ss/netstat 가 이미지에 없어서 /dev/tcp)
if ! (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null; then
    echo "중단: 7447 에 zenoh 라우터가 없다. 라우터를 띄우는 쪽(동료 스택)을 먼저 올려라." >&2
    echo "      직접 띄우려면:  ros2 run rmw_zenoh_cpp rmw_zenohd" >&2
    exit 1
fi

# 무장 확인. 팀 로봇이 같은 도메인(25)에 떠 있을 수 있으므로, /cmd_vel 에 이미
# 발행자가 있으면 (시뮬이든 다른 세션이든) 거기에 두 번째를 얹지 않는다.
if [ "$TOPIC" = /cmd_vel ]; then
    echo "무장: $TOPIC -- /cmd_vel 발행자를 확인한다"
    # 그래프를 못 읽는 것과 "발행자가 없다"를 구별해야 한다. ros2 topic info 는
    # 토픽이 그래프에 아예 없으면 "Publisher count: 0" 이 아니라 **아무것도** 찍지
    # 않아서, 출력만 보면 둘이 똑같아 보인다. 그래서 목록으로 존재를 먼저 가른다.
    if ! topics=$(timeout 15 ros2 topic list 2>/dev/null); then
        echo "중단: ros2 topic list 가 실패했다. 라우터와 도메인(25)을 확인해라." >&2
        exit 1
    fi
    if printf '%s\n' "$topics" | grep -qx /cmd_vel; then
        info=$(timeout 15 ros2 topic info /cmd_vel 2>/dev/null || true)
        echo "$info"
        pubs=$(printf '%s\n' "$info" | sed -n 's/^Publisher count: //p')
        subs=$(printf '%s\n' "$info" | sed -n 's/^Subscription count: //p')
        [ "${pubs:-x}" = 0 ] || {
            echo "중단: /cmd_vel 발행자가 ${pubs:-?} 개다. 0 이어야 한다." >&2; exit 1; }
        [ "${subs:-0}" -ge 1 ] || echo "경고: /cmd_vel 구독자가 없다 -- 로봇 드라이버가 안 떠 있다"
    else
        echo "경고: /cmd_vel 이 그래프에 없다 -- 로봇 드라이버가 안 떠 있다. 무장은 계속한다"
    fi
fi

echo "launch: cmd_vel_topic:=$TOPIC"
ros2 launch tidybot_sim real.launch.py cmd_vel_topic:="$TOPIC"
