#!/usr/bin/env bash
# 실기 스택(받는 쪽)을 컨테이너 안에서 한 번에 띄운다. planner_server + MPPI.
# 호스트에서는 nav_mppi_docker/run_real.sh 가 이 파일을 docker exec 로 부른다.
#
#   real_up.sh         -> /cmd_vel_dryrun (건식. 로봇은 움직이지 않는다)
#   real_up.sh --arm   -> /cmd_vel (실제로 움직인다)
set -euo pipefail

TOPIC=/cmd_vel_dryrun
[ "${1:-}" = "--arm" ] && TOPIC=/cmd_vel

. /home/moon/ws/src/tidybot_sim/scripts/env_team.sh

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
    info=$(timeout 10 ros2 topic info /cmd_vel || true)
    echo "$info"
    pubs=$(printf '%s\n' "$info" | sed -n 's/^Publisher count: //p')
    subs=$(printf '%s\n' "$info" | sed -n 's/^Subscription count: //p')
    [ "${pubs:-x}" = 0 ] || {
        echo "중단: /cmd_vel 발행자가 ${pubs:-?} 개다. 0 이어야 한다." >&2; exit 1; }
    [ "${subs:-0}" -ge 1 ] || echo "경고: /cmd_vel 구독자가 없다 -- 로봇 드라이버가 안 떠 있다"
fi

echo "launch: cmd_vel_topic:=$TOPIC"
ros2 launch tidybot_sim real.launch.py cmd_vel_topic:="$TOPIC"
