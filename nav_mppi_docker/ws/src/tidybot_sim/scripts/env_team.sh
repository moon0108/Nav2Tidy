# 팀(동료 PC) 연결용 환경. **source 해서 쓴다** -- 실행하면 아무 효과가 없다.
#
#   source ~/ws/src/tidybot_sim/scripts/env_team.sh
#
# 새 터미널마다 이걸 빠뜨려서 생기는 증상은 전부 "조용히 아무것도 안 보임"이다:
# RMW 가 다르면 노드가 서로를 못 보고, 도메인이 다르면 zenoh 키가 달라 역시 안 보인다.
# 그래서 마지막에 현재 값을 한 줄로 찍는다.
#
# 주의: 라우터(rmw_zenohd)는 호스트와 컨테이너 중 **한 곳에서만** 떠야 한다.
# 둘 다 띄우면 포트 7447 이 충돌한다.

if [ -n "${BASH_SOURCE[0]}" ] && [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "이 파일은 실행하는 것이 아니라 source 해야 한다:" >&2
    echo "    source $0" >&2
    exit 1
fi

. /opt/ros/humble/setup.bash
[ -f "$HOME/ws/install/setup.bash" ] && . "$HOME/ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=25
# zenoh 에서는 효과가 없는 변수지만, CycloneDDS 를 쓰던 설정이 남아 있으면 혼란스럽다.
unset ROS_LOCALHOST_ONLY

echo "team 환경: RMW=${RMW_IMPLEMENTATION}  DOMAIN=${ROS_DOMAIN_ID}  LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"
