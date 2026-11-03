#!/usr/bin/env bash
# 호스트에서 한 줄로 실기 스택을 띄운다. 컨테이너를 깨우고 안에서 real_up.sh 를 돈다.
# zenoh 라우터는 띄우지 않는다 -- 동료 스택이 띄우는 것을 쓴다.
#
#   ./run_real.sh          -> /cmd_vel_dryrun (건식. 로봇은 움직이지 않는다)
#   ./run_real.sh --arm    -> /cmd_vel (실제로 움직인다)
set -euo pipefail

docker start nav_mppi >/dev/null || {
    echo "nav_mppi 컨테이너가 없다. ./setup.sh 를 먼저 돌려라" >&2; exit 1; }

exec docker exec -it nav_mppi \
    /home/moon/ws/src/tidybot_sim/scripts/real_up.sh "$@"
