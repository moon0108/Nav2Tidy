#!/usr/bin/env bash
# 환경을 한 번에 맞춘다. 호스트에 필요한 것은 docker 와 docker compose 뿐이다
# (ROS 를 호스트에 깔지 않는다).
#
#   ./setup.sh            이미지 빌드 -> 컨테이너 기동 -> 워크스페이스 빌드 -> 자가진단
#   ./setup.sh --no-build 이미지 빌드를 건너뛴다 (이미 있을 때)
#   ./setup.sh --rebuild  이미지를 캐시 없이 다시 만든다
#
# 멱등이다. 여러 번 돌려도 된다.
set -euo pipefail
cd "$(dirname "$0")"

say()  { printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok()   { printf '   \033[32mOK\033[0m   %s\n' "$*"; }
warn() { printf '   \033[33m주의\033[0m %s\n' "$*"; }
die()  { printf '   \033[31m실패\033[0m %s\n' "$*" >&2; exit 1; }

BUILD=1; NOCACHE=
for a in "$@"; do
  case "$a" in
    --no-build) BUILD=0 ;;
    --rebuild)  NOCACHE=--no-cache ;;
    -h|--help)  sed -n '2,11p' "$0"; exit 0 ;;
    *) die "모르는 인자: $a" ;;
  esac
done

# ── 1. 호스트 전제 ───────────────────────────────────────────────────────────
say "1/5  호스트 확인"
command -v docker >/dev/null || die "docker 가 없다.  https://docs.docker.com/engine/install/"
docker compose version >/dev/null 2>&1 \
  || die "docker compose v2 가 없다 (docker-compose-plugin 패키지)."
docker info >/dev/null 2>&1 \
  || die "docker 데몬에 접근할 수 없다. sudo usermod -aG docker \$USER 후 재로그인."
ok "docker $(docker version -f '{{.Server.Version}}'), compose $(docker compose version --short)"

# bind mount 한 ws/ 에 컨테이너가 파일을 쓴다. UID 가 다르면 colcon build 가
# 권한으로 막히므로 호스트 값을 그대로 이미지에 넣는다.
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
ok "UID/GID ${HOST_UID}/${HOST_GID} 로 빌드한다"

# GPU 는 **선택**이다. nvidia 장치 예약이 toolkit 없는 PC 에서 compose 를 실패시키므로
# 오버레이 파일로 분리해 두었고, 여기서 있을 때만 붙인다.
FILES=(-f compose.yaml)
if docker info 2>/dev/null | grep -qi 'Runtimes:.*nvidia'; then
  FILES+=(-f compose.gpu.yaml); ok "nvidia runtime 감지 -- GPU 오버레이를 붙인다"
else
  warn "nvidia runtime 없음 -- GPU 없이 돈다. Gazebo/RViz 는 소프트웨어 렌더링."
fi

if [ -z "${DISPLAY:-}" ]; then
  warn "DISPLAY 가 비어 있다. RViz·Gazebo GUI 는 못 띄운다 (headless 는 된다)."
fi

# ── 2. 이미지 ────────────────────────────────────────────────────────────────
if [ "$BUILD" = 1 ]; then
  say "2/5  이미지 빌드 (처음이면 10~20 분, 약 8 GB 내려받는다)"
  docker compose "${FILES[@]}" build $NOCACHE
  ok "nav_mppi:humble"
else
  say "2/5  이미지 빌드 건너뜀 (--no-build)"
  docker image inspect nav_mppi:humble >/dev/null 2>&1 || die "nav_mppi:humble 이 없다. --no-build 를 빼고 다시 돌려라."
fi

# ── 3. 컨테이너 ──────────────────────────────────────────────────────────────
say "3/5  컨테이너 기동"
docker compose "${FILES[@]}" up -d
for _ in $(seq 20); do
  [ "$(docker inspect -f '{{.State.Running}}' nav_mppi 2>/dev/null)" = true ] && break
  sleep 0.5
done
[ "$(docker inspect -f '{{.State.Running}}' nav_mppi)" = true ] || die "컨테이너가 안 떴다: docker compose ${FILES[*]} logs"
ok "nav_mppi 실행 중"

# ── 4. 워크스페이스 빌드 ─────────────────────────────────────────────────────
# --symlink-install: config/ 와 scripts/ 가 install 트리에 심링크로 들어가서
# 설정이나 파이썬을 고칠 때 재빌드가 필요 없다. C++ 은 없다.
say "4/5  colcon build (--symlink-install)"
docker exec nav_mppi bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  cd /home/moon/ws
  colcon build --symlink-install
' || die "colcon build 실패 (위 출력을 보라)"
ok "athome_interfaces + tidybot_sim"

# ── 5. 자가진단 ──────────────────────────────────────────────────────────────
say "5/5  자가진단"
docker exec nav_mppi bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  source /home/moon/ws/install/setup.bash
  python3 - <<PY
import importlib, sys
bad = []
for m in ("numpy", "scipy", "yaml"):
    try: importlib.import_module(m)
    except Exception as e: bad.append(f"{m}: {e}")
# nav 패키지는 PYTHONPATH=/home/moon/ws 로 잡힌다 (colcon 패키지가 아니다)
for m in ("nav.mppi", "nav.astar", "nav.grid", "nav.scene", "nav.metrics"):
    try: importlib.import_module(m)
    except Exception as e: bad.append(f"{m}: {e}")
from nav.mppi import MPPIConfig
c = MPPIConfig()
print(f"   nav.mppi  K={c.K} T={c.T} dt={c.dt} v_max={c.v_max}")
if bad:
    print("   임포트 실패:", *bad, sep="\n     "); sys.exit(1)
PY
  for t in test_override test_ancillary; do
    python3 /home/moon/ws/install/tidybot_sim/lib/tidybot_sim/$t.py >/dev/null \
      && echo "   회귀 테스트 $t 통과" || { echo "   회귀 테스트 $t 실패"; exit 1; }
  done
  ros2 pkg list 2>/dev/null | grep -qx athome_interfaces || { echo "   athome_interfaces 없음"; exit 1; }
  echo "   athome_interfaces 등록됨"
' || die "자가진단 실패"

printf '\n\033[1;32m준비 끝.\033[0m 다음 단계는 README.md 를 보라.\n\n'
printf '  컨테이너 들어가기 :  docker exec -it nav_mppi bash\n'
printf '  시뮬 돌려보기     :  ros2 launch tidybot_sim sim.launch.py\n'
printf '  내리기            :  docker compose %s down\n\n' "${FILES[*]}"
