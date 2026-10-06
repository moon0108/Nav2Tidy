#!/bin/sh
# 이 워크스페이스가 띄운 것을 전부 내린다.
#
# **이름 목록으로 고르지 않는다.** 노드 이름을 하나 빠뜨리면 그 노드만 살아남고,
# 증상은 엉뚱한 곳에서 나타난다: "action server 가 두 개", lifecycle 이 unconfigured
# 에서 멈춤, /clock 이 여러 브리지에서 비단조 스탬프로 도착, 이유 없이 실패하는 goal.
# 이 프로젝트에서 네 번 당했다 -- navigate_to_goal, fastlio_mock, 검증 잔류 프로세스,
# nav2_costmap_2d. 목록을 고치는 것으로는 다섯 번째가 막히지 않는다.
#
# 구조로 고른다. 두 패스가 서로를 덮는다:
#
#   (1) `ros2 launch tidybot_sim ...` 의 **프로세스 그룹**을 통째로 죽인다. launch 가
#       띄운 자식은 이름과 무관하게 같은 그룹에 있으므로, 새 노드를 launch 에 추가해도
#       이 스크립트는 고칠 필요가 없다. 파라미터를 파일이 아니라 dict 로 받는 노드
#       (real_planner.launch.py 의 lifecycle_manager)와 경로를 안 지니는 노드
#       (static_transform_publisher, parameter_bridge)가 여기서 잡힌다.
#
#   (2) 명령줄이 **이 워크스페이스의 tidybot_sim 경로**를 가리키는 프로세스를 죽인다.
#       launch 없이 손으로 띄운 노드(ros2 run tidybot_sim ..., python3 scripts/...,
#       rviz2 -d .../config/real.rviz)와, 그룹을 벗어나 고아가 된 잔류분(ign gazebo 는
#       월드 sdf 경로를 지닌다)이 여기서 잡힌다.
#
# 죽이지 않는 것:
#   rmw_zenohd  라우터는 스택과 수명이 다르다. 스택을 다시 띄우는 동안 살아 있어야 한다.
#   이 스크립트 자신과 그 조상  경로 패턴이 stop.sh 자신의 경로에도 걸리기 때문에
#              (ws/src/tidybot_sim/scripts/stop.sh), 보호하지 않으면 자살한다.
set -u
PAT='ws/(install|src)/tidybot_sim'

# 보호 목록: 나 자신과 모든 조상. 호출자가 `bash -c '... stop.sh'` 처럼 경로를 명령줄에
# 지니고 있으면 (2) 에 걸린다.
protect=" $$ "
p=$(ps -o ppid= -p $$ 2>/dev/null | tr -d ' ')
while [ -n "${p:-}" ] && [ "$p" -gt 1 ] 2>/dev/null; do
    protect="$protect$p "
    p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
done

# (1) launch 프로세스 그룹
for pid in $(pgrep -f 'ros2 launch tidybot_sim' 2>/dev/null); do
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    # pgid 는 그룹 리더의 pid 다. 그것이 내 조상 중 하나라면 나도 그 그룹 안에 있다는
    # 뜻이므로 건너뛴다 -- 안 그러면 그룹을 죽이면서 나를 죽인다.
    case "$protect" in *" $pgid "*) continue ;; esac
    [ -n "$pgid" ] && kill -9 "-$pgid" 2>/dev/null
done

# (2) 이 워크스페이스를 가리키는 프로세스. 그룹 종료가 끝날 틈을 준 뒤 두 번 훑는다
# -- 첫 패스에서 죽은 부모가 자식을 고아로 남기면 그 자식은 두 번째 패스에서 걸린다.
#
# (2b) 상대 경로로 띄운 python 스크립트는 명령줄에 워크스페이스 경로가 없다
#      (`cd ~/ws/src/tidybot_sim && python3 -u scripts/mock_explorer.py` 가 문서상
#      사용법이다). 그때는 **cwd** 를 본다 -- 패키지 안에서 돌고 있는 .py 는 이
#      워크스페이스의 것이다. 노드 이름을 쓰지 않는다는 점이 중요하다.
candidates() {
    pgrep -f "$PAT" 2>/dev/null
    for pid in $(pgrep -f '[Pp]ython3?' 2>/dev/null); do
        cwd=$(readlink "/proc/$pid/cwd" 2>/dev/null) || continue
        case "$cwd" in
            */ws/src/tidybot_sim*|*/ws/install/tidybot_sim*) ;;
            *) continue ;;
        esac
        tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q '\.py' && echo "$pid"
    done
}

n=0
while [ "$n" -lt 2 ]; do
    for pid in $(candidates | sort -u); do
        case "$protect" in *" $pid "*) continue ;; esac
        kill -9 "$pid" 2>/dev/null
    done
    n=$((n + 1))
    [ "$n" -lt 2 ] && sleep 1
done

# 남은 것을 보고한다. 조용히 실패하는 것이 이 스크립트의 원래 문제였다.
#
# 목록을 ps 출력으로 직접 만든다. pgrep 으로 pid 를 모아 두고 나중에 ps 하면, 그 사이에
# 죽은 프로세스가 "남았다"로 보고되어 **목록은 비었는데 rc=1** 이 된다 (실제로 겪었다).
# kill -9 가 반영될 틈을 주려고 한 번 쉰다.
sleep 1
left=''
for pid in $(candidates | sort -u); do
    case "$protect" in *" $pid "*) continue ;; esac
    line=$(ps -o pid=,args= -p "$pid" 2>/dev/null | cut -c1-120)
    [ -n "$line" ] && left="$left$line
"
done
if [ -n "$left" ]; then
    echo "stop.sh: 남은 프로세스가 있다:" >&2
    printf '%s' "$left" >&2
    exit 1
fi
exit 0
