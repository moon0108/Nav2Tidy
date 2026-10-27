# 실기 dry-run 절차

동료 PC 의 SLAM/costmap 과 같은 도메인에서 MPPI 를 돌리되, **로봇은 움직이지 않는**
상태로 전 경로를 확인한다. `cmd_vel_topic` 기본값이 `/cmd_vel_dryrun` 이라 안전하다.

전제: 동료 PC 에서 SLAM·costmap·구동 노드가 떠 있을 것. (2026-10-02~03 실측 기준)

---

## 0. 라우터 (한 번만, 한 곳에서만)

```sh
# 컨테이너 안
ros2 run rmw_zenoh_cpp rmw_zenohd
```

다른 터미널에서 동료 라우터에 연결:

```sh
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<동료 IP>:7447"]' \
  ros2 run rmw_zenoh_cpp rmw_zenohd
```

**라우터는 호스트와 컨테이너 중 한 곳에서만 띄운다** — 둘 다면 7447 이 충돌한다.
연결 확인:

```sh
ss -tn | grep ':7447'      # 동료 IP 와 ESTAB 이 보여야 한다
```

## 1. 모든 터미널 공통

```sh
source ~/ws/src/tidybot_sim/scripts/env_team.sh
# team 환경: RMW=rmw_zenoh_cpp  DOMAIN=25  LOCALHOST_ONLY=unset
```

이 줄을 빠뜨리면 **아무것도 안 보인다**. RMW 가 다르면 노드가 서로를 못 보고,
도메인이 다르면 zenoh 키가 달라 역시 안 보인다. 증상이 똑같아서 구분이 안 된다.

## 2. 터미널 A — planner

```sh
ros2 launch tidybot_sim real_planner.launch.py
```

`planner_server` + `lifecycle_manager_nav` 만 뜬다. `map_server` 는 동료 것을 쓰고,
`controller_server` 와 `bt_navigator` 는 **일부러 띄우지 않는다** (goal 이 없어도
`/cmd_vel` 발행자를 만들어 "발행자 1개" 조건을 깬다).

## 3. 터미널 B — MPPI (dry-run)

```sh
ros2 run tidybot_sim navigate_to_goal.py --ros-args \
  --params-file ~/ws/install/tidybot_sim/share/tidybot_sim/config/team_real.yaml
```

기동 로그에 이 두 줄이 보여야 한다:

```
MPPIConfig overridden: v_max_xy=0.2 v_max_wz=0.6
/athome/navigate_to_goal up: K=512, T=25, dt=0.1, solve 10.0 Hz, publish 10.0 Hz, ...
```

## 4. 터미널 C — 목표 입력

```sh
cd ~/ws/src/tidybot_sim
python3 -u scripts/mock_explorer.py --listen-goal-pose
```

`-u` 는 출력 버퍼링을 끈다. 로그를 파일로 넘길 때 이게 없으면 비어 보인다.

planner 없이 직선 경로로 시험하려면 `--straight-path` 를 붙인다. 그 경로는 장애물을
전혀 보지 않으므로 벽을 관통하면 MPPI 가 회피하다 BLOCKED 나 NO_PROGRESS 로 끝난다.

## 5. 터미널 D — RViz

```sh
rviz2 -d ~/ws/install/tidybot_sim/share/tidybot_sim/config/real.rviz
```

"2D Goal Pose" 로 찍으면 `/goal_pose` 가 나가고 터미널 C 가 받는다.

---

## 확인 항목

| 확인 | 명령 | 기대 |
|---|---|---|
| 동료 노드 | `ros2 node list` | `laser_mapping`, `transform_fusion`, `map_server`, 두 costmap |
| TF | `ros2 run tf2_ros tf2_echo map base_link` | 값이 나올 것 |
| 두 costmap | `ros2 topic hz /mppi_static_costmap/costmap` / `/local_costmap/costmap` | 약 0.65 Hz / 3.3 Hz |
| 우리 costmap | `ros2 topic hz /global_costmap/costmap` | 약 0.8 Hz |
| **`/cmd_vel` 발행자** | `ros2 topic info /cmd_vel -v` | **0 개** (또는 `Unknown topic`) |
| dry-run 출력 | `ros2 topic hz /cmd_vel_dryrun` | 10 Hz |
| `input_wait` 통과 | 터미널 B 로그 | `입력 대기` 경고가 2 초 안에 사라질 것 |
| rollouts | RViz | 실제 costmap 위에 그려질 것 |

목표를 찍었을 때 터미널 C 에 이렇게 나오면 전 경로가 선 것이다:

```
/goal_pose 수신 (-5.61, +3.42, +0.00)
#1 drive    (-5.61, 3.42, +0.00)  116 poses
      phase -> DRIVING  (remaining 5.91 m)
      phase -> STOPPING (remaining 5.93 m)
      -> FAILED/NO_PROGRESS  stopped=true  10.5s
```

**dry-run 에서는 NO_PROGRESS 가 정상이다.** `/cmd_vel_dryrun` 으로 내보내므로 로봇이
안 움직이고, 따라서 경로 진척이 0 이다. 봐야 할 것은 `DRIVING` 에 들어갔는지와
`stopped=true` 인지다.

## 자주 걸리는 것

**경로가 안 나온다 (`status=6`, 0 점).** 로봇의 현재 셀이 static costmap 에서
lethal(99) 일 가능성이 높다. `robot_radius 0.37` 이라 벽에서 0.37 m 안이면 그렇게 된다.
확인:

```sh
ros2 run tf2_ros tf2_echo map base_link      # 위치를 보고 RViz 의 /map 과 대조
```

로봇이 지도상 벽 안에 있으면 `2D Pose Estimate` 로 정합을 다시 잡거나 로봇을 옮긴다.
실제로 한 번 겪었고, 옮기자 바로 해결됐다.

**자유 공간이 성분 17 개로 잘게 나뉘어 있다.** 로봇이 있는 성분 밖의 목표는 planner 가
경로를 못 낸다. `robot_radius 0.37 + inflation 0.40` 이면 폭 0.8 m 미만 통로가 전부
막히기 때문이다. 버그가 아니라 반경 설정의 결과다.

---

## 실제로 로봇을 움직일 때 (armed)

```sh
ros2 run tidybot_sim navigate_to_goal.py --ros-args \
  --params-file ~/ws/install/tidybot_sim/share/tidybot_sim/config/team_real.yaml \
  -p cmd_vel_topic:=/cmd_vel
```

**이 순간부터 실제 로봇이 움직인다.** 라우터가 동료와 연결된 상태이므로 `/cmd_vel` 이
그대로 구동 노드에 닿는다. 전에 반드시:

1. `ros2 topic info /cmd_vel -v` 로 구독자가 구동 노드 하나인지 확인
2. 비상 정지를 손에 둘 것
3. 바퀴를 들거나 주변을 비울 것

### 무선 경유의 한계 — 읽고 시작할 것

`team_real.yaml` 의 세 값은 **무선 경유 테스트 전용**이다.

```
tf_timeout 2.0 / local_stale 2.0 / static_stale 30.0
```

기본값(0.5 / 1.0 / 3.0)으로는 10 % 주기에서 abort 가 난다. 2026-10-02 실측:

```
TF map->base_link 나이   p50 0.279  p90 0.580  p99 1.364  최대 1.652 s
/local_costmap/costmap   3.27 Hz, 최대 간격 1.18 s
/mppi_static_costmap     0.65 Hz, 최대 간격 2.59 s
```

올려 두었다는 것은 **0.3~1.6 s 묵은 pose 로 제어한다**는 뜻이다. 0.5 m/s 에서 1.6 s 는
0.8 m 틀린 위치다. 그래서 `mppi_overrides` 로 속도를 (0.2, 0.2, 0.6) 으로 낮춰 둔다.

**실주행은 로봇 PC 에서 돌려야 한다.** 같은 PC 면 지연이 ~1 ms 로 떨어져 이 세 값을
원래대로 되돌릴 수 있고, 속도 제한도 풀 수 있다.
