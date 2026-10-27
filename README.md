# Nav2Tidy — TidyBot++ 홀로노믹 베이스용 MPPI 로컬 플래너

전방향(holonomic) 베이스를 위한 MPPI 로컬 플래너와, 그것을 ROS 2 Humble 위에 올린
스택이다. 두 가지로 쓴다:

- **시뮬** — Gazebo + Nav2. Nav2 의 MPPI 컨트롤러와 성능을 비교하는 벤치마크가 들어 있다.
- **실기** — 측위·costmap 은 동료 PC 의 FAST-LIO 스택에서 받고, 우리는 `/cmd_vel` 만 낸다.

전역 경로는 Nav2 의 `planner_server`(A\*, SmacPlanner2D)에서 받고, 그 경로를 따라가는
지역 제어가 이 저장소의 본체다 (`nav_mppi_docker/ws/nav/mppi.py`).

---

## 1. 준비 — 호스트에 필요한 것

```sh
docker --version          # 20.10 이상
docker compose version    # v2 (docker-compose-plugin)
docker info               # 권한 오류가 나면 아래
```

`docker info` 에서 권한 오류가 나면:

```sh
sudo usermod -aG docker $USER && newgrp docker
```



## 2. 설치

```sh
git clone https://github.com/moon0108/Nav2Tidy.git
cd Nav2Tidy/nav_mppi_docker
./setup.sh
```

## 3. 시뮬 돌려보기

```sh
docker exec -it nav_mppi bash          # 컨테이너 안으로. ROS 환경은 .bashrc 가 잡는다
```

```sh
# 베이스만 (Gazebo + 로봇). cmd_vel 로 밀어 보는 데 쓴다
ros2 launch tidybot_sim sim.launch.py

# Nav2 스택까지. controller 로 세 가지 구성을 고른다
ros2 launch tidybot_sim nav2.launch.py controller:=nav2    # Nav2 의 MPPI (baseline)
ros2 launch tidybot_sim nav2.launch.py controller:=mppi    # MPPI, Nav2 BT 아래
ros2 launch tidybot_sim nav2.launch.py controller:=team    # Team MPPI, BT 없이 액션으로

# GUI 가 없거나 느릴 때
ros2 launch tidybot_sim nav2.launch.py controller:=team headless:=true rviz:=false
```

주요 인자:

| 인자 | 값 | 뜻 |
|---|---|---|
| `controller` | `nav2` \| `mppi` \| `team` | 스택 구성. 기본 `nav2` |
| `scenario` | `corridor`, `clutter`, `doorway`, … | 월드. 기본 `corridor` |
| `localization` | `gt` \| `fastlio_mock` | Gazebo 참값 또는 FAST-LIO 흉내 TF 사슬 |
| `headless` / `rviz` | `true` \| `false` | GUI |
| `mppi_overrides` | `"v_max_xy=0.5 w_head=2.0"` | `MPPIConfig` 를 문자열로 덮는다 |
| `team_overlay` | 파일 이름 | `controller:=team` 에서 params 오버레이를 얹는다 |

`controller:=team` 은 목표를 액션으로 받는다. 보내는 쪽은:

```sh
ros2 run tidybot_sim mock_explorer.py --listen-goal-pose   # RViz 의 2D Goal Pose 를 받는다
```

벤치마크:

```sh
ros2 run tidybot_sim benchmark.py          # 세 컨트롤러 x 시나리오, results/ 에 CSV
```

## 4. 실기 모드

측위(`/map`, TF `map→base_link`)와 장애물 costmap 은 **동료 PC 의 FAST-LIO 스택**이
낸다. 이 저장소는 전역 플래너와 `/cmd_vel` 만 담당한다. 그래서 실기 모드는 상대 쪽
스택이 떠 있어야 동작한다.

```sh
# 모든 터미널 공통 -- RMW 를 zenoh 로 바꾸고 팀 도메인(25)으로 옮긴다
source /home/moon/ws/src/tidybot_sim/scripts/env_team.sh

# 1) zenoh 라우터. **정확히 한 PC 에서만** 띄운다 (포트 7447 충돌)
source /home/moon/ws/src/tidybot_sim/scripts/env_team.sh
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<라우터IP>:7447"]'
ros2 run rmw_zenoh_cpp rmw_zenohd

# 2) 전역 플래너 (/global_costmap/costmap 과 /compute_path_to_pose 를 낸다)
source /home/moon/ws/src/tidybot_sim/scripts/env_team.sh
ros2 launch tidybot_sim real_planner.launch.py

# 3) MPPI 로컬 플래너.
source /home/moon/ws/src/tidybot_sim/scripts/env_team.sh
ros2 run tidybot_sim navigate_to_goal.py --ros-args \
  --params-file .../team_real.yaml -p cmd_vel_topic:=/cmd_vel

# 4) 목표를 보내는 쪽 (RViz 의 2D Goal Pose)
source /home/moon/ws/src/tidybot_sim/scripts/env_team.sh
ros2 run tidybot_sim mock_explorer.py --listen-goal-pose
```

라우터를 띄우지 않는 PC 는 이렇게 붙는다:

```sh
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<라우터IP>:7447"]'
```

## 5. 코드 구조

```
nav_mppi_docker/
  Dockerfile              ROS Humble + Nav2 + 두 RMW + numpy/scipy
  compose.yaml            서비스 정의. GPU 없이 돌아간다
  compose.gpu.yaml        NVIDIA 오버레이 (setup.sh 가 있을 때만 붙인다)
  setup.sh                한 번에 환경 맞추기
  ws/
    nav/                  MPPI 본체. 순수 numpy/scipy, ROS 비의존
      mppi.py             샘플링 MPPI (K=512, T=25). Biased sampling 포함
      astar.py grid.py scene.py metrics.py
      runner.py viz.py benchmark.py sweep.py    MuJoCo 하네스
      MPPI.md NOTES.md    유도식과 측정 기록
      tests/              
    src/
      tidybot_sim/        ROS 패키지
        scripts/
          navigate_to_goal.py   액션 프론트엔드 (athome_interfaces/NavigateToGoal)
          executor.py           목표 하나를 끝까지 책임지는 상태기계
          mppi_controller.py    Nav2 FollowPath 프론트엔드
          mock_explorer.py      목표 송신
          fastlio_mock.py       실기와 같은 TF 사슬을 시뮬에서 만든다
          benchmark.py gen_world.py
        config/           params. team_real.yaml 이 실기 프로필
        launch/           sim / nav2 / real_planner
        ARCHITECTURE.md   설계와 측정 기록
      athome_interfaces/  NavigateToGoal 액션 정의
```
