# Nav2Tidy — TidyBot++ 홀로노믹 베이스용 MPPI 로컬 플래너

전방향(holonomic) 베이스를 위한 MPPI 로컬 플래너와, 그것을 ROS 2 Humble 위에 올린
스택이다. 두 가지로 쓴다:

- **시뮬** — Gazebo + Nav2. Nav2 의 MPPI 컨트롤러와 성능을 비교하는 벤치마크가 들어 있다.
- **실기** — 측위·costmap 은 동료 PC 의 FAST-LIO 스택에서 받고, 우리는 `/cmd_vel` 만 낸다.

전역 경로는 Nav2 의 `planner_server`(A\*, SmacPlanner2D)에서 받고, 그 경로를 따라가는
지역 제어가 이 저장소의 본체다 (`nav_mppi_docker/ws/nav/mppi.py`).

---

## 1. 준비 — 호스트에 필요한 것

**Docker 와 Docker Compose v2 뿐이다.** ROS 를 호스트에 설치하지 않는다.

```sh
docker --version          # 20.10 이상
docker compose version    # v2 (docker-compose-plugin)
docker info               # 권한 오류가 나면 아래
```

`docker info` 에서 권한 오류가 나면:

```sh
sudo usermod -aG docker $USER && newgrp docker
```

GPU 는 **선택**이다. NVIDIA + [container toolkit][nct] 이 있으면 `setup.sh` 가 자동으로
붙인다. 없으면 그냥 돌아가고 Gazebo·RViz 가 소프트웨어 렌더링으로 떨어진다. 실기
모니터링은 Gazebo 를 쓰지 않으므로 GPU 가 없어도 아무 지장이 없다.

[nct]: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

## 2. 설치 — 한 번에

```sh
git clone <이 저장소 URL> Nav2Tidy
cd Nav2Tidy/nav_mppi_docker
./setup.sh
```

`setup.sh` 가 하는 일: 호스트 전제 확인 → 이미지 빌드 → 컨테이너 기동 →
`colcon build --symlink-install` → 자가진단(파이썬 의존성, `nav` 패키지 임포트,
회귀 테스트 2개, `athome_interfaces` 등록). **멱등이라 여러 번 돌려도 된다.**

처음 한 번은 베이스 이미지(`osrf/ros:humble-desktop-full`)를 내려받느라
**10~20 분, 약 8 GB** 가 걸린다. 그다음부터는 수 초다.

```sh
./setup.sh --no-build   # 이미지가 이미 있을 때
./setup.sh --rebuild    # 캐시 없이 이미지를 다시 만든다
```

통과하면 이렇게 끝난다:

```
== 5/5  자가진단
   nav.mppi  K=512 T=25 dt=0.1 v_max=(0.5, 0.5, 1.0)
   회귀 테스트 test_override 통과
   회귀 테스트 test_ancillary 통과
   athome_interfaces 등록됨
준비 끝.
```

## 3. 시뮬 돌려보기

```sh
docker exec -it nav_mppi bash          # 컨테이너 안으로. ROS 환경은 .bashrc 가 잡는다
```

```sh
# 베이스만 (Gazebo + 로봇). cmd_vel 로 밀어 보는 데 쓴다
ros2 launch tidybot_sim sim.launch.py

# Nav2 스택까지. controller 로 세 가지 구성을 고른다
ros2 launch tidybot_sim nav2.launch.py controller:=nav2    # Nav2 의 MPPI (기준선)
ros2 launch tidybot_sim nav2.launch.py controller:=mppi    # 우리 MPPI, Nav2 BT 아래
ros2 launch tidybot_sim nav2.launch.py controller:=team    # 우리 MPPI, BT 없이 액션으로

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
ros2 run rmw_zenoh_cpp rmw_zenohd

# 2) 전역 플래너 (/global_costmap/costmap 과 /compute_path_to_pose 를 낸다)
ros2 launch tidybot_sim real_planner.launch.py

# 3) MPPI 로컬 플래너. 기본은 드라이런이다
ros2 run tidybot_sim navigate_to_goal.py --ros-args \
  --params-file /home/moon/ws/install/tidybot_sim/share/tidybot_sim/config/team_real.yaml

# 4) 목표를 보내는 쪽 (RViz 의 2D Goal Pose)
ros2 run tidybot_sim mock_explorer.py --listen-goal-pose
```

라우터를 띄우지 않는 PC 는 이렇게 붙는다:

```sh
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<라우터IP>:7447"]'
```

### 실제로 로봇을 움직이려면 — 무장

기본 `cmd_vel_topic` 은 `/cmd_vel_dryrun` 이다. **같은 망에 실제 로봇이 있을 때
시뮬이나 시험 주행이 로봇을 밀지 않게 하는 장치다.** 로봇을 움직이려면 명시적으로
바꿔야 한다:

```sh
ros2 topic info /cmd_vel          # 먼저 Publisher 0 / Subscription 1 을 확인한다
ros2 run tidybot_sim navigate_to_goal.py --ros-args \
  --params-file .../team_real.yaml -p cmd_vel_topic:=/cmd_vel
```

`controller_server` 와 `bt_navigator` 는 실기 모드에서 **일부러 띄우지 않는다** —
그 둘은 목표가 없어도 `/cmd_vel` 발행자를 만들어 위 확인을 무의미하게 만들고,
`/follow_path` 에 목표가 들어가면 실제로 속도를 낸다.

자세한 절차는 `nav_mppi_docker/ws/src/tidybot_sim/scripts/check_team_dryrun.md`.

## 5. 저장소 구조

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
      runner.py viz.py benchmark.py sweep.py    MuJoCo 하네스 (아래 주의)
      MPPI.md NOTES.md    유도식과 측정 기록
      tests/              pytest
    src/
      tidybot_sim/        ROS 패키지
        scripts/
          navigate_to_goal.py   액션 프론트엔드 (athome_interfaces/NavigateToGoal)
          executor.py           목표 하나를 끝까지 책임지는 상태기계
          mppi_controller.py    Nav2 FollowPath 프론트엔드 (벤치마크용, 동결)
          mock_explorer.py      목표를 보내는 쪽. 실제 탐색 시스템을 흉내 낸다
          fastlio_mock.py       실기와 같은 TF 사슬을 시뮬에서 만든다
          benchmark.py gen_world.py
        config/           params. team_real.yaml 이 실기 프로필
        launch/           sim / nav2 / real_planner
        ARCHITECTURE.md   설계와 측정 기록 (긴 문서)
      athome_interfaces/  NavigateToGoal 액션 정의 (임시 — 팀 원문 수령 시 교체)
```

### 알아 둘 것

- **컨테이너 안의 사용자는 호스트와 무관하게 항상 `moon`** 이고 작업 경로는
  `/home/moon/ws` 다. `compose.yaml` 의 `CYCLONEDDS_URI` · `PYTHONPATH` 와 묶여
  있어서, 바꾸려면 `Dockerfile` 의 `USERNAME` 까지 세 곳을 같이 고쳐야 한다.
  UID/GID 만 호스트 값으로 맞춘다(`setup.sh` 가 넘긴다) — 다르면 bind mount 한
  `ws/` 에 컨테이너가 쓴 파일 소유자가 어긋나 `colcon build` 가 막힌다.

- **`ws/nav` 는 colcon 패키지가 아니다.** `package.xml` 이 없어서 colcon 이 무시하고,
  `PYTHONPATH=/home/moon/ws` 로 잡힌다. 그래서 고쳐도 재빌드가 필요 없다.

- **`--symlink-install`** 이므로 `config/` 의 yaml 과 `scripts/` 의 파이썬은 고치는
  즉시 반영된다. 재빌드가 필요한 것은 `CMakeLists.txt` 와 `athome_interfaces` 뿐이다.

- **`ws/nav/{runner,viz,benchmark,sweep}.py` 는 MuJoCo 하네스**이고 이 이미지에서는
  돌지 않는다. `runner.py` 가 TidyBot++ 저장소 루트의 `constants.py` · `mujoco_env.py`
  를 임포트하기 때문이다. ROS 스택이 쓰는 것은 `mppi / astar / grid / scene / metrics`
  다섯 개뿐이고 그 폐쇄집합의 서드파티 의존성은 numpy 와 scipy 둘뿐이다.

- **`nav/` 의 출처.** [jimmyyhwu/tidybot2][ub] (MIT) 의 워크트리에서 개발하다 이
  저장소로 옮겼다. `nav/` 자체는 상위 코드를 포함하지 않는 새 디렉터리다.

[ub]: https://github.com/jimmyyhwu/tidybot2

## 6. 자주 막히는 곳

| 증상 | 원인 |
|---|---|
| `docker compose up` 이 nvidia 로 실패 | `compose.gpu.yaml` 을 수동으로 붙였다. `setup.sh` 를 쓰면 자동 판별한다 |
| `ModuleNotFoundError: nav` | `PYTHONPATH=/home/moon/ws` 가 빠졌다. `docker exec -it nav_mppi bash` 로 들어가면 compose 가 넣어 준다 |
| `colcon build` 가 권한 오류 | 호스트 UID 가 1000 이 아닌데 이미지를 다른 값으로 빌드했다. `./setup.sh --rebuild` |
| `rmw_create_node: failed to create domain` | CycloneDDS 참가자 수 한계(기본 9). `CYCLONEDDS_URI` 가 빠졌는지 보라 |
| RViz·Gazebo 가 안 뜬다 | `DISPLAY` 가 비었거나 `xhost` 가 막았다. `headless:=true rviz:=false` 로 피해 간다 |
| 실기에서 goal 이 바로 실패 | 동료 PC 스택이 떠 있는지, 라우터가 **한 곳에서만** 도는지 (`ss -tn \| grep 7447`) |

## 7. 테스트

```sh
docker exec -it nav_mppi bash -lc '
  cd /home/moon/ws && python3 -m pytest nav/tests -q
  python3 install/tidybot_sim/lib/tidybot_sim/test_override.py
  python3 install/tidybot_sim/lib/tidybot_sim/test_ancillary.py
'
```
