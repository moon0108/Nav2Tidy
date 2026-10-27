# ROS 2 구조

`controller:=mppi` 기준. 노드 목록과 pub/sub은 실제로 스택을 띄워
`ros2 node list` / `ros2 node info`로 뜬 것이고, 나머지는 소스에서 정리했다.

## 실행

```sh
ros2 launch tidybot_sim nav2.launch.py controller:=mppi scenario:=clutter
python3 scripts/send_goal.py clutter 120
```

RMW 는 `rmw_cyclonedds_cpp` 이고 **라우터가 필요 없다** (팀 표준). 기본 실행은
`ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=42` 로 PC 밖에 나가지 않는다 — 15 절을 볼 것.

launch 인자: `scenario`(corridor/doorway/utrap/clutter), `controller`(nav2/mppi/**team**),
`rviz`, `headless`, `sim`, `mppi_overrides`.

`controller:=team` 은 팀 연동용 스택이다. 아래 1~12 절은 `controller:=mppi` 기준이고,
team 모드는 13 절에 따로 있다.

---

## 1. 실행 계층

```
nav2.launch.py                      <- 진입점
|-- sim.launch.py  (sim:=true일 때 include)
|   |-- ign gazebo            tidy.sdf          * ROS 노드 아님
|   |-- robot_state_publisher tidybot.urdf
|   |-- ros_gz_sim/create     (4초 뒤 스폰 후 종료)
|   |-- ros_gz_bridge         5개 토픽
|   |-- static_transform_publisher  map->odom
|   `-- rviz2                 (rviz:=true)
`-- Nav2 + 자체 플래너
    |-- map_server / planner_server / controller_server
    |-- behavior_server / bt_navigator
    |-- mppi_controller       (controller:=mppi일 때만)
    `-- lifecycle_manager_nav
```

`sim:=false`로 주면 이미 돌아가는 시뮬레이션에 Nav2만 붙는다.

---

## 2. 노드 (총 20개)

실사용 13개

| 노드 | 패키지 | 역할 |
|---|---|---|
| `robot_state_publisher` | robot_state_publisher | URDF -> `/tf_static` |
| `ros_gz_bridge` | ros_gz_bridge | Gazebo <-> ROS 5개 토픽 |
| `map_to_odom` | tf2_ros | `map->odom` 항등 static TF (AMCL 대체) |
| `map_server` | nav2_map_server | `maps/tidy.yaml` 서빙 |
| `planner_server` | nav2_planner | SmacPlanner2D 전역 경로 |
| `global_costmap/global_costmap` | nav2_costmap_2d | planner_server 소유 |
| `controller_server` | nav2_controller | FollowPath 비활성. local_costmap 소유용으로만 생존 |
| `local_costmap/local_costmap` | nav2_costmap_2d | controller_server 소유 |
| `behavior_server` | nav2_behaviors | spin / backup / drive_on_heading / wait |
| `bt_navigator` | nav2_bt_navigator | NavigateToPose BT |
| **`mppi_controller`** | **tidybot_sim** | **자체 MPPI, FollowPath 서버** |
| `lifecycle_manager_nav` | nav2_lifecycle_manager | 5개 노드 전이 관리 |
| `rviz` | rviz2 | nav2_default_view |

내부 7개 — `bt_navigator_navigate_{to,through}_pose*_rclcpp_node` 2개,
`rviz_navigation_dialog_action_client`, `transform_listener_impl_*` 4개.

---

## 3. 노드별 pub / sub

아래 표에서 생략한 것: `/parameter_events`, `/rosout`, `*_parameters` 서비스 6종은
모든 노드에 있고, `/clock`은 모든 노드가 구독한다(`use_sim_time: true`).

### 데이터 경로에 있는 노드

| 노드 | 구독 | 발행 |
|---|---|---|
| `ros_gz_bridge` | `/cmd_vel` | `/clock` `/odom` `/scan` `/tf` |
| `robot_state_publisher` | `/joint_states` | `/robot_description` `/tf` `/tf_static` |
| `map_to_odom` | - | `/tf_static` |
| `map_server` | `/bond` | `/map` `/bond` |
| `global_costmap` | `/map` `/scan` `/global_costmap/footprint` | `/global_costmap/costmap` `/global_costmap/costmap_raw` `/global_costmap/costmap_updates` `/global_costmap/published_footprint` |
| `local_costmap` | `/scan` `/local_costmap/footprint` | `/local_costmap/costmap` `/local_costmap/costmap_raw` `/local_costmap/costmap_updates` `/local_costmap/published_footprint` |
| `planner_server` | `/bond` | `/plan` `/unsmoothed_plan` `/bond` |
| `controller_server` | `/odom` `/speed_limit` `/bond` | `/cmd_vel` `/trajectories` `/transformed_global_plan` `/bond` |
| **`mppi_controller`** | **`/local_costmap/costmap` `/tf` `/tf_static`** | **`/cmd_vel` `/mppi_controller/rollouts`** |
| `behavior_server` | `/local_costmap/costmap_raw` `/local_costmap/published_footprint` `/bond` | `/cmd_vel` `/bond` |
| `bt_navigator` | `/goal_pose` `/odom` `/tf` `/tf_static` `/bond` | `/bond` |
| `lifecycle_manager_nav` | `/bond` | `/diagnostics` `/bond` |

`mppi_controller`가 `/odom`을 구독하지 않는 점에 주의. 로봇 pose는 전부 TF에서
읽는다. 오도메트리 토픽을 직접 쓰면 프레임이 암묵적으로 고정돼, AMCL을 넣는 순간
틀린 값을 읽게 된다.

### 내부 노드

| 노드 | 구독 | 발행 |
|---|---|---|
| `bt_navigator_navigate_{to,through}_pose*_rclcpp_node` | - | `/behavior_tree_log` |
| `transform_listener_impl_*` (4개) | `/tf` `/tf_static` | - |
| `rviz` | 아래 참조 | `/clicked_point` `/initialpose` |
| `rviz_navigation_dialog_action_client` | - | `/waypoints` |

`rviz`는 `nav2_default_view`를 그대로 쓰므로 이 스택에 없는 토픽도 구독한다.
발행자가 없는 것들: `/downsampled_costmap(_updates)`, `/local_plan`, `/map_updates`,
`/particle_cloud`(AMCL 없음), `/mobile_base/sensors/bumper_pointcloud`,
`/{local,global}_costmap/voxel_marked_cloud`(voxel 레이어 없음).
실제로 그림이 그려지는 것은 `/map`, `/scan`, `/plan`, 양쪽 costmap,
`/local_costmap/published_footprint`뿐이다.

`/mppi_controller/rollouts`는 기본 뷰에 없다. RViz에서 MarkerArray 디스플레이를
수동으로 추가해야 보인다.

---

## 4. 토픽과 메시지 형식

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | ros_gz_bridge | 전 노드 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | mppi_controller, controller_server, behavior_server | ros_gz_bridge |
| `/odom` | `nav_msgs/msg/Odometry` | ros_gz_bridge | bt_navigator, controller_server |
| `/scan` | `sensor_msgs/msg/LaserScan` | ros_gz_bridge | 양쪽 costmap, rviz |
| `/tf` | `tf2_msgs/msg/TFMessage` | ros_gz_bridge, robot_state_publisher | transform_listener x4, bt_navigator, mppi_controller |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | robot_state_publisher, map_to_odom | 위와 동일 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | map_server | global_costmap, rviz |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | local_costmap | **mppi_controller**, rviz |
| `/local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | local_costmap | behavior_server |
| `/local_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | local_costmap | rviz |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | local_costmap | behavior_server, rviz |
| `/local_costmap/footprint` | `geometry_msgs/msg/Polygon` | (없음) | local_costmap |
| `/global_costmap/*` | 위와 같은 4종 | global_costmap | rviz |
| `/plan` | `nav_msgs/msg/Path` | planner_server | rviz |
| `/unsmoothed_plan` | `nav_msgs/msg/Path` | planner_server | (없음) |
| `/transformed_global_plan` | `nav_msgs/msg/Path` | controller_server | (없음) |
| `/trajectories` | `visualization_msgs/msg/MarkerArray` | controller_server | (없음) |
| `/mppi_controller/rollouts` | `visualization_msgs/msg/MarkerArray` | mppi_controller | (RViz 수동 추가) |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | (없음) | bt_navigator |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | rviz | (없음, AMCL 부재) |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | rviz | (없음) |
| `/behavior_tree_log` | `nav2_msgs/msg/BehaviorTreeLog` | bt_navigator rclcpp 노드 2개 | (없음) |
| `/robot_description` | `std_msgs/msg/String` | robot_state_publisher | ros_gz_sim/create (스폰 시) |
| `/joint_states` | `sensor_msgs/msg/JointState` | (없음, 가동 관절 없음) | robot_state_publisher |
| `/speed_limit` | `nav2_msgs/msg/SpeedLimit` | (없음) | controller_server |
| `/waypoints` | `visualization_msgs/msg/MarkerArray` | rviz_navigation_dialog_action_client | rviz |
| `/bond` | `bond/msg/Status` | 라이프사이클 5종 + manager | 동일 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | lifecycle_manager_nav | (없음) |
| `/<node>/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 라이프사이클 노드 각각 | (없음) |

`/goal_pose`에 발행자가 없는 이유: RViz의 "2D Goal Pose"는 토픽이 아니라
NavigateToPose 액션 클라이언트를 쓴다. `send_goal.py`도 마찬가지다. 그 구독은
외부 도구용으로 열려 있을 뿐이다.

`/cmd_vel`은 이름 기준으로 발행 노드가 3개지만 실제 엔드포인트는 6개다
(`rp topic info /cmd_vel -v`). `behavior_server`가 behavior 플러그인마다 하나씩
만들기 때문이다.

### QoS가 중요한 곳

`mppi_controller`의 costmap 구독은 `nav2_costmap_2d`의 발행자와 맞춰야 한다.
안 맞으면 조용히 한 장도 못 받는다.

```python
QoSProfile(depth=1,
           reliability=ReliabilityPolicy.RELIABLE,
           durability=DurabilityPolicy.TRANSIENT_LOCAL)
```

`/cmd_vel` 발행은 depth 10 기본 QoS.

---

## 5. 브리지 형식 변환

`ros_gz_bridge`가 Gazebo Fortress 타입과 ROS 타입을 양쪽으로 옮긴다.
Fortress는 `ignition.msgs.*`, Harmonic은 `gz.msgs.*`다. 섞으면 안 된다.

| 토픽 | 방향 | ROS 타입 | Gazebo 타입 |
|---|---|---|---|
| `/clock` | gz -> ros | `rosgraph_msgs/msg/Clock` | `ignition.msgs.Clock` |
| `/odom` | gz -> ros | `nav_msgs/msg/Odometry` | `ignition.msgs.Odometry` |
| `/tf` | gz -> ros | `tf2_msgs/msg/TFMessage` | `ignition.msgs.Pose_V` |
| `/scan` | gz -> ros | `sensor_msgs/msg/LaserScan` | `ignition.msgs.LaserScan` |
| `/cmd_vel` | **ros -> gz** | `geometry_msgs/msg/Twist` | `ignition.msgs.Twist` |

브리지 인자에서 `[`가 gz->ros, `]`가 ros->gz다.

Gazebo 쪽 생산자:

| 플러그인 / 센서 | 내보내는 것 |
|---|---|
| `VelocityControl` | `cmd_vel`을 **모델 프레임**으로 받아 베이스에 적용 |
| `OdometryPublisher` | `odom` + `tf`, `odom_frame: odom`, `robot_base_frame: base_link`, 50 Hz, 2D |
| `gpu_lidar` | `scan`, 640 samples, 360도, 10 Hz, `ignition_frame_id: lidar_link` |

`ignition_frame_id`가 없으면 scan의 frame_id가 `tidybot/lidar_link/lidar`가 되고,
그런 이름의 TF는 없으므로 RViz가 모든 메시지를 버린다.

---

## 6. 데이터 흐름

```
        +----------- Gazebo Fortress (tidy.sdf) ------------+
        |  VelocityControl <- cmd_vel (모델 프레임)         |
        |  OdometryPublisher -> odom, tf   @50 Hz           |
        |  gpu_lidar -> scan               @10 Hz           |
        +------------------+--------------------------------+
                           |  ros_gz_bridge
   /clock  <--- gz         |
   /odom   <--- gz         |
   /tf     <--- gz         |
   /scan   <--- gz         |
   /cmd_vel ---> gz        |
                           v
  /scan --+--> local_costmap  (odom, 6x6 m rolling, 0.05 m) --> /local_costmap/costmap
          `--> global_costmap (map, static+obstacle+inflation) --> /global_costmap/costmap
                     ^                                                    |
  /map ------------- '                                                    |
   ^                                                                      v
 map_server                                              planner_server -> /plan
                                                                  |
                                            ComputePathToPose 액션 결과로 BT에 전달
                                                                  |
                                                                  v
                                              FollowPath 액션 goal (nav_msgs/Path)
                                                                  |
                                                                  v
                                                          mppi_controller
                                                                  |
                                           +----------------------+-----------------+
                                    /cmd_vel @30 Hz                   ~/rollouts (MarkerArray)
```

---

## 7. TF 트리

```
map
 `-- odom            <- map_to_odom (static, 항등)
      `-- base_link  <- Gazebo OdometryPublisher -> /tf @50 Hz
           `-- lidar_link  <- URDF fixed joint -> /tf_static
```

`map->odom`이 항등인 이유는 오도메트리가 ground truth이기 때문이다. AMCL이 없다.
그래도 `mppi_controller`는 모든 변환을 TF로 명시 조회한다. 나중에 AMCL을 끼워 넣어도
노드를 안 고치기 위해서다.

프레임 회전이 두 군데 있다.

- `_path_to()` — path(`map`) -> costmap(`odom`). 90여 개 pose를 개별 tf2 호출 대신
  강체 변환 한 번으로 처리.
- `_publish()` — `mppi.py`는 전역 프레임 `(vx, vy, w)`를 반환하는데 Gazebo
  VelocityControl은 모델 프레임을 받는다. 그래서 30 Hz마다 `-theta`로 회전.
  theta는 매번 새로 읽는다. 해를 구할 때 한 번 회전해두면 그 사이 회전한 만큼
  방향이 틀어지기 때문이다.

---

## 8. 액션

```
RViz "2D Goal Pose" / send_goal.py
        |  NavigateToPose
        v
   bt_navigator --+-- ComputePathToPose --> planner_server   (BT가 1 Hz로 재계산)
                  |-- FollowPath ---------> mppi_controller  <== 핵심 스위치
                  `-- Spin / BackUp / DriveOnHeading / Wait --> behavior_server
```

| 액션 | 서버 | 클라이언트 |
|---|---|---|
| `/navigate_to_pose` | bt_navigator | rviz, send_goal.py |
| `/navigate_through_poses` | bt_navigator | rviz |
| `/compute_path_to_pose` | planner_server | bt_navigator (to_pose rclcpp 노드) |
| `/compute_path_through_poses` | planner_server | bt_navigator (through_poses rclcpp 노드) |
| **`/follow_path`** | **mppi_controller** | bt_navigator rclcpp 노드 2개 |
| `/follow_path_unused` | controller_server | (없음) |
| `/spin` `/backup` `/drive_on_heading` `/wait` | behavior_server | bt_navigator rclcpp 노드 2개 |

### 페이로드 형식

**`nav2_msgs/action/FollowPath`** — 우리 노드가 서버인 것

```
goal:      nav_msgs/Path path          # 경로. frame_id는 보통 map
           string controller_id
           string goal_checker_id
---
result:    std_msgs/Empty              # 성공/실패는 goal status로만 전달
---
feedback:  float32 distance_to_goal
           float32 speed
```

`mppi_controller`는 피드백 두 필드를 매 solve마다 채운다
(`speed = hypot(vx, vy)`), 종료는 `succeed()` / `canceled()` / `abort()`로 알린다.

**`nav2_msgs/action/NavigateToPose`** — 바깥에서 들어오는 것

```
goal:      geometry_msgs/PoseStamped pose
           string behavior_tree
---
result:    std_msgs/Empty
---
feedback:  geometry_msgs/PoseStamped current_pose
           builtin_interfaces/Duration navigation_time
           builtin_interfaces/Duration estimated_time_remaining
           int16 number_of_recoveries
           float32 distance_remaining
```

**`nav2_msgs/action/ComputePathToPose`** — 경로가 만들어지는 곳

```
goal:      geometry_msgs/PoseStamped goal
           geometry_msgs/PoseStamped start
           string planner_id
           bool use_start                # false면 현재 pose를 시작점으로
---
result:    nav_msgs/Path path
           builtin_interfaces/Duration planning_time
```

### FollowPath 스왑

`controller:=mppi`일 때 `controller_server`의 FollowPath가 `follow_path_unused`로
밀려난다.

```python
FOLLOW_PATH_AWAY = [(f'follow_path/_action/{e}', f'follow_path_unused/_action/{e}')
                    for e in ('send_goal','cancel_goal','get_result','feedback','status')]
```

액션 이름만 리매핑하면 안 된다. `rcl_action`이 `<action>/_action/<entity>`를 만들기
때문에 하위 5개를 각각 옮겨야 한다. 검증: 바깥 이름만 리매핑한 상태에서
`ros2 action info /follow_path`가 여전히 서버 2개를 나열했다.

`controller_server`를 아예 안 띄우지 않는 이유는 `local_costmap`,
`published_footprint`, `clear_entirely_local_costmap`을 그 노드가 소유하기
때문이다. 각각 우리 플래너의 입력, behavior_server의 필수 입력, BT 복구 분기가
호출하는 서비스다. 대신 `nav2_costmap_2d`를 직접 띄우면 노드 이름이 `costmap`으로
하드코딩돼 있고 `__ns`를 제 이름으로 덮어써서, 리매핑 5개와 파라미터 섹션 중복이
필요하다. 목표를 아무도 안 보내므로 그쪽 MPPI는 한 번도 돌지 않는다.

### 주요 서비스

| 서비스 | 서버 | 호출자 |
|---|---|---|
| `/local_costmap/clear_entirely_local_costmap` | local_costmap | bt_navigator (복구 분기) |
| `/global_costmap/clear_entirely_global_costmap` | global_costmap | bt_navigator (복구 분기) |
| `/is_path_valid` | planner_server | BT |
| `/map_server/load_map`, `/map_server/map` | map_server | - |
| `/lifecycle_manager_nav/manage_nodes`, `/is_active` | lifecycle_manager_nav | - |
| `/<node>/change_state`, `/get_state` | 라이프사이클 5종 | lifecycle_manager_nav |

---

## 9. 라이프사이클

```
lifecycle_manager_nav (autostart: true)
  map_server -> planner_server -> controller_server -> behavior_server -> bt_navigator
```

순서대로 하나씩 `active`가 돼야 다음이 configure된다. 서로 `/bond`로 생존을
확인한다.

`mppi_controller`는 라이프사이클 밖이다. 평범한 rclpy 노드라 launch 즉시 뜨고
bond 감시도 안 받는다.

---

## 10. 주기

| 무엇 | Hz | 비고 |
|---|---|---|
| `/clock` | 1000 | `max_step_size` 0.001, RTF 1.0 |
| `/odom` | 50 | OdometryPublisher |
| `/scan` | 10 | gpu_lidar, 640 samples, 360도 |
| local_costmap | 10 | update = publish |
| global_costmap | 1 | |
| planner (BT 재계산) | 1 | |
| **MPPI solve** | **10** | `== 1/MPPIConfig.dt`, 노드가 assert로 검증 |
| **cmd_vel 발행** | **30** | solve 사이에는 마지막 명령을 유지 + 재회전 |
| behavior_server | 10 | |

`solve_frequency`가 `1/dt`와 어긋나면 `step()`이 호출마다 워밍스타트를 정확히
`dt`만큼 밀기 때문에 지평선이 타버린다. 그래서 생성자에서 assert한다.

---

## 11. 코드 위치

| 파일 | 줄 | 내용 |
|---|---|---|
| `launch/sim.launch.py` | 95 | Gazebo, URDF, 브리지, static TF, RViz |
| `launch/nav2.launch.py` | 110 | Nav2 5종, FollowPath 스왑, 라이프사이클 |
| `scripts/mppi_controller.py` | 518 | FollowPath 서버 + CostmapGrid + TF + 타이밍 로깅 |
| `config/nav2_params.yaml` | 307 | Nav2 전체 + `mppi_controller` 섹션 |
| `scripts/gen_world.py` | 242 | `scene.WALLS` -> SDF + PGM/yaml, 정합성 assert |
| `scripts/benchmark.py` | 365 | 시나리오 x 반복 실행, 10 Hz `/odom` 기록, CSV |
| `scripts/send_goal.py` | 150 | 텔레포트 + 목표 전송 |
| `scripts/stop.sh` | - | 스택 전부 정리 (팀 브리지는 건드리지 않는다) |
| `urdf/tidybot.urdf` | - | 베이스 + 마찰 0 캐스터 구 + gpu_lidar |

플래너 본체는 이 패키지에 없다. 별도 worktree의 `nav/` 패키지를 컨테이너에
마운트해서 쓴다.

```
/home/moon/Tidy/tidybot2/.claude/worktrees/nav-astar-mppi/nav/
  -> 컨테이너 /home/moon/tidybot2  (PYTHONPATH)
```

`mppi.py`(최적화기), `scene.py`(벽/시나리오 정의 — SDF와 PGM의 단일 출처),
`grid.py`, `metrics.py`, `astar.py`가 거기 있다. 복사본을 두면 어느 쪽이 진짜인지
모르게 되므로 원본을 rw로 붙였다.

`gen_world.py`가 두 세계의 정합성을 assert한다. `robot_radius`, 속도 한계, 지평선,
`model_dt >= 1/controller_frequency`, ObstaclesCritic inflation == costmap inflation,
`solve_frequency == 1/dt`, 목표 허용오차가 `scene.py` / `MPPIConfig` /
`nav2_params.yaml` 사이에서 어긋나면 월드 생성이 실패한다.

---

## 12. 관측 (ros2probe)

호스트에 `rp`를 설치해 커널에서 트래픽을 본다. compose가 `network_mode: host`라
컨테이너의 loopback 트래픽이 곧 호스트 loopback 트래픽이므로, 컨테이너에 eBPF 권한을
줄 필요가 없다.

순서가 중요하다. `rp run`이 노드보다 먼저 관측하고 있어야 discovery 트래픽을 잡는다.

```
터미널 1 (호스트)     rp run              <- 가장 먼저, 스스로 root로 올라간다
터미널 2 (컨테이너)   ros2 launch ...
터미널 3 (컨테이너)   send_goal.py
터미널 4 (호스트)     rp gui / rp topic hz /cmd_vel
```

RMW 가 CycloneDDS 로 바뀌면서 rp 의 **RTPS 경로**가 제 역할을 한다. zenoh 를 쓰던
동안 타임아웃하던 `rp discover` 는 이제 `--mode rtps` 로 쓰면 된다.

알려진 제약 세 가지.

- **GUI 그래프에는 graphviz가 필요하다.** `rp`는 배치를 `/usr/bin/dot`에 외주하는데
  (`gui/ros_graph.rs:357`, `stderr`는 `/dev/null`) 없으면 조용히 실패해 영원히
  "Computing layout..."을 띄운다. `sudo apt install graphviz`. README의 Requirements
  표에 빠져 있다.
- **`rp node info`는 액션을 안 보여준다.** `rp action info /follow_path`는 참여 노드를
  찾지만 서버와 클라이언트를 구분하지 못하고 양쪽에 똑같이 나열한다. 액션 배선은
  컨테이너에서 `ros2 node info`로 확인할 것.
- **`rp topic echo`는 호스트에서 안 된다.** rclpy가 필요한데 호스트에 ROS가 없다.
  `rp topic list/hz/bw/delay/info -v`는 커널 캡처만으로 되므로 영향 없다.
  `rp discover` 는 CycloneDDS 에서는 `--mode rtps` 를 쓴다.


---

## 13. 팀 연동 모드 (`controller:=team`)

탐색 시스템이 Nav2 planner 로 A* 경로를 받은 뒤 우리 노드의 `NavigateToGoal` 액션을
직접 부르는 구조. BT 가 없다.

```sh
ros2 launch tidybot_sim nav2.launch.py controller:=team scenario:=clutter
python3 scripts/mock_explorer.py --scenario clutter --rotate4
```

### 1~12 절과 무엇이 다른가

| | `controller:=mppi` | `controller:=team` |
|---|---|---|
| 진입 액션 | `/navigate_to_pose` (bt_navigator) | `/athome/navigate_to_goal` |
| 경로 공급 | bt_navigator 가 1 Hz 로 재계산 | 탐색 시스템이 `ComputePathToPose` 로 직접 |
| 실행 노드 | `mppi_controller` (FollowPath 서버) | `navigate_to_goal` + `executor` |
| 장애물 입력 | `/local_costmap/costmap` (odom, 6x6 rolling) | `/global_costmap/costmap` (map, 전역) |
| 프레임 변환 | path(map) -> costmap(odom) 필요 | 경로·costmap 모두 map, 변환 없음 |
| costmap 주기 | 0.5 Hz (planner 전용) | 10 Hz (컨트롤러의 유일한 입력) |
| 뜨는 노드 | 13 개 | **8 개** |
| 복구 | BT 의 spin/backup | 없음. 정지 후 reason 과 함께 abort |

빠지는 노드: `bt_navigator`, `behavior_server`, `controller_server`, 그리고
controller_server 가 소유하던 `local_costmap`. 라이프사이클 관리 대상은 `map_server`,
`planner_server` 둘뿐이다. 재시도와 재계획은 탐색 시스템의 일이다.

### 파일

| 파일 | 줄 | 역할 |
|---|---|---|
| `ws/src/athome_interfaces/` | — | **임시 정의.** 팀 원문 수령 시 통째로 교체 |
| `scripts/executor.py` | 524 | 상태머신·모니터·costmap·cmd 출력. **액션 타입을 모른다** |
| `scripts/navigate_to_goal.py` | 335 | 액션 서버. athome_interfaces 를 아는 유일한 파일 |
| `scripts/mock_explorer.py` | 300 | 탐색 시스템 대역 + 실패 경로 주입 |
| `config/demo.yaml` | 28 | 팀 공유 설정 사본 |

분리의 핵심은 `executor.py` 가 `athome_interfaces` 를 import 하지 않는다는 것이다.
outcome 과 phase 를 소문자 문자열로 내보내고, 정수 인코딩은 `.action` 을 아는
프론트엔드가 맡는다. 원문 교체 시 고칠 곳은 `navigate_to_goal.py` 상단의
`# ── msg 경계 ──` 블록 하나뿐이다.

### 상태 머신

```
IDLE --start()--> RUNNING --(도착 | 모니터 | cancel | fail)--> STOPPING --> IDLE
```

**모든** 종료 경로가 STOPPING 을 거친다. STOPPING 은 발행 명령을 `decel_limit` 로 0 까지
램프하고, `/odom` twist 가 `stop_speed` 아래로 `stop_hold` 초 유지되면 멈춘 것으로
본다. 팀 계약이 "로봇이 정지한 뒤 result 를 보낸다" 이므로 정지는 가정이 아니라
측정값이다. `stop_timeout` 을 넘기면 `stopped=false` 로 정직하게 보고한다.

**순서 계약**: 정지 판정 -> executor IDLE -> busy 해제 -> result 전송. `_done` 과
`IDLE` 은 같은 락 안에서 함께 바뀌고, 프론트엔드의 `busy` 해제는 `finally` 라 rclpy 가
result 를 보내기 전에 끝난다. 그래서 result 를 받은 클라이언트가 즉시 다음 goal 을
보내도 busy 로 거절되지 않는다 — 관측 회전 4 연발이 이 순서에 걸려 있다.

### 제자리 회전

탐색 시스템이 도착 후 90도 간격 4방향 관측을 **같은 액션**으로 요청한다
(path = 현재 위치 한 점, goal = 위치 동일·yaw 만 다름). 별도 회전 제어기는 없다.
`len(path) <= 1` 이면 회전으로 판정하고 `MPPI.set_weights()` 로 가중치 세트만 바꾼다.

```
주행   w_goal 15, w_head 0.5, w_dir 0.20   (MPPIConfig 기본값)
회전   w_goal 60, w_head 40,  w_dir 0      (rot_w_goal, rot_w_head)
```

새 비용 항은 없다. 같은 항의 다른 무게다. `w_goal` 이 위치를 붙들고 `w_head` 가
돌린다. `nav/tests/test_mppi.py::test_rotation_weights_turn_in_place` 가 고정한다.

주행 goal 도 yaw 를 갖는데 주행 가중치로는 yaw 허용오차에 못 든다. 그래서 xy 에
도착하면 회전 가중치로 전환해 제자리에서 heading 을 마무리한다 — 관측 회전과 같은
메커니즘이고, 두 번째 제어기가 아니다.

**한 점 경로인데 목표가 `xy_tolerance` 밖이면 거부한다** (status=2, reason=8). 회전으로
처리할 수 없는데 — 베이스를 움직여야 닿지만 어디로 갈지에 대한 정보가 경로에 없다 —
조용히 회전으로 읽으면 로봇은 제자리에서 돌다가 NO_PROGRESS 나 TIMEOUT 으로 끝나고
클라이언트는 이유를 알 수 없다.

### 모니터와 reason 코드

| reason | 이름 | 판정 |
|---|---|---|
| 2 | MAP_MISMATCH | `goal.map_version` != 로드한 지도 판본 |
| 4 | BLOCKED | 전 rollout 이 lethal 인 상태가 `blocked_time` 지속 |
| 5 | NO_PROGRESS | `no_progress_time` 동안 남은 거리가 `no_progress_distance` 만큼도 안 줆 |
| 6 | TIMEOUT | goal 의 `timeout_sec`, 0 이면 demo.yaml 의 drive/rotate 기본값 |
| 7 | LOCALIZATION_LOST | TF 조회 실패 또는 `tf_timeout` 보다 오래된 TF |
| 8 | INTERNAL | heartbeat 끊김, 회전 goal 검증 실패, 프레임 불일치 |

남은 거리는 직선이 아니라 **경로 위**로 잰다. 모퉁이를 도는 것도 진척으로 쳐야 하기
때문이다. `Executor._waiting_for_dynamic()` 은 보행자 예측이 들어올 자리다. 지금은
`False` 만 돌려준다 — 정당하게 대기 중인 로봇에 NO_PROGRESS 가 걸리면 안 되므로,
그때 이 함수만 채우면 된다.

heartbeat 는 `/athome/executor/heartbeat` (std_msgs/Header) 를 2 Hz 로 받고, **노드
clock 기준 수신 시각**으로 판정한다 (메시지의 `header.stamp` 가 아니다 — 감시 대상이
"송신 노드가 살아 있는가"이지 "지연이 얼마인가"가 아니다). 시계는 goal 수락 시점부터
돌고, 끊기면 detail 에 `heartbeat lost (age=2.0s)` 처럼 나이를 남긴다.
`heartbeat_timeout: 0` 은 감시를 끄며, 노드 시작 시와 goal 수락 시 WARN 을 찍는다.

### map_version — 스펙의 "거절"과 구현이 다르다

**스펙은 "map_version 불일치면 reason=2 로 거절"이라고 하지만, 구현은 수락 후 즉시
abort 다.** ROS 2 액션의 `GoalResponse.REJECT` 는 클라이언트에 **Result 를 전혀 보내지
않는다.** 거절됐다는 사실만 전달되고 reason 을 실을 자리가 없다. reason=2 가 도달하려면
goal 을 수락한 뒤 `abort()` 하는 수밖에 없다.

**팀과 합의 필요.** 클라이언트가 "rejected" 와 "accepted then failed" 를 구분해서
처리한다면 이 차이가 드러난다.

반면 **실행 중 두 번째 goal 은 진짜 REJECT** 한다. reason 코드 목록에 "busy" 가 없고,
수락했다 abort 하면 돌고 있는 goal 의 result 와 구분하기 어렵다.

판본 자체는 `gen_world.py` 가 `sha256(tidy.pgm + tidy.yaml)` 앞 8 자를
`maps/tidy.version` 에 쓰고 (현재 `f96a7082`), launch 가 파라미터로 넣어준다. 노드는
문자열 두 개를 비교할 뿐이라 계산 방식이 바뀌어도 코드는 그대로다.

### 검증 결과

4 개 시나리오 주행, 4 방향 회전, 실패 경로 5 종 전부 통과. status 3 종과 reason 4 종이
실제로 관측됐고, 모든 종료가 `stopped=true` 다.

| | 항목 | 결과 |
|---|---|---|
| A | corridor / doorway 주행 | SUCCEEDED · 21.9 / 20.3 s · err 0.12–0.14 m |
| B | 4 방향 회전 | 4/4 · 위치 드리프트 0.006–0.014 m · 각 2.8–3.1 s · REJECT 없음 |
| C-1 | 잘못된 map_version | FAILED/MAP_MISMATCH · 0.1 s |
| C-2 | 실행 중 두 번째 goal | REJECTED · 첫 goal 정상 완주 |
| C-3 | 주행 중 cancel | CANCELED · 요청 0.8 s 뒤 정지 |
| C-4 | heartbeat 중단 | FAILED/INTERNAL · 중단 2.4 s 뒤 정지 · detail 에 age |
| C-5 | 벽을 향한 경로 | FAILED/NO_PROGRESS · 0.638 m 앞에서 정지 |
| C-6 | 한 점 경로 + 먼 목표 | FAILED/INTERNAL · 0.1 s · detail 에 거리와 허용오차 |
| D | 4 개 시나리오 | corridor 21.7 · doorway 20.5 · utrap 16.0 · **clutter 32.3** s |

**clutter 가 통과한 이유는 짚어둘 필요가 있다.** 경로가 246 점 13.22 m 다 — 기존 mppi
모드에서 0/5 로 막히던 1.05 m 기둥 틈을 전역 planner 가 아예 우회했다. 팀 모드가 그
문제를 푼 것이 아니라, 입력이 global costmap 으로 바뀌면서 planner 가 통과 불가로
판단해 돌아간 것이다.

### 구현 중 실제로 터진 것

나중에 같은 곳을 건드릴 사람을 위해 남긴다.

- **rclpy 로거는 호출 지점별로 심각도를 캐싱한다.** 한 줄에서 `info` 와 `warn` 을
  번갈아 부르면 `ValueError: Logger severity cannot be changed between calls`. 그
  예외가 액션 콜백 밖으로 나가면 rclpy 가 **기본 Result(status=0)** 로 abort 해서,
  실패한 goal 이 금지값 0 으로 보고된다. `executor.py:_stop()` 이 두 줄로 나뉜 것은
  스타일이 아니라 이 때문이다.
- **`RewrittenYaml` 은 키 이름으로 파일 전체를 바꾼다.** `publish_frequency` 가
  costmap 과 `navigate_to_goal` 양쪽에 있어서, 통째로 넘기면 cmd_vel 발행이 30 -> 15 Hz
  로 끌려간다. 재작성본은 costmap 소유 서버에만 준다.
- **costmap 주기는 요청값대로 안 나온다.** 10/10 -> 5.0 Hz, 20/10 -> 6.67 Hz,
  20/15 -> 10.0 Hz. `ros2 topic hz` 로 재야 한다.
- **`install(PROGRAMS)` 는 실행 비트를 만들어주지 않는다.** 소스가 644 면 launch 가
  "executable not found" 로 죽는다.
- **`stop.sh` 의 프로세스 목록은 노드를 추가할 때마다 같이 고쳐야 한다.** 안 고치면
  이전 인스턴스가 살아남아 `/cmd_vel` 발행자가 2 개가 된다.

---

## 14. localization 모드

실제 로봇에서는 Gazebo ground truth 대신 FAST-LIO 가 위치를 준다. `controller:=team`
에서 그 인터페이스를 시뮬로 재현해, MPPI 가 실제와 같은 조건에서 돌게 한다.

```sh
ros2 launch tidybot_sim nav2.launch.py controller:=team localization:=fastlio_mock
```

`localization` 은 `gt`(기본) 또는 `fastlio_mock` 이고, **`fastlio_mock` 은
`controller:=team` 에서만 허용한다** — nav2/mppi 기준선은 ground truth 로 측정된
숫자를 갖고 있으므로 launch 가 assert 로 막는다.

### TF 트리

```
gt                                  fastlio_mock
map                                 map
 `-- odom        static 항등          `-- camera_init      static 항등 (map->camera_init 미구현)
      `-- base_link  Gazebo @50 Hz         `-- imu_link     fastlio_mock @10 Hz
           `-- lidar_link                       `-- base_link    static, 장착의 역
                                                     `-- lidar_link
```

`fastlio_mock` 에서는 **Gazebo 의 `/tf` 브리지를 끈다.** 켜 두면 `base_link` 에 부모가
둘 생긴다. `/odom` 토픽 자체는 mock 의 입력이므로 남긴다.

장착 변환을 `imu_link -> base_link` 방향으로 내는 이유도 같다. `imu_link` 는 이미
`camera_init` 의 자식이므로, `base_link -> imu_link` 로 내면 `base_link` 에 부모가
둘 생긴다.

검증: `ros2 run tf2_tools view_frames` 가 낸 그래프에
`map -> camera_init -> imu_link -> base_link -> lidar_link` 만 있고 부모가 둘인
프레임은 없다.

### mock 파라미터 (`nav2_params.yaml` 의 `fastlio_mock:`)

| 파라미터 | 기본값 | |
|---|---|---|
| `mount_xyz` | `[0.10, 0.0, 0.50]` | **임시값.** base_link 기준 imu_link 위치 |
| `mount_rpy` | `[0.0, 0.05, 0.0]` | **임시값.** 장착 자세 |
| `rate` | 10.0 | Livox Mid-360 스캔마다 |
| `drop_after` / `drop_duration` | 0 / 0 | 위치 소실 테스트. 0 이면 비활성 |
| `camera_init_frame` / `imu_frame` / `base_frame` | | 프레임 이름 |
| `odom_topic` / `odometry_topic` | `/odom` / `/Odometry` | 입력 / 출력 |

`drop_*` 중 하나를 런타임에 `ros2 param set` 으로 바꾸면 그 순간이 새 원점이 된다.
노드가 뜬 시각부터 재는 고정 시계로는 goal 이 언제 시작될지 알 수 없어서, 주행 도중에
끊으려면 그 방법밖에 없다. `mock_explorer.py --drop-localization AFTER DUR` 이 그것을
부른다.

### 출력

`/Odometry` (nav_msgs/Odometry): `frame_id=camera_init`, `child_frame_id=imu_link`,
**twist 는 전부 0**. 실제 FAST-LIO 가 twist 를 채우지 않으므로 여기서도 채우지
않는다 — 채우면 시뮬에서만 동작하는 정지 판정을 만들게 된다.

그래서 executor 의 속도는 **TF pose 차분**으로만 구한다. `vel_window`(0.3 s) 양 끝
두 표본의 차이이고, 창 안 표본이 2 개 미만이거나 최신 표본이 창보다 오래되면 속도는
**None(모름)** 이다. 0 이 아니다. 모르면 정지 판정을 통과시키지 않는다 — 위치를 잃은
채 관성으로 굴러가는 로봇을 "정지했다"로 보고하는 것이 이 계약에서 가장 나쁜
거짓말이다. 각속도는 `stop_yaw_rate` 로 따로 보아, 제자리 회전 중을 정지로 오판하지
않는다.

### 시뮬과 실제의 차이

여기 있는 것은 **인터페이스의 모양**이지 센서의 거동이 아니다.

| | 이 mock | 실제 FAST-LIO |
|---|---|---|
| 노이즈 | 없음. ground truth 를 그대로 옮긴다 | 있음 |
| 드리프트 | 없음 | 누적된다. 루프 클로저 없이는 되돌아오지 않는다 |
| `camera_init` 자세 | Gazebo world 와 일치시킨다 (중력 정렬 가정) | **켤 때의 IMU 자세.** 로봇이 기울어진 채 켜지면 camera_init 도 기울어진다 |
| `map -> camera_init` | 항등 static | 미구현. 지도를 FAST-LIO PCD 에서 만들므로 당분간 항등이지만, 정렬이 필요해지면 이 자리 |
| 장착 수치 | 임시값 | 실측 필요 |
| 발행 끊김 | `drop_*` 로 의도적으로만 | 실제로 끊기고, 다시 붙을 때 점프할 수 있다 |

그래서 이 모드가 검증하는 것은 "**TF 체인과 10 Hz 입력에서 제어가 성립하는가**"까지다.
노이즈와 드리프트에 대한 강건성은 실제 데이터로 따로 봐야 한다.

### 검증 결과

`map->base_link` 가 Gazebo ground truth 와 일치: 위치 오차 **1.1e-16 m**, yaw
**8.9e-16 rad** (기준 1e-3). `/Odometry` 10.002 Hz, twist 전부 0.

| 항목 | gt | fastlio_mock | |
|---|---|---|---|
| A: corridor 주행 | 22.1 s / err 0.122 m, 0.016 rad | 22.2 s / err 0.122 m, 0.014 rad | 동등 |
| B: utrap 주행 | 16.9 s / err 0.133 m | 17.1 s / err 0.120 m | 동등 |
| B: 4 방향 회전 | 4/4, 드리프트 0.005–0.035 m | 4/4, 드리프트 0.011–0.033 m | 동등 |
| jerk (corridor) | 1.233 | 1.236 | **+0.2 %** |
| jerk (utrap) | 1.450 | 1.495 | +3.1 % |
| 속도 미상 비율 | 0.0 % | 0.5–0.6 % | 첫 두 주기뿐 |

jerk 는 지령 속도의 2 계 차분 크기의 RMS 로, 두 모드에서 같은 정의로 쟀다
(`results/<mode>_<시각>.csv`). 10 Hz pose 입력이 제어 품질을 거의 떨어뜨리지 않는다.

`mount_rpy` 의 pitch 를 0.1 rad 으로 올려도 A 와 B 결과가 같다 (주행 22.2 s,
err 0.125 m / 0.013 rad). 장착 기울기가 `map->base_link` 의 yaw 에 새지 않는다는 뜻이고,
`fastlio_mock.py` 의 왕복 검산에서도 500 회 무작위 장착에 대해 yaw 오차 4e-16 rad 이다.

**위치 소실**: 주행 중 1 s 발행 중단 -> TF 나이가 0.52 s 가 된 시점(중단 0.4 s 후)에
감지, `FAILED/LOCALIZATION_LOST`(reason=7), `stopped=true`, 중단 시각부터 result 까지
**3.0 s**. 그 중 1.0 s 는 암전 자체이고, 나머지는 감속과 정지 확인이다. 암전이
`stop_timeout`(3 s)보다 길면 `stopped=false` 로 보고된다 — 모르는 것을 참이라고 하지
않는다.

---

## 15. RMW 와 네트워크

솔로 시뮬은 **CycloneDDS + `ROS_LOCALHOST_ONLY=1` + 도메인 42** 로 PC 안에 갇혀
있다. 아래의 격리 측정은 그 모드에 대한 것이고 지금도 유효하다.

**팀 연결은 zenoh-bridge-ros2dds 방식을 접었다.** 양쪽을 `rmw_zenoh_cpp` 로 맞추고
`rmw_zenohd` 라우터 하나를 `ZENOH_CONFIG_OVERRIDE` 로 연결하는 형태로 바꿨고,
2026-10-26 실기 주행이 그걸로 돌았다. 진입점은 `scripts/env_team.sh` 다
(`RMW_IMPLEMENTATION=rmw_zenoh_cpp`, `ROS_DOMAIN_ID=25`, `ROS_LOCALHOST_ONLY` 해제).
`net_team.sh` 와 `bridge_team_sim.json5` 는 삭제했다.

> **TODO — 팀 링크의 안전 서술이 비어 있다.** 브리지 allow-list 와
> `ROS_LOCALHOST_ONLY=1` 이 담당했던 "시뮬 `/cmd_vel` 이 실제 로봇에 닿지 않는다"는
> 보장이 지금은 다른 두 장치로 옮겨갔다 — `cmd_vel_topic` 기본값
> `/cmd_vel_dryrun`(무장은 명시적 `-p cmd_vel_topic:=/cmd_vel`)과, 실기 모드에서
> `controller_server`·`bt_navigator` 를 띄우지 않는 것. 이 둘이 옛 보장과 같은 수준인지
> **측정으로 확인한 뒤** 이 절을 다시 쓴다. 그때까지 이 문단이 유일한 기록이다.

### 두 가지 실행 모드

| | 솔로 (기본) | 팀 연결 |
|---|---|---|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | 같음 |
| `ROS_DOMAIN_ID` | **42** | **25** (팀 도메인) |
| `ROS_LOCALHOST_ONLY` | **1** | **1 (그대로 유지)** |
| `IGN_IP` | `127.0.0.1` | 같음 |
| PC 간 전송 | 없음 | `rmw_zenohd` 라우터 1 개 (TCP 7447) |
| PC 밖으로 나가는 것 | 없음 | 브리지의 zenoh(TCP 7447)뿐 |

```sh
# 솔로 -- compose.yaml 의 환경변수가 그대로 적용된다. 할 일 없음.
ros2 launch tidybot_sim nav2.launch.py controller:=team localization:=fastlio_mock

# 팀 연결 -- env_team.sh 를 source 하고 라우터를 한 곳에서만 띄운다.
. scripts/env_team.sh
ros2 run rmw_zenoh_cpp rmw_zenohd        # 라우터. **정확히 한 PC 에서만** (포트 7447)
```

라우터가 두 곳에서 돌면 7447 이 충돌한다. 다른 PC 는 라우터를 띄우지 않고
`ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<라우터IP>:7447"]'` 로 붙는다.

솔로 도메인을 42 로 둔 것은 그대로다. 격리가 실수로 빠져도 로봇과 다른 그래프에
있게 하는 장벽이다.

### 격리가 실제로 되는지 — 측정

```
$ ip maddr show | grep 239.255.0.1          # DDS 디스커버리 멀티캐스트
(없음)
$ grep "selected interface" <launch log>
selected interface "lo" is not multicast-capable: disabling multicast
$ ss -lnup | grep 192.168.0.13               # LAN 에 묶인 DDS 소켓
(없음)
```

CycloneDDS 가 인터페이스로 `lo` 를 고르고 멀티캐스트를 끈다. 어떤 인터페이스도 DDS
디스커버리 그룹에 가입하지 않으므로, 밖으로 나갈 경로도 들어올 경로도 없다.

소켓 자체는 `0.0.0.0` 에 바인딩된다(Cyclone 의 기본 동작이다). 막는 것은 바인딩이
아니라 디스커버리다 — LAN 피어를 영영 발견하지 못하므로 아무것도 보내지 않는다.

**한계를 정직하게 적는다: 기계가 한 대라 "다른 PC 에서 안 보인다"를 직접 증명할 수는
없다.** 증명한 것은 탈출 경로가 디스커버리 수준에서 막혔다는 것까지다. `network_mode:
host` 라 같은 PC 안의 다른 셸에서는 당연히 보인다 — 그것은 격리 실패가 아니다.

### Gazebo 는 DDS 가 아니다 (이번에 발견)

`ROS_LOCALHOST_ONLY` 는 ROS 의 DDS 만 막는다. **Gazebo 는 ign-transport 로 따로
디스커버리를 하고, 그것은 막히지 않는다.** 측정:

| | `IGN_IP` 없음 | `IGN_IP=127.0.0.1` |
|---|---|---|
| 239.255.0.7 가입 인터페이스 | **wlo1**, docker0, br-* (3곳) | `lo` (1곳) |

스택을 띄우면 생기고 끄면 사라지는 것으로 우리 것임을 확인했다. 같은 망의 다른
Gazebo 와 ign 토픽이 섞일 수 있었다. `IGN_IP=127.0.0.1` 로 닫았다.

### 참가자 수 한계 (이번에 발견)

CycloneDDS 의 기본 `MaxAutoParticipantIndex` 는 9 다. 호스트와 도메인당 참가자가
10 개로 묶이고, 넘으면 `rmw_create_node: failed to create domain` 으로 노드가 죽는다.

`controller:=team` 은 노드가 9 개라 통과했지만 `controller:=nav2`(13)와
`controller:=mppi`(14)는 그대로 실패했다. `config/cyclonedds.xml` 에서 60 으로 올렸고,
`CYCLONEDDS_URI` 로 주입한다. 격리와는 무관한 설정이다.

**팀 로봇도 노드가 많으면 같은 벽을 만난다.** 공유할 가치가 있는 정보다.

### 전환 전후 비교 (`controller:=team localization:=fastlio_mock`)

| 토픽 | rmw_zenoh | CycloneDDS |
|---|---|---|
| `/clock` | 999.9 | 1000.0 |
| `/odom` | 50.0 | 50.0 |
| `/Odometry` | 10.0 | 10.0 |
| `/tf` | 10.0 | 10.0 |
| `/scan` | 10.0 | 10.0 |
| `/global_costmap/costmap` | 10.0 | 10.0 |
| `/cmd_vel` | 30.0 | 30.0 |

| 주행 | | 소요 | jerk | MPPI p50 | p99 | costmap p50 |
|---|---|---|---|---|---|---|
| corridor | rmw_zenoh | 21.2 s | 1.236 | 6.27 ms | 11.01 | 0.13 ms |
| corridor | CycloneDDS | 21.2 s | 1.226 | **8.34 ms** | 16.17 | **0.18 ms** |
| utrap | rmw_zenoh | 16.1 s | 1.495 | 5.79 ms | 9.61 | 0.12 ms |
| utrap | CycloneDDS | 16.1 s | 1.503 | **8.20 ms** | 12.05 | **0.17 ms** |

검증 A(주행)와 B(4 방향 회전)는 그대로 통과한다. 소요 시간과 jerk 는 사실상 동일하다.

**solve 시간이 33~42 % 늘었다.** 다만 `mppi_ms` 와 `cm_ms` 는 둘 다 순수 numpy 연산
이고 **같은 비율로 함께 늘었다** — 플래너 안에서 뭔가 느려진 것이 아니라 CPU 여유가
줄어든 것이다. 1000 Hz `/clock` 을 9~14 개 노드에 뿌리는 것이 지배적인 트래픽이고,
DDS 는 메시지당 비용이 zenoh 보다 크다. 예산은 10 Hz solve 의 100 ms 이고 p99 가
16 ms 이므로 여유는 충분하다.

> **기존 벤치마크 수치(Nav2 MPPI 대 자체 MPPI: jerk 0.42 대 0.92, 도달 19.3 s 대
> 18.7 s 등)는 전부 `rmw_zenoh_cpp` 에서 측정된 것이다.** CycloneDDS 에서 다시 재지
> 않았다. 위 표가 보여주듯 제어 품질은 바뀌지 않지만, 숫자를 인용할 때는 측정 조건을
> 함께 밝힐 것.

---

## 16. 실기 연동 (동료 PC)

실측 출처: **2026-10-02 ~ 10-03, 동료 PC 로부터 실제 수신**. 추정값은 없다.

절차는 `scripts/check_team_dryrun.md` 에 따로 있다.

### 네 가지 실행 모드

| | RMW | 도메인 | 라우터 원격 연결 | cmd_vel | 쓰는 파일 |
|---|---|---|---|---|---|
| 솔로 시뮬 | rmw_zenoh_cpp | 42 | 없음 | `/cmd_vel` → Gazebo | `nav2_params*.yaml` |
| 팀 연결 (관찰) | rmw_zenoh_cpp | 25 | 동료 라우터 | — | — |
| **실기 dry-run** | rmw_zenoh_cpp | 25 | 동료 라우터 | **`/cmd_vel_dryrun`** | `team_real.yaml` |
| **실기 armed** | rmw_zenoh_cpp | 25 | 동료 라우터 | **`/cmd_vel`** → 실제 로봇 | `team_real.yaml` + 덮어쓰기 |

솔로 격리는 **"우리가 먼저 걸지 않는다 + 아무도 자동으로 못 찾는다 + 도메인 42"** 다.
rmw_zenoh 라우터 기본값이 `scouting.multicast.enabled: false`, `autoconnect.router: []`,
`connect.endpoints: []` 이기 때문이다. `ROS_LOCALHOST_ONLY` 는 **zenoh 에서 효과가 없다**.
약점은 인바운드다: 라우터가 `tcp/[::]:7447` 로 모든 인터페이스에서 듣고 있어, 상대가
우리 IP 를 알고 걸어오면 연결된다. 도메인이 다르면 그래프는 안 섞인다.

```sh
ss -tn | grep ':7447'      # 원격 IP 와 ESTAB 이 없어야 솔로
```

### 실기 TF

```
map → camera_init → body → base_link      (부모 중복 없음, 실측 확인)
```

`base_link` z ≈ −0.36, roll/pitch < 0.01 rad.

**지연이 크다.** TF `map→base_link` stamp 나이:

| 최소 | p50 | p90 | p99 | 최대 |
|---|---|---|---|---|
| 0.114 s | 0.279 s | 0.580 s | 1.364 s | 1.652 s |

최소값 0.114 s 가 시계차 + 최소지연이고, 그 위 1.5 s 가 **실제 무선 지터**다
(시계 어긋남이 아니다). 기본 `tf_timeout` 0.5 s 로는 약 10 % 주기에서
LOCALIZATION_LOST 가 난다.

### 실기 토픽

| 토픽 | 크기·형식 | 주기 (최대 간격) | QoS |
|---|---|---|---|
| `/mppi_static_costmap/costmap` | 747×1316 @0.05, origin (−8.109, −42.229), frame map | 0.65 Hz (2.59 s) | RELIABLE / KEEP_LAST(1) / **TRANSIENT_LOCAL** |
| `/local_costmap/costmap` | 160×160 @0.05 = 8×8 m rolling, frame map | 3.27 Hz (1.18 s) | 같음 |
| `/map` | 747×1316, 구조물 정적 지도 | 1 회 | 같음 |
| `/Odometry` | camera_init → body, **twist 전부 0** | 10.5 Hz | RELIABLE / KEEP_LAST(20) / VOLATILE |

**우리 구독 QoS 가 정확히 일치한다** — 안 맞으면 조용히 한 장도 안 온다.

없는 것: **`/compute_path_to_pose` 등 액션 0 개**(planner 서버 없음), `/goal_pose`,
`/scan`, `/clock`. `/cmd_vel` 은 구동 노드가 떠 있을 때만 존재한다.

### 속도 추정 — velocity_frame 과 vel_window

`map→base_link` 에는 ICP 정합 보정이 섞인다. 로봇이 **멈춰 있어도** ~1 Hz 로 2~3 cm
계단 점프가 생기고, 그걸 차분하면 정지인데도 속도가 나온다. 결과는 모든 goal 이
`stopped=false` 로 끝나는 것이었다.

| 설정 | 측정 속도 p50 | 결과 |
|---|---|---|
| `map→base_link`, 창 0.3 s | **0.0560 m/s** | stopped=false, 13.3 s |
| `camera_init→base_link`, 창 0.3 s | 0.0200 m/s | stopped=false (임계 0.02 에 걸침) |
| **`camera_init→base_link`, 창 1.0 s** | **0.0062 m/s** | **stopped=true, 10.5 s** |

프레임 교체가 2.8 배, 창 확대가 3.2 배 — 합쳐 **9 배** 줄었다. `camera_init→base_link`
에는 ICP 점프가 없다(팀 실측). `vel_window` 는 `_velocity()` 한 곳에서만 쓰이고 그
결과는 정지 판정과 진단 CSV 에만 쓰인다 — `MPPI.step()` 은 속도를 받지 않으므로
제어 경로에는 닿지 않는다.

### 무선 경유 임계값은 테스트 전용이다

`team_real.yaml` 의 `tf_timeout 2.0 / local_stale 2.0 / static_stale 30.0` 은 위
실측을 덮으려고 올린 값이다. **올렸다는 것은 0.3~1.6 s 묵은 pose 로 제어한다는
뜻이다.** 0.5 m/s 에서 1.6 s 는 0.8 m 틀린 위치다. 그래서 `mppi_overrides` 로 속도를
(0.2, 0.2, 0.6) 으로 낮춰 둔다.

**실주행은 로봇 PC 에서 돌려야 한다.** 같은 PC 면 지연이 ~1 ms 로 떨어져 이 값들을
원래대로(0.5 / 1.0 / 3.0) 되돌릴 수 있다.

### 자유 공간이 17 개로 나뉘어 있다 (분석)

`/map` 의 자유 셀에 거리 변환을 걸고, 여유 ≥ t 인 셀을 8-연결로 묶어 t 를 낮춰 가며
언제 합쳐지는지 보았다.

| t (= 필요 반경) | 통로 폭 2t | 성분 수 | 최대 성분 | 로봇 성분 |
|---|---|---|---|---|
| **0.37** (현재) | 0.74 m | **17** | 824,404 | **5,788** |
| 0.30 | 0.60 m | 41 | 845,523 | 7,373 |
| 0.26 | 0.52 m | — | 863,669 | 8,152 |
| **0.25** | **0.50 m** | 52 | 884,093 | **884,093 (합쳐짐)** |

**지도 잡음이 아니다.** 17 개 중 5 셀 이하(잡음)는 **4 개**뿐이고, 100 셀(0.25 m²)
이상인 실제 구역이 6 개다. 로봇이 있는 구역은 8,152 셀 = 20.4 m² 이고, 전체
863,669 셀 = 2,159 m² 와 **단 하나의 통로**로 이어진다:

```
통로 위치 (−6.78, +6.31),  폭 0.51 m,  로봇에서 2.91 m
```

`robot_radius 0.37` 이면 로봇은 0.74 m 가 필요하다. **0.51 m 통로는 통과할 수 없다.**
실제 좁은 통로(문·가구 사이)이지 지도 잡음이 아니므로, **탐색 시스템도 같은 costmap 으로
A\* 를 쓰는 한 똑같이 막힌다.** → 팀 확인 대기.

### 로봇이 지도상 벽 안에 있는 경우

planner 가 `status=6`(ABORTED), 경로 0 점을 낸다. 실제로 겪었다: 로봇 셀의 static
cost 가 99 였고, **우리 costmap 과 동료 costmap 이 같은 지점에서 같은 값**이었다
(설정 차이가 아니라 실제 상황). 로봇을 옮기자 즉시 해결됐다.

원인은 둘 중 하나다:

- **초기 위치(2D Pose Estimate) 부정확** — ICP 정합이 로봇을 벽 안에 놓았다.
  `/initialpose` 로 다시 잡으면 된다. 같은 지점을 local costmap 은 0(자유)으로 보는
  것이 이쪽을 가리키는 신호다.
- 로봇이 실제로 벽에서 0.37 m 안에 있다.

---

## 17. 기술 부채 · 팀 확인 대기

### 팀 확인 대기 (실기)

| 항목 | 내용 |
|---|---|
| **0.51 m 통로** | 로봇이 있는 구역(20.4 m²)이 전체(2,159 m²)와 폭 0.51 m 통로 하나로만 이어진다. `robot_radius 0.37` 로는 통과 불가(0.74 m 필요). 지도 잡음이 아니라 실제 좁은 통로다. **탐색 시스템도 같은 costmap 으로 A\* 를 쓰므로 동일하게 막힌다.** 반경을 실측 footprint 로 낮출지, 그 통로를 치울지 결정 필요 |
| **`/cmd_vel` 규격 미확인** | 속도의 기준 프레임, 단위, 타임아웃, 게임패드와의 우선순위를 모른다. armed 전에 확인 필요 |
| **planner 서버 부재** | 동료 쪽에 액션이 0 개다. 지금은 우리가 `real_planner.launch.py` 로 띄워 쓴다. 실제 구성에서 누가 경로를 만드는지 미정 |
| **`map_version` 형식** | 실기는 파일별 SHA256 전체값이고, NavigateToGoal 에 싣는 문자열 형식이 미합의. 지금은 빈 값이라 검사를 건너뛴다 |
| **초기화 완료 신호** | 정합이 끝났다는 것을 어떻게 아는지 미정. `require_initialpose` 는 `/initialpose` 수신만 본다 |
| **지도에 없는 장애물 재계획 공백** | static costmap 은 PCD 기반 구조물 지도라 실시간 장애물이 없다. 경로 위에 그런 장애물이 있으면 MPPI 가 회피하다 NO_PROGRESS 로 끝나고, **탐색 시스템이 재시도해도 planner 는 같은 경로를 낸다.** 시뮬 한계가 아니라 팀 구조의 공백 |
| **reason 코드 부재** | costmap 상실에 해당하는 코드가 없어 INTERNAL(8) 로 보낸다. 전용 코드 추가를 제안할 만하다 |

### 미실시 · 재작성 대기

| 항목 | 내용 |
|---|---|
| **팀 링크 안전 서술 미작성** | 브리지 allow-list 가 담당했던 '시뮬 `/cmd_vel` 이 로봇에 닿지 않는다' 보장이 `cmd_vel_topic` 기본값과 실기 모드의 노드 구성으로 옮겨갔다. 같은 수준인지 측정으로 확인하고 15 절을 다시 써야 한다 |
| **솔로 시뮬 회귀 미실시** | `velocity_frame` · `vel_window` 변경 후 A(주행) · B(4방향 회전) 를 돌리지 않았다. 실기 연결 중에는 시뮬을 띄울 수 없어 미뤘다. **로봇 PC 배포 전 필수** |
| **carrot 임계 미결** | 경로점 건너뛰기가 `cost >= 99` 기준인데, 측정상 cost 0.959 인 점이 채택되어 상자 회피가 실패한다. `carrot_max_cost` 추가 제안이 미승인 상태 |
| **Dockerfile 재빌드 미실시** | 두 RMW 를 모두 설치하도록 고쳤지만 **아직 재빌드하지 않았다**. 돌아가는 컨테이너는 zenoh 가 손으로 설치된 상태이고 그대로 동작한다. 다음 재빌드 때 이미지와 일치하게 된다 |

| 항목 | 내용 |
|---|---|
| **`athome_interfaces` 가 임시 정의** | 필드 이름은 스펙과 일치하고 타입은 추정이다. 팀 원문 수령 시 패키지를 통째로 교체하고 `navigate_to_goal.py` 의 msg 경계 블록만 확인하면 된다 |
| **map_version "거절" 의 구현 차이** | 스펙의 "reason=2 로 거절" 을 수락 후 즉시 abort 로 구현했다. ROS 2 액션 REJECT 는 result 를 전달할 수 없기 때문이다. **팀과 합의 필요** |
| **team 모드에 costmap 을 지우는 주체가 없다** | BT 를 들어내면서 `clear_entirely_global_costmap` 을 부르던 복구 분기도 사라졌다. 마크가 누적되면 planner 가 빈 경로로 ABORT 한다 (측정됨: 클리어 전 ABORTED, 클리어 후 SUCCEEDED). `mock_explorer.py` 는 텔레포트 직후 지우지만 그것은 검증 도구 쪽 처방일 뿐이다. 실제 운용에서 탐색 시스템이 맡을지, 우리 노드가 BLOCKED/NO_PROGRESS 때 한 번 지우고 재시도할지 **팀과 정해야 한다** |
| **벤치마크 수치의 측정 조건** | Nav2 대 자체 MPPI 비교 수치는 `rmw_zenoh_cpp` 에서 측정됐다. CycloneDDS 재측정은 하지 않았다 (15 절) |
| **장착 수치가 임시값** | `mount_xyz` / `mount_rpy` 는 추정이다. 실측을 받으면 `nav2_params.yaml` 의 `fastlio_mock:` 섹션만 고치면 된다. `demo.yaml` 의 `robot_radius` 와 같은 이유로 `gen_world.py` 의 정합성 assert 에는 일부러 넣지 않았다 |
| **`camera_init` 중력 정렬 가정** | mock 은 `camera_init` 을 Gazebo world 와 일치시킨다. 실제 FAST-LIO 의 `camera_init` 은 켤 때의 IMU 자세라 기울어질 수 있고, 그러면 `map -> camera_init` 을 항등으로 둘 수 없다. 지도를 PCD 에서 만드는 동안은 문제가 없지만, 정렬이 필요해지는 시점이 온다 |
| **회전의 yaw 여유가 거의 없다** | 관측 회전의 yaw 오차가 0.182–0.250 rad 로 허용오차 0.25 에 붙어 있다. 도착 판정이 허용오차를 넘는 순간 멈추기 때문이다. 팀이 더 엄격한 tolerance 를 쓰면 실패한다 |
| `CostmapGrid` 중복 | `executor.py` 와 `mppi_controller.py` 에 각각 있다. 미지(-1) 처리가 파라미터냐 하드코딩이냐가 실제 차이지만, FollowPath 프론트엔드를 executor 로 합칠 때 하나로 만들어야 한다. `# ponytail:` 주석으로 표시됨 |
| costmap 크롭 미구현 | 스펙은 "로봇 주변 창만 잘라서 lookup" 을 요구한다. 현재 전체 맵을 쓴다. 근거: 비용의 대부분이 `msg.data` -> numpy 변환인데 크롭해도 안 줄고, 그 변환은 solve 마다가 아니라 costmap 콜백에서 한 번만 한다. **팀 실제 맵 크기에서 `cm_ms` 를 재고 판단할 것** |
| `robot_radius` 합의 | demo.yaml 은 0.35, 팀 임시값은 0.30. 0.35 인 이유는 nav2_params 의 양쪽 costmap 이 0.35 로 inflation 을 굽고 있고 여기만 내리면 기준선 벤치마크가 무효가 되기 때문. 실제 footprint 기준으로 통합 전 합의 필요 |
| global_costmap 주기 | 20/15 는 이 머신에서 10 Hz 가 나오는 조합이다. 팀 실제 맵 크기에서 다시 재야 한다 |
| `mppi_controller.py` 미통합 | FollowPath 경로는 기준선 벤치마크 보존을 위해 동결. team 모드 검증이 끝났으므로 통합 여부를 판단할 수 있다 |
| `nav/MPPI.md`, `nav/NOTES.md` | 정리 작업 이후 코드와 어긋난 상태. 삭제된 `adapt_lam` / `sample_previous_plan` 을 설명하고 `lam` 을 온도라고 부른다 |
