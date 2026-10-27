# athome_interfaces (임시)

**임시 정의 — 팀 원문 수령 시 교체.**

팀 저장소의 `ros2/athome_interfaces` 를 받기 전까지 시뮬레이션 작업을 진행하기 위한
대역이다. PDF 스펙의 **필드 이름은 정확히 일치**하고, **타입은 추정**이다.

## 교체 방법

1. 이 디렉터리를 통째로 팀 원문으로 바꾼다.
2. `colcon build --packages-select athome_interfaces tidybot_sim`
3. `scripts/navigate_to_goal.py` 의 변환 함수 두 개(`goal_from_msg`,
   `result_to_msg`)만 확인한다. msg 와 내부 자료구조 사이의 변환은 그 두 함수
   안에만 있으므로, 타입이 달라져도 고칠 곳은 거기뿐이다.

## 알려진 불확실성

| 필드 | 추정 | 다를 수 있는 것 |
|---|---|---|
| `request_id` | `string` | `uint32` 일 수도 |
| `phase` | `string` | `uint8` enum 일 수도 |
| `xy_tolerance` 등 | `float64` | `float32` 일 수도 |
| `status`, `reason` | `uint8` 상수 | 값 자체는 스펙에 명시돼 있어 확실 |

## 설계 메모

`status` 에 0 이 없는 것은 스펙의 요구다. uint8 의 기본값 0 이 "성공"으로 읽히면,
result 를 채우지 못한 경로가 조용히 성공으로 보고된다.
