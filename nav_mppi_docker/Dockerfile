# TidyBot++ 홀로노믹 베이스용 MPPI 로컬 플래너. 두 가지로 쓴다:
#   시뮬   Gazebo + Nav2 (controller:=nav2 | mppi | team)
#   실기   동료 PC 의 FAST-LIO 측위·costmap 에 붙어 /cmd_vel 만 낸다
#
# 컨테이너 안의 사용자 이름은 호스트와 무관하게 항상 moon 이다 -- 경로
# (/home/moon/ws)가 compose.yaml 의 CYCLONEDDS_URI·PYTHONPATH 와 묶여 있다.
# 바꾸려면 세 곳을 같이 고쳐야 한다. UID/GID 만 호스트와 맞춘다.
FROM osrf/ros:humble-desktop-full

ARG USERNAME=moon
ARG UID=1000
ARG GID=1000

# 파이썬 의존성을 **apt 로** 명시한다. nav/ 의 폐쇄집합(mppi, astar, grid, scene,
# metrics)이 쓰는 서드파티는 numpy 와 scipy 둘뿐이고, 나머지는 표준 라이브러리다.
#
# 지금까지는 둘이 ros-humble-desktop-full 의 전이 의존성으로 우연히 들어왔고
# `pip3 install numpy scipy` 한 줄이 있었지만 무효였다 -- apt 가 먼저 깔아서 pip 이
# '이미 충족'으로 넘긴다 (실측: 둘 다 apt 버전 1.21.5 / 1.8.0 그대로였다).
# pip 로 받으면 PyPI 의 더 새 numpy 가 /usr/local 에 들어가 ROS 의 C 확장(cv_bridge
# 등)과 ABI 가 어긋날 수 있으므로, ROS 가 쓰는 apt 버전에 맞춘다.
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-rmw-cyclonedds-cpp ros-humble-rmw-zenoh-cpp \
    ros-humble-ros-gz \
    ros-humble-teleop-twist-keyboard \
    python3-numpy python3-scipy python3-yaml \
    python3-pip python3-colcon-common-extensions \
    git vim \
 && rm -rf /var/lib/apt/lists/*

# 마운트 폴더 권한 문제 방지: 호스트와 같은 UID
RUN groupadd -g ${GID} ${USERNAME} && \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 두 RMW 를 **모두** 설치한다. 둘 다 쓰기 때문이다:
#
#   솔로 시뮬 : rmw_cyclonedds_cpp + ROS_LOCALHOST_ONLY=1 + 도메인 42 (compose.yaml).
#               PC 안에 갇혀 있고, 벤치마크 수치가 이 설정에서 측정됐다. 그래서 기본값.
#   팀 연결   : rmw_zenoh_cpp + rmw_zenohd 라우터. scripts/env_team.sh 가 전환한다.
#               2026-10-26 실기 주행이 이 경로로 돌았다.
#
# 하나만 설치하면 안 된다. 지금까지 이미지에는 cyclonedds 만 있고 zenoh 는 돌아가는
# 컨테이너에 손으로 설치된 상태였고, 재빌드하면 실기 링크가 끊어지는 함정이었다.
# (옛 계획은 PC 사이를 zenoh-bridge-ros2dds 로 잇는 것이었고 그때는 둘이 호환되지
#  않아 하나를 골라야 했다. 그 방식은 접었다 -- ARCHITECTURE.md 15 절.)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ws
