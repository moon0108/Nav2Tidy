FROM osrf/ros:humble-desktop-full

ARG USERNAME=moon
ARG UID=1000
ARG GID=1000

RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-rmw-zenoh-cpp \
    ros-humble-ros-gz \
    ros-humble-teleop-twist-keyboard \
    python3-pip python3-colcon-common-extensions \
    git vim \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy scipy

# 마운트 폴더 권한 문제 방지: 호스트와 같은 UID
RUN groupadd -g ${GID} ${USERNAME} && \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ws
