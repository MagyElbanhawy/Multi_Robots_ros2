FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
WORKDIR /workspace/Multi_Robots_ros2

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-launch-testing \
    && rm -rf /var/lib/apt/lists/*

COPY . /workspace/Multi_Robots_ros2
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch emrmf_core emrmf_sim.launch.py"]
