# ------------------ Humble controller package ----------------------
FROM osrf/ros:humble-desktop AS pmr_tp_final_humble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PMR_WS=/root/ros2_ws

# Base ROS/Python tools used to build and run this package.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-pip \
    python3-rosdep \
    ros-humble-crazyflie-interfaces \
 && rm -rf /var/lib/apt/lists/*

# Create workspace and copy dependency manifests first for Docker layer cache.
RUN mkdir -p "${PMR_WS}/src/pmr_tp_final"
WORKDIR ${PMR_WS}

COPY package.xml ${PMR_WS}/src/pmr_tp_final/

# Install rosdep-resolvable dependencies. Gurobi is optional until the QP path
# is active, so skip it for this lightweight controller image.
RUN rosdep init || true \
 && rosdep update \
 && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && apt-get update \
 && rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}" \
    --skip-keys python3-gurobipy \
 && rm -rf /var/lib/apt/lists/*

# Copy and build the package.
COPY . ${PMR_WS}/src/pmr_tp_final
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build --symlink-install --packages-select pmr_tp_final

# Automate workspace sourcing for interactive bash sessions (docker exec).
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "if [ -f ${PMR_WS}/install/setup.bash ]; then source ${PMR_WS}/install/setup.bash; fi" >> /root/.bashrc

WORKDIR ${PMR_WS}
CMD ["bash"]
