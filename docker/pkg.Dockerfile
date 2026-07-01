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

# Install rosdep-resolvable dependencies. ``python3-gurobipy`` is not available
# from the ROS/Ubuntu apt sources used here, so rosdep skips it and we install
# the official Python wheel below.
RUN rosdep init || true \
 && rosdep update \
 && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && apt-get update \
 && rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}" \
    --skip-keys python3-gurobipy \
 && rm -rf /var/lib/apt/lists/*

# Gurobi's pip package includes the restricted/free license path, which is
# enough for this controller's tiny QPs without requiring a separate license
# file. Pin below the next major to avoid unexpected wheel/API changes.
RUN python3 -m pip install --no-cache-dir 'gurobipy>=12,<13'

# Copy and build the package.
COPY . ${PMR_WS}/src/pmr_tp_final
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build --symlink-install --packages-select pmr_tp_final

# Automate workspace sourcing for interactive bash sessions (docker exec).
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "if [ -f ${PMR_WS}/install/setup.bash ]; then source ${PMR_WS}/install/setup.bash; fi" >> /root/.bashrc

WORKDIR ${PMR_WS}
CMD ["bash"]
