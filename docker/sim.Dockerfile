FROM osrf/ros:humble-desktop AS crazysim

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV CRAZYSIM_HOME=/CrazySim

# Newer Intel/AMD iGPUs may need a newer Mesa userspace than Ubuntu Jammy ships.
# kisak-mesa "fresh" no longer publishes Jammy packages, so use the stable PPA.
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
 && add-apt-repository -y ppa:kisak/turtle \
 && apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libegl-mesa0 \
    libgbm1 \
    mesa-vulkan-drivers \
    mesa-utils \
 && rm -rf /var/lib/apt/lists/*

# Base build/ROS tools plus Gazebo Garden (CrazySim's Gazebo backend).
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    gnupg \
    vim \
    lsb-release \
    python-is-python3 \
    python3-colcon-common-extensions \
    python3-dev \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    libboost-program-options-dev \
    libgl1 \
    libusb-1.0-0-dev \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    ros-humble-motion-capture-tracking \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends gz-garden \
 && rm -rf /var/lib/apt/lists/*

# Python pieces used by CrazySim, cflib/Crazyswarm2, and the MuJoCo backend.
RUN python3 -m pip install --no-cache-dir --upgrade pip \
 && python3 -m pip install --no-cache-dir \
    Jinja2 \
    mujoco \
    "numpy>=2" \
    tomli \
    rowan \
    "transforms3d>=0.4.2"

# CrazySim includes the modified SITL firmware and Crazyswarm2 as submodules.
RUN git clone --recursive https://github.com/gtfactslab/CrazySim.git "${CRAZYSIM_HOME}"

# cflib must be installed from source for CrazySim's UDP SITL transport support.
RUN git clone https://github.com/bitcraze/crazyflie-lib-python.git "${CRAZYSIM_HOME}/crazyflie-lib-python" \
 && cd "${CRAZYSIM_HOME}/crazyflie-lib-python" \
 && SETUPTOOLS_SCM_PRETEND_VERSION=0.1.31 python3 -m pip install --no-cache-dir -e . \
 && python3 -m pip install --no-cache-dir --force-reinstall "numpy>=2" "transforms3d>=0.4.2"

# Optional Crazyflie desktop client, pinned to the commit CrazySim documents.
RUN git clone https://github.com/bitcraze/crazyflie-clients-python.git "${CRAZYSIM_HOME}/crazyflie-clients-python" \
 && cd "${CRAZYSIM_HOME}/crazyflie-clients-python" \
 && git checkout d649b6615a58ac0eb34aa72a4edef4c5d821eeab \
 && python3 -m pip install --no-cache-dir -e . \
 && python3 -m pip install --no-cache-dir --force-reinstall "numpy>=2" "transforms3d>=0.4.2"

# Build the SITL firmware.
RUN cd "${CRAZYSIM_HOME}/crazyflie-firmware" \
 && mkdir -p sitl_make/build \
 && cd sitl_make/build \
 && cmake .. \
 && make -j"$(nproc)" all

# Install ROS package dependencies and build the bundled Crazyswarm2 workspace.
RUN rosdep init || true \
 && rosdep update \
 && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && cd "${CRAZYSIM_HOME}/crazyswarm2_ws" \
 && rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}" \
 && apt-get purge -y python3-transforms3d || true \
 && python3 -m pip install --no-cache-dir --force-reinstall "numpy>=2" "transforms3d>=0.4.2" \
 && apt-get update \
 && apt-get install -y --reinstall ros-humble-tf-transformations \
 && python3 -m pip install --no-cache-dir --force-reinstall "numpy>=2" "transforms3d>=0.4.2" \
 && rm -rf /var/lib/apt/lists/* \
 && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Automate workspace sourcing for interactive bash sessions (docker exec).
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "if [ -f ${CRAZYSIM_HOME}/crazyswarm2_ws/install/setup.bash ]; then source ${CRAZYSIM_HOME}/crazyswarm2_ws/install/setup.bash; fi" >> /root/.bashrc

WORKDIR ${CRAZYSIM_HOME}
CMD ["bash"]
