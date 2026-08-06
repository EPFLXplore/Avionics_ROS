###########################################
# Xplore Elec/Dev image (Machine-dependent)
# - Works on x86 and ARM (RPi5) depending
#   on where you build it
# - Includes micro-ROS tools
###########################################
# --------------------------------------------------------
# CHANGE 1: Use the pre-built micro-ros-agent image as base
# It already includes a ROS 2 installation and the micro-ROS Agent.
# --------------------------------------------------------
FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive

# --------------------------------------------------------
# Optional: install full ROS desktop tools
# --------------------------------------------------------
RUN apt-get update && apt-get install -y ros-humble-desktop

# --------------------------------------------------------
# Fix ROS apt key issue (same trick as your old images)
# FIX: Cleaned up spacing and added curl dependency to prevent shell errors.
# --------------------------------------------------------
RUN apt-get update && apt-get install -y curl && \
    rm -f /usr/share/keyrings/ros-archive-keyring.gpg || true && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt update && apt install -y ros2-apt-source && \
    rm -f /etc/apt/sources.list.d/ros2.list && \
    rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# --------------------------------------------------------
# Create non-root user
# --------------------------------------------------------
ENV USERNAME=xplore
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# --------------------------------------------------------
# Common dependencies + python3-pip + rosdep
# --------------------------------------------------------
RUN apt-get update && apt-get install -y \
      git-core \
      bash-completion \
      libyaml-cpp-dev \
      iproute2 \
      kmod \
      busybox \
      libserialport-dev \
      libgstreamer1.0-dev \
      libgstreamer-plugins-base1.0-dev \
      python3-pip \
      python3-rosdep \
      python3-colcon-common-extensions \
      && ACCEPT_EULA=Y apt-get install -y ubuntu-restricted-extras

# ARM toolchain included for x86 builds; harmless on ARM
RUN apt-get install -y --no-install-recommends \
      gcc-arm-none-eabi \
      libstdc++-arm-none-eabi-newlib \
      libnewlib-arm-none-eabi \
      gdb-multiarch || true

# --------------------------------------------------------
# CHANGE 2: REMOVED MANUAL micro-ROS SETUP (now done by base image)
# --------------------------------------------------------

# --------------------------------------------------------
# Python deps (Modbus, serial, numpy…)
# --------------------------------------------------------
RUN pip3 install --no-cache-dir numpy pymodbus pyserial minimalmodbus

# Clean apt lists after all root-side apt work
RUN rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------
# Fix device access permissions (for dialout group)
# Ensures user 'xplore' can access serial devices like ttyACM0.
# --------------------------------------------------------
RUN DIALOUT_GID=$(getent group dialout | cut -d: -f3) || true; \
    if [ -n "$DIALOUT_GID" ]; then \
      groupadd --gid $DIALOUT_GID dialout || true && \
      usermod -aG dialout $USERNAME; \
    fi

# --------------------------------------------------------
# Switch to user and install same Python deps for user
# --------------------------------------------------------
USER $USERNAME
RUN pip3 install --no-cache-dir numpy pymodbus pyserial minimalmodbus

# --------------------------------------------------------
# Workspace skeleton
# --------------------------------------------------------
# The workspace is deliberately NOT built here. This image builds the
# environment; the workspace is built inside the container, against the src/
# that run.sh bind-mounts from the host. A colcon build at image build time is
# masked at runtime anyway: run.sh mounts a named volume over the whole home,
# and a named volume is seeded from the image only once, when it is first
# created, then never refreshed - so a baked install silently freezes at
# whatever the volume captured on day one.
WORKDIR /home/$USERNAME/dev_ws
RUN mkdir -p src

# .bashrc is the only place the environment gets sourced (run.sh/attach.sh just
# open a shell). The underlay must be unconditional: --entrypoint /bin/bash in
# run.sh bypasses the base image's ros_entrypoint.sh, so nothing else sources
# /opt/ros. The overlay is guarded - install/ does not exist until the first
# in-container colcon build.
RUN echo "source /opt/ros/humble/setup.bash" \
    >> /home/${USERNAME}/.bashrc && \
    echo "[ -f /home/${USERNAME}/dev_ws/install/setup.bash ] && source /home/${USERNAME}/dev_ws/install/setup.bash" \
    >> /home/${USERNAME}/.bashrc

USER $USERNAME
WORKDIR /home/$USERNAME
