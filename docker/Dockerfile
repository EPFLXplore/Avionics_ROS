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
FROM microros/micro-ros-agent:humble

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
# ROS2 dev workspace
# --------------------------------------------------------
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

WORKDIR /home/$USERNAME/dev_ws

# Auto-source dev_ws (Only need to source the user's workspace now)
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" \
    >> /home/${USERNAME}/.bashrc

# --------------------------------------------------------
# Remove confidential source code from the image
# (you mount your host src at runtime)
# --------------------------------------------------------
USER root
RUN rm -rf /home/$USERNAME/dev_ws/src/*

USER $USERNAME
WORKDIR /home/$USERNAME
