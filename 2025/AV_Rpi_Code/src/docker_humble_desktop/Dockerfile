###########################################
# Xplore Rover image Build Docker
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop as build-rover

ARG DEBIAN_FRONTEND=noninteractive

RUN sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN sudo apt update && sudo apt install ros2-apt-source  
RUN sudo rm /etc/apt/sources.list.d/ros2.list
RUN sudo rm /usr/share/keyrings/ros-archive-keyring.gpg

# Install Elec packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends
RUN apt-get -y install libyaml-cpp-dev iproute2 kmod busybox

RUN ACCEPT_EULA=Y apt install -y ubuntu-restricted-extras
RUN apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

# Install ARM GCC toolchain and libraries for STM32 cross-compiling
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    libnewlib-arm-none-eabi \
    gdb-multiarch

RUN apt-get update && apt-get install -y libserialport-dev

USER $USERNAME

RUN python3 -m pip install numpy pymodbus pyserial

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .


# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Add the source of the project to the .bashrc
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*