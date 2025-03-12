###########################################
# Xplore Rover image Build Docker
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop as build-rover

ARG DEBIAN_FRONTEND=noninteractive

# Install Elec packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends
RUN apt-get -y install libyaml-cpp-dev iproute2 kmod busybox

RUN ACCEPT_EULA=Y apt install -y ubuntu-restricted-extras
RUN apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart

# Install ARM GCC toolchain and libraries for STM32 cross-compiling
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    libnewlib-arm-none-eabi \
    gdb-multiarch

# RUN git clone https://github.com/STMicroelectronics/STM32CubeH7.git /opt/STM32CubeH7
# WORKDIR /opt/STM32CubeH7
# RUN git submodule update --init --recursive

# RUN ls -al /opt/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver
# RUN ls -al /opt/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Inc

# ENV STM32CUBEH7_PATH=/opt/STM32CubeH7

# RUN ln -s /opt/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Inc /usr/local/include/stm32h7_hal

# WORKDIR /home/xplore/dev_ws

USER $USERNAME

RUN python3 -m pip install numpy

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .


# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Add the source of the project to the .bashrc
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# Install ROS 2 Nav dependencies
RUN rosdep update && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y --skip-keys "libg2o g2o"

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*