# Interact with the MCU with a microROS Agent

- [Micro-ROS documentation on Notion](https://www.notion.so/xplore-doc/Electronics-Main-Page-9bd09037225f44079b4082249d664c20?p=2a010d0104f880479e73e5e9036b130a&pm=s).

## Install microROS on the device thats gonna talk with the MCU

Once you have a ROS 2 installation in the computer, follow these steps to install the micro-ROS build system:

```cpp
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

## Create a microROS Agent

**On the microros_ws folder you already created (point 5.)**

First build tools and source if you haven’t already:

```cpp
# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

Then create the agent, build it and source.

```cpp
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

## Use it to read the publisher on the MCU

```cpp
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```

Where `[device]` is the USB port you connected the connector corresponding to the USB CDC of your board. You can find it with the following command comparing the outputs with and without the connected board

```cpp
ls /dev/tty*
```

You should also see a difference when doing `lsusb`, if the command line blocks when doing that the USB enumeration failed: check your USB configuration of your STM.

When running the microROS command that terminal window is now blocked, open a new one, source and you are ready to interact with the board!

```cpp
source install/local_setup.bash

#List topics
ros2 topic list

#Read "Hello from STM32H7!"
ros2 topic echo /chatter
```
