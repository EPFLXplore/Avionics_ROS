# Interact with the MCU with a microROS Agent

- [Micro-ROS documentation on Notion for a generic setup](https://www.notion.so/xplore-doc/Electronics-Main-Page-9bd09037225f44079b4082249d664c20?p=2a010d0104f880479e73e5e9036b130a&pm=s).

# Launch the agent from the docker

```cpp
source /opt/ros/humble/setup.bash
source /uros_ws/install/local_setup.bash
```

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
source /opt/ros/humble/setup.bash

#List topics
ros2 topic list

#Read "Hello from STM32H7!"
ros2 topic echo /chatter
```
