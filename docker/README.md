# Avionics Docker

Image: `elec:humble-local` — includes ROS 2 Humble, micro-ROS Agent, and the custom workspace.

## Build

```bash
./build.sh
```

For a fully clean rebuild (no cache):

```bash
docker build --no-cache --progress=plain -t elec:humble-local -f Dockerfile ..
```

## Run

```bash
./run.sh
```

Mounts `./src` into `/home/xplore/dev_ws/src` inside the container.

To open a second shell into a running container:

```bash
./attach.sh
```

## Inside the container

Source the workspace:

```bash
source ~/dev_ws/install/setup.bash
```

Launch the Python node:

```bash
ros2 launch python_node python_launch.py
```

Run the micro-ROS agent (second terminal):

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

## Clean reset

```bash
docker rm -f elec_humble_local
docker rmi elec:humble-local
docker volume rm elec_humble_local_home_volume
./build.sh
```
