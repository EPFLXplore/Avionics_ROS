# Avionics Docker

Image: `elec:humble-local` — based on `ros:humble`, adds ROS 2 Humble Desktop, the custom workspace (`custom_msg` + `python_node` + `avionics_nexus`), and Python serial/Modbus libraries. micro-ROS has been removed: the `avionics_nexus` bridge talks to the MCUs directly over USB-CDC.

Builds natively on both **x86-64 (PC)** and **arm64 (Raspberry Pi 5)**.

## Prerequisites

- Docker installed and daemon running
- A `cyclonedds.xml` file present in this directory (checked by `run.sh`)

A minimal `cyclonedds.xml` is already committed. Edit it if you need to restrict the network interface or tune DDS settings.

## Build

```bash
./build.sh
```

For a fully clean rebuild (no cache):

```bash
docker build --no-cache --network=host --progress=plain -t elec:humble-local -f Dockerfile ..
```

## Run

### 1. Run the docker container

```bash
./run.sh
```

What this does:
- Mounts `./src` → `/home/xplore/dev_ws/src` (live, symlinked)
- Mounts `/dev` so serial devices (e.g. `/dev/ttyACM0`) are accessible
- Mounts `cyclonedds.xml` → `/etc/cyclonedds/cyclonedds.xml` (read-only)
- Sets `ROS_DOMAIN_ID=0` and `CYCLONEDDS_URI` pointing at the mounted config
- Sets up X11 forwarding for GUI tools (rqt, rviz, etc.)
- Persists the home directory across runs via a named volume (`elec_humble_local_home_volume`)
- Runs with `--privileged --net=host` (required for DDS discovery and serial access)

To open a second shell into a running container:

```bash
./attach.sh
```

### 3. Source

```bash
source install/setup.bash
```

### 3. Run avionics

Launch the serial bridge (one Nexus node per master board, hardcoded in the launch file):

```bash
ros2 launch avionics_nexus bridge.launch.py
```

On another terminal launch the Python node:

```bash
ros2 launch python_node python_launch.py
```

If you edit `src/` and need to rebuild:

```bash
colcon build
```

## Clean reset

Removes the container, image, and home volume, then rebuilds from scratch:

```bash
docker rm -f elec_humble_local
docker rmi elec:humble-local
docker volume rm elec_humble_local_home_volume
./build.sh
```

## Note — handy messages

### Mass

Quick commands for poking the MCU by hand, with the bridge running.

**Read a value**

```bash
ros2 topic echo /EL/mass_packet
```

**Tare a load cell** (zero it — the scale must be EMPTY):

```bash
ros2 topic pub --once /EL/mass_req custom_msg/msg/MassRequest \
  "{id: 1, tare: true, change_scale: false, scale: 0.0}"
```

**Override a calibration slope** at runtime (what `calibrate_mass.sh` sends):

```bash
ros2 topic pub --once /EL/mass_req custom_msg/msg/MassRequest \
  "{id: 1, tare: false, change_scale: true, scale: 0.000517406}"
```

This lives in RAM on the MCU and is lost on reset. For a persistent value, put
it in `src/avionics_nexus/config/mass_cal.yaml` (`mass_slope_id_0` /
`mass_slope_id_1`). Nexus replays those to the MCU on every link-up, so a
power cycle or a reflash no longer costs you the calibration. That file is
bind-mounted, so editing a slope needs a node restart, not a `colcon build`.
The slopes compiled into `MassThread.h` are only the fallback used when nothing
is configured there.

### Servo

**Move a servo** (`angle` in degrees):

```bash
ros2 topic pub --once /EL/servo_req custom_msg/msg/ServoRequest \
  "{id: 1, angle: 90, go_to_zero: false}"
```

Back to zero:

```bash
ros2 topic pub --once /EL/servo_req custom_msg/msg/ServoRequest \
  "{id: 1, angle: 0, go_to_zero: true}"
```




