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

## Inside the container

The workspace is **auto-sourced** via `.bashrc` — no manual `source` needed for `dev_ws`.

Launch the serial bridge (one Nexus node per master board, hardcoded in the launch file):

```bash
ros2 launch avionics_nexus bridge.launch.py
```

Launch the Python node:

```bash
ros2 launch python_node python_launch.py
```

No micro-ROS agent is needed — `avionics_nexus` opens the MCU's USB-CDC port
(`/dev/ttyACM0`, or a `/dev/serial/by-id/...` path) directly.

### Workspace packages

| Package | Description |
|---------|-------------|
| `custom_msg` | Custom ROS 2 message/service/action definitions |
| `avionics_nexus` | USB-serial bridge: MCU master boards ↔ ROS 2 topics |
| `python_node` | Main avionics Python node |

### Python libraries available

`numpy`, `pymodbus`, `pyserial`, `minimalmodbus`

## Rebuild workspace inside the container

If you edit `src/` and need to rebuild:

```bash
cd ~/dev_ws && colcon build --symlink-install
```

## Clean reset

Removes the container, image, and home volume, then rebuilds from scratch:

```bash
docker rm -f elec_humble_local
docker rmi elec:humble-local
docker volume rm elec_humble_local_home_volume
./build.sh
```
