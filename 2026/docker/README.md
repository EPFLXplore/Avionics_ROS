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
it in `src/avionics_nexus/config/mass_cal.yaml` (`mass_slope_sand_rocks` /
`mass_slope_drill`). Nexus replays those to the MCU on every link-up, so a
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

### LEDs

**Light a segment** — here the NAV segment, solid on:

```bash
ros2 topic pub --once /EL/led_req custom_msg/msg/LEDRequest "{system: 0, mode: 1}"
```

`LEDRequest` carries no id: there is one strip in the rover, so the message says
which segment (`system`) and what it does (`mode`).

| `system` | segment | | `mode` | pattern |
|---|---|---|---|---|
| 0 | NAV | | 0 | off |
| 1 | HD | | 1 | on |
| 2 | DRILL | | 2 | blink |
| 3 | AVIONICS | | 3 | fault |
| | | | 4 | emergency motors (whole strip) |
| | | | 5 | emergency shutdown (whole strip) |

Modes 4 and 5 are not tied to a segment, so `system` is ignored for those.

## ID mapping

The `id` byte in `ServoRequest`, `MassRequest` and `MassPacket` names a **device**,
fleet-wide. Which board serves it, and through which connector, is that board's
own business — nothing on the wire carries it. The authoritative enums are
`MassIdType` / `ServoIdType` in `src/custom_msg/include/device_ids.h`, which both
the RPi and the firmware compile against.

**Servos** (`/EL/servo_req`)

| id | device | board |
|---|---|---|
| 0 | front camera | 3 |
| 1 | drill | 0 |
| 2 | left service module | 3 |
| 3 | right service module | 3 |

**Load cells** (`/EL/mass_req`, `/EL/mass_packet`)

| id | device | board |
|---|---|---|
| 0 | sand & rocks | 3 |
| 1 | drill | 0 |

**No id**: the pH probe (board 3) and the LED strip (board 3) are single
instances, so their messages carry no `id` field at all.

The board column is which master currently serves the device — useful when only
one board is powered or you are watching a single bridge node. It is not part of
the protocol: commands go to every master and the one that owns the id acts on
it. Which connector a device sits on is not listed at all. Both come from
`PROFILES[]` in the firmware's `BoardProfile.h`, which is where to look when
hardware moves — the id stays put when it does.




