#!/bin/sh

sudo busybox devmem 0x0c303000 32 0x0000C400
sudo busybox devmem 0x0c303008 32 0x0000C458
sudo busybox devmem 0x0c303010 32 0x0000C400
sudo busybox devmem 0x0c303018 32 0x0000C458

# sudo modprobe can
# sudo modprobe can_raw
# sudo modprobe mttcan

# sudo ip link set down can0
# sudo ip link set down can1

# sudo ip link set can0 type can bitrate 500000 sample-point 0.8 dbitrate 2000000 dsample-point 0.75 fd on restart-ms 100
# sudo ip link set can1 type can bitrate 500000 sample-point 0.8 dbitrate 2000000 dsample-point 0.75 fd on restart-ms 100

# sudo ifconfig can0 txqueuelen 1000
# sudo ifconfig can1 txqueuelen 1000

# sudo ip link set up can0 mtu 72
# sudo ip link set up can1 mtu 72

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set down can0
sudo ip link set down can1

#~ sudo ip link set can0 type can bitrate 500000 sample-point 0.875 dbitrate 2000000 dsample-point 0.9 berr-reporting on one-shot on fd on restart-ms 100
#~ sudo ip link set can1 type can bitrate 500000 sample-point 0.875 dbitrate 2000000 dsample-point 0.9 berr-reporting on one-shot on fd on restart-ms 100

sudo ip link set can0 type can bitrate 500000 sample-point 0.8 dbitrate 2000000 dsample-point 0.75 fd on restart-ms 100
sudo ip link set can1 type can bitrate 500000 sample-point 0.8 dbitrate 2000000 dsample-point 0.75 fd on restart-ms 100

sudo ifconfig can0 txqueuelen 1000
sudo ifconfig can1 txqueuelen 1000

sudo ip link set up can0 mtu 72
sudo ip link set up can1 mtu 72
