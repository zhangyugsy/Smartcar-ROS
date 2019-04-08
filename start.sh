#! /bin/bash
# remove default gateway for wlan and velodyne-wired-connection
sudo route del default gw 192.168.1.1
# can setup
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 berr-reporting on fd on
sudo ip link set up can0
