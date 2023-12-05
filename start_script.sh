#!/bin/bash
pacmd set-default-sink alsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo
pacmd set-sink-volume 1 0x10000
. /home/nvidia/INF3995-104/embedded/agilex_ws/devel/setup.bash 
sudo chmod 666 /dev/ttyTHS1
sudo usermod -a -G dialout nvidia
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 &
roslaunch environment_setting launch_cmds.launch &

