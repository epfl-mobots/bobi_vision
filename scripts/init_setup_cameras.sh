#!/bin/bash

set -e

# Verify that PylonViewerApp is not launched
isPylonLaunched=`ps ax | grep PylonViewerApp | grep -v grep | sed 's/[[:space:]]*$//'`
if [ ! -z "$isPylonLaunched" ]; then
        echo PylonViewerApp already launched. Please quit PylonViewerApp before attempting to run this script.
        exit 1
fi

if [[ $(pidof gst-launch-1.0) ]]; then
        echo Stopping gstreamer active streams
        sudo kill -9 `pidof gst-launch-1.0`
fi

sudo modprobe -r uvcvideo
sudo modprobe -r v4l2loopback
sleep 2
sudo modprobe v4l2loopback devices=5 exclusive_caps=0,0,0,0,0
sleep 2

# gst-launch-1.0 pylonsrc camera=0 sensorreadoutmode=normal exposure=30110 gamma=0.61 gain=9.0  width=2848 height=2848 offsetx=548 offsety=100 fps=30 flipy=true flipx=true imageformat=mono8 \
# # 		 ! videoconvert ! tee ! v4l2sink device=/dev/video0 sync=false &
# gst-launch-1.0 pylonsrc camera=0 sensorreadoutmode=normal exposure=30110 gamma=0.61 gain=12.0  width=2848 height=2848 offsetx=548 offsety=100 fps=30 flipy=true flipx=true imageformat=mono8 \
# 		 ! videoconvert ! tee ! v4l2sink device=/dev/video0 sync=false &
gst-launch-1.0 pylonsrc camera=0 sensorreadoutmode=normal exposure=30110 gamma=0.55 gain=1.2  width=2848 height=2848 offsetx=640 offsety=0 fps=30 flipy=true flipx=true imageformat=mono8 \
		 ! videoconvert ! tee ! v4l2sink device=/dev/video0 sync=false &
sleep 8

# gst-launch-1.0 v4l2src device=/dev/video0 ! videoscale ! video/x-raw, format='GRAY8,width=1200,height=1200,framerate=30/1' ! videoconvert ! tee ! v4l2sink device=/dev/video1 sync=false &
# sleep 2

gst-launch-1.0 v4l2src device=/dev/video0 ! videoscale ! video/x-raw, format='GRAY8,width=512,height=512,framerate=30/1' ! videoconvert ! tee ! v4l2sink device=/dev/video2 sync=false &
sleep 8

gst-launch-1.0 v4l2src device=/dev/video0 ! videoscale ! video/x-raw, format='GRAY8,width=1500,height=1500,framerate=30/1' ! videoconvert ! tee ! v4l2sink device=/dev/video4 sync=false &
sleep 8

sudo modprobe uvcvideo

sleep 1

v4l2-ctl -d /dev/video5 --set-fmt-video=width=640,height=480,pixelformat='YUYV'
v4l2-ctl -d /dev/video5 -c exposure_absolute=305

gst-launch-1.0 v4l2src device=/dev/video5 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoflip method=horizontal-flip ! videoconvert ! tee ! v4l2sink device=/dev/video3 sync=false &

