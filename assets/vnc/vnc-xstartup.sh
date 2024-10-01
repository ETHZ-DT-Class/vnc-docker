#!/bin/bash

# set to as configured in Xvfb
export DISPLAY=:0

# https://forums.developer.nvidia.com/t/jetpack-4-3-mesa-loader-failed-to-open-swrast-while-in-xrdp-session/111199/42
# It seems it has to be run inside the docker container to have effect
ln -sf /usr/lib/aarch64-linux-gnu/libdrm.so.2.4.0 /usr/lib/aarch64-linux-gnu/libdrm.so.2

# optional resource loading
if [ -f $HOME/.Xresources ]; then
    xrdb $HOME/.Xresources
fi

# start the session
startxfce4 &

# fascilitate copy-paste
autocutsel -fork

# build simulator
cd /code/catkin_ws && catkin build simulator && source /code/catkin_ws/devel/setup.bash

# Keep the script running
tail -f /dev/null
