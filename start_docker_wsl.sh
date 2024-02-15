#!/bin/bash

if [ -d /mnt/wslg ]; then
    echo "The WSLg is available!"
else 
    echo "The WSLg folder is not found on defualt WSL 2. Rviz will not work!"
    exit 1
fi

echo "===================== Running the docker image ==========================="

docker run -it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="LIBGL_ALWAYS_SOFTWARE=1" \
--volume="/mnt/wslg:/mnt/wslg" \
--volume="/mnt/wslg/.X11-unix:/tmp/.X11-unix" \
--volume "./../robotique_experimentale:/home/catkin_ws/src/robotique_experimentale" \
--net="host" \
--name="tp_fanuc" \
tp_fanuc_docker

