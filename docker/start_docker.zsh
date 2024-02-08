docker run -it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="LIBGL_ALWAYS_SOFTWARE=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="$XAUTHORITY:/dot.Xauthority" \
--volume="./.zsh_history:/root/.zsh_history" \
--volume "./../robotique_experimentale:/home/catkin_ws/src/robotique_experimentale" \
--net="host" \
--name="tp_fanuc" \
tp_fanuc_docker
