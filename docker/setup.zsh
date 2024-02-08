# xauth
xauth merge /dot.Xauthority

# ROS
# source /opt/ros/noetic/setup.zsh

# git
git config --global alias.graph "log --all --graph --decorate --oneline"

# add prefix to prompt
export PROMPT="[tp_fanuc_docker]$PROMPT"

# catkin_ws repository
cd /home/catkin_ws