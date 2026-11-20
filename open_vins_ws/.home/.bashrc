source /opt/ros/noetic/setup.bash
[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash
[ -e ~/datasets ] || ln -s /datasets ~/datasets
PS1='\[\e[1;32m\]ov-docker\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ '
cd /catkin_ws
