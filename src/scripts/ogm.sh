#!/bin/bash 

xterm -e "source devel/local_setup.bash; roscore" &
sleep 5
xterm -e "source devel/local_setup.bash; rosrun ogm_node ogm_node" &
sleep 5
xterm -e "source devel/local_setup.bash; rosrun visible_markers visible_markers" &
sleep 5
xterm -e "source devel/local_setup.bash; rviz rviz" &
sleep 5
xterm -e "source devel/local_setup.bash; rqt"