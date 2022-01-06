#!/bin/bash 

xterm -e "source devel/local_setup.bash; roscore" &
sleep 5
xterm -e "source devel/local_setup.bash; rosrun ogm_node ogm_node" &
sleep 5
xterm -e "source devel/local_setup.bash; rqt --clear-config"