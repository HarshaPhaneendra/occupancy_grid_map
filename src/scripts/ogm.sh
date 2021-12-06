#!/bin/bash

# exporting gazebo world along with robot 
xterm  -e  "source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 10
xterm -e "source devel/setup.bash; roslaunch my_robot teleop.launch" & 
sleep 5
xterm -e "source devel/setup.bash; rosrun ogm ogm" &
sleep 5
xterm -e "source devel/setup.bash; rqt --clear-config"