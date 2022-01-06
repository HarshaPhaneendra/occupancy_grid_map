# occupancy_grid_map 

## Build and Run
### Prerequisite 
* ROS disto - Kenitic (and above, checked in neotic)
* Cmake
* OpenCV 

### Terminal 1
* Download and Build the project
```
cd /home/<username>/catkin_ws/src
git clone git@github.com:HarshaPhaneendra/occupancy_grid_map.git
cd ..
catkin_make
source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/<ros-distro>/setup.bash" >> ~/.bashrc   # change '<ros-distro>' to installed version/ros distribution.
source devel/setup.bash
```

### Generation of OGM
* Change access permision for script file 
* Run the script
```
chmod +x src/scripts/ogm.sh       
./src/scripts/ogm.sh
```
