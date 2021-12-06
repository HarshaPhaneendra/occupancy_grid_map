# occupancy_grid_map

This program it to create Occupancy grid map in ROS system.

As per the whole model/task concerns, Sensor published data is been fed to 'sensor measurement model' to get 
'sensor measurement values'. These values inturn fed to generate "occupancy grid map" using 'inverse measurement model'. 

I appoligize, its unfortunate that I couldn't converte series of complex mathematical equations of 'sensor measurement model' to code.
I will continue to figure this out.

But at the meanwhile, I have taken a set of pre-stored 'sensor measurement values' 
along with 'robot pose' values to generate 'occupancy grid map' using 'inverse measurement model'.
Values of OGM is been stored in 2d vector, later map is been created using opencv::mat lib.
At the end, cv::mat image is been converted to sensor_msgs/image to be published 
on ROS topic "ogm/ogm_image".

## Build and Run
### Terminal 1
* Download and Build the project
```
cd /home/<username>/catkin_ws/src
git clone git@github.com:HarshaPhaneendra/occupancy_grid_map.git -b master
cd ..
catkin_make
source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/<ros-distro>/setup.bash" >> ~/.bashrc   # change '<ros-distro>' to installed version/ros distribution.
source devel/setup.bash
```
* Change access permision for script files 
* Run appropriate script 
```
chmod +x src/scripts/ogm.sh       # Generation of OGM using pre-stored measurement values 
chmod +x src/scripts/launch.sh    # Generation of OGM using 'rtab_map' ROS-package 
./src/scripts/<file_name>
```
