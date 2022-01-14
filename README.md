# occupancy_grid_map 

## Build and Run
### Prerequisite 
* ROS disto - Kenitic (and above, checked in noetic)
* Cmake
* OpenCV 

## Terminal 1
### Download and Build the project
```
cd /home/<username>
mkdir occupancy_grid_map
cd occupancy_grid_map
git clone git@github.com:HarshaPhaneendra/occupancy_grid_map.git
catkin_make
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc   # change 'noetic' to installed ros distribution.
source devel/setup.bash
```
### To Run the project
* Change access permision for script file 
* Run the script
```
chmod +x src/scripts/ogm.sh       
./src/scripts/ogm.sh
```
### To visualize Markers (i.e dynamic movements of objects)
* Go to 'rviz' window (created under 'ogm.sh')
* ADD->By_topic->visualization_marker_1->Marker
* ADD->By_topic->visualization_marker_2->Marker

### To visualize generated occupancy grid map (pub from 'ogm' node)
* Go to 'rqt' window 
* Select topic 'OGM/ogm_image'

