/* 
    Occupancy grid map size 150m X 150m represented in 150X150 grid cells 
    Each cell resolution, 1m X 1m  
 */

#if !defined(OGM_NODE_H)
#define OGM_NODE_H

#include "ros/ros.h"
#include <bits/stdc++.h>
#include "std_msgs/String.h"

#include <sstream>
#include <fstream>
#include <iostream>

#include <math.h>
#include <vector>

// to visualize publish generated ogm
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "geometry_msgs/Point.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>


using namespace std;
using namespace cv;

// each grid represents 100cm x 100cm or 1 m x 1m of real world
const double gridWidth = 100, gridHeight = 100; 
const double mapWidth = 15000, mapHeight = 15000; // map width and height (cm) of real world 

// occupancy grid map size : 300 x 150 representing 150m X 150m of real world
const auto row_value = mapWidth/gridWidth;      // occupancy grid map row value count-> 150 
const auto col_value = mapHeight/gridHeight;    // occupancy map column value count -> 150

// ogm vector container 
std::vector<std::vector<double>> ogm_vec{row_value,std::vector<double>(col_value)};

// struct to hold x and y coordinates 
struct s_coordinates
{
    double x;
    double y;
};

// struct of 4 static vehicles of type 's_coordinates'
struct s_static_vehicle
{
    s_coordinates car_1;
    s_coordinates car_2;
    s_coordinates car_3;
    s_coordinates car_4;
    s_coordinates car_5;
    s_coordinates car_6;
    s_coordinates car_7;
    s_coordinates car_8;
};


class OGM_Class
{
private:
    
    image_transport::ImageTransport it_;    // image transport ptr
    image_transport::Publisher pub;         // publisher data type 
    sensor_msgs::ImagePtr msg;              // local Imgeptr
    bool parallel_cars = true;              // helps in ChangeToOccupied() function to make certain pixels occupied 

public:

    OGM_Class(image_transport::ImageTransport* image_transport);  // construtor 
    ~OGM_Class(); // destructor

    void AddStaticVehicles(s_static_vehicle& vehicle, 
        const double& row_value,
            const double& col_value);

    void AddPerpendicularVehicles(s_static_vehicle& vehicle, 
        const double& row_value,
            const double& col_value);

    void ChangeToOccupied(const double& vehicle_x, 
        const double& vehicle_y,
            const double& row_value,
                const double& col_value);

    void ColorCodeMap(cv::Mat& image, 
        const double& row_value,
            const double& col_value); // to color code Occupancy grid map 

    void PublishMsg(cv::Mat& image);

};



#endif // OGM_NODE_H
 
