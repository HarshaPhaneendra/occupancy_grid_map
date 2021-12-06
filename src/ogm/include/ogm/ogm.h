#ifndef OGM_H
#define OGM_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <fstream>
#include <iostream>

#include <math.h>
#include <vector>

// to visualize publish generated ogm
//#include "ogm/matplotlibcpp.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "geometry_msgs/Point.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/msg/image.hpp>
 
using namespace std;
using namespace cv;
//namespace plt = matplotlibcpp;

// Defining Map Characteristics
double Zmax = 5000, Zmin = 170;
double l0 = 0, locc = 0.4, lfree = -0.4;
double gridWidth = 100, gridHeight = 100;
double mapWidth = 30000, mapHeight = 15000;
double robotXOffset = mapWidth / 5, robotYOffset = mapHeight / 3;
vector<vector<double> > l(mapWidth / gridWidth, vector<double>(mapHeight / gridHeight));

double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[]);
void occupancyGridMapping(double Robotx, double Roboty, double Robottheta, double sensorData[]);

void visualization(cv::Mat& image, const double& row_value, const double& col_value);


#endif