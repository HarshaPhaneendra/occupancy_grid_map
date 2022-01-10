#ifndef VISIBLE_MARKERS_H
#define VISIBLE_MARKERS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include<geometry_msgs/Point.h>
#include <cmath>
#include<bits/stdc++.h>
#include<string>

/* certain parameters of visualization_mags::Marker has to be unique or fed different values 
 created a struct to hold all such parameters */
struct marker_parameters
{
std::string pub_topic_name;
float initialize_pose_x;
float initialize_pose_y;
unsigned int red;
unsigned int green;
unsigned int blue;
};


class DynamicMarker
{
private:
    ros::NodeHandle* nh; // node handeler 
    ros::Publisher pub; // publisher type
    std::string topic_name;

    // Set our initial shape type for visualication Marker to be a sphere
    uint32_t shape = visualization_msgs::Marker::SPHERE;

public:

    DynamicMarker(ros::NodeHandle* n, std::string& pub_topic_name); //constructor 
    ~DynamicMarker(); // destructor 

    // to set initial paramerters of marker
    void InitializeMarker(visualization_msgs::Marker& marker,
        marker_parameters* marker_para); 

    void UpdateMarkerLocation(visualization_msgs::Marker& marker); // to update marker location

    /* 
        ChangeMarkerShape() an addition feature of changing marker shape
        It can be included by uncommenting its functin call
    */
    void ChangeMarkerShape(); 

    void PublishMarker(visualization_msgs::Marker& marker); // to publish visualization marker

};


#endif // end of VISIBLE_MARKERS_H