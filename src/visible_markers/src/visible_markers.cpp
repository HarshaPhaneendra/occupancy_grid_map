/*

  "visible_markers" ROS Node,
  To keep track of dynamic objects (may it be person or car).
  ROS compatible 'visualization_msgs::Marker' is the best way to keep track of moving object.

  Output : 
    This node Publishes 2 topics of type <visualization_msgs::Marker> 
      It can be visualized in 'rviz', 
      ADD->by_topic->visualization_marker_1->Marker
      ADD->by_topic->visualization_marker_2->Marker
      Which creates layers on top of fixed_frame -> "/map".
      This "/map" is a default one given by rviz, not the generated one!!
  
  Assuptions: 
    Dynamic objects velocity is known, so that its location at every sec is know.
    Henceforth Instead of updating dynamic locations of moving objects and then indicating markers at these locations.
    The location of marker is been hard coded and updated on to follow rectangular shape.
    This indicates the dynamic object movement

  Output missmatch: 
    -> output of this node creates a layer on top of "/map" - indicated dynamic movements of objects. 
    -> output from "ogm" node, publishes an image under <sensor_msgs::image> which indicates
      'occupancy grid map' with static objects. 
    -> These two topics are visualized in separate window!! 
      It has to be layered on one another to get the acctual feel of occupancy map with static and dynamic objects.
*/

#include "visible_markers/visible_markers.h"


/* 
  DynamicMarker::DynamicMarker() is a constructor 
  Ipnut Parameters:
    ros::NodeHandle 
*/
DynamicMarker::DynamicMarker(ros::NodeHandle* n, std::string& pub_topic_name)
  :nh(n),topic_name(pub_topic_name)
{
    //ROS_INFO("DynamicMarker is Created with the topic name: " + topic_name);
    std::cout << "DynamicMarker with the topic: " << topic_name << " is created." <<"\n";
    // publisher is initialized 
    pub = nh->advertise<visualization_msgs::Marker>(topic_name, 1);
}

/* 
  DynamicMarker::~DynamicMarker() is a default disstructor to free up the memory utilised
 */
DynamicMarker::~DynamicMarker()
{
  ROS_INFO("DynamicMarker is Destroyed!");
}

/* 
  DynamicMarker::InitializeMarker() is to set initial parameters of visualization Marker
  Input Parameter: 
    visualization_msgs::Marker
*/
void DynamicMarker::InitializeMarker(
  visualization_msgs::Marker& marker,
    marker_parameters* marker_para)
{

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "visible_markers";
  //marker.id = 0;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = marker_para->initialize_pose_x;
  marker.pose.position.y = marker_para->initialize_pose_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = marker_para->red;
  marker.color.g = marker_para->green;
  marker.color.b = marker_para->blue;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

}

/* 
  DynamicMarker::UpdateMarkerLocation() is to regular update in marker's location, 
  here marker represent Dynamic movement of an object (person)
  Input Parameter:
    visualization_msgs::Marker 
*/ 
void DynamicMarker::UpdateMarkerLocation(visualization_msgs::Marker& marker)
{
  if (marker.pose.position.x < 5.0 && marker.pose.position.y == 0.0)
    marker.pose.position.x = marker.pose.position.x + 1.0;
  else if(marker.pose.position.x == 5.0 && marker.pose.position.y < 5.0)
    marker.pose.position.y = marker.pose.position.y + 1.0;
  else if(marker.pose.position.y == 5.0 && marker.pose.position.x > 0)
    marker.pose.position.x = marker.pose.position.x - 1.0;
  else if(marker.pose.position.x == 0.0 && marker.pose.position.y >0)
    marker.pose.position.y = marker.pose.position.y - 1.0;
  else
  {
    marker.pose.position.x = 0.0; // default value @ (0,0)
    marker.pose.position.y = 0.0;
  }
}

/* 
  DynamicMarker::ChangeMarkerShape() is an addtional feature
  which keeps on updating visualization-marker 'shape' by cycles between CUBE, SPHERE, ARROW, CYLINDER
  Can be implemented just by uncommenting its function call
*/
void DynamicMarker::ChangeMarkerShape()
{
  // Cycle between different shapes
  switch (shape)
  {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
  }
} 

/*
  DynamicMarker::PublishMarker() is to update the marker shape, add marker.action 
  and finially to publish visualization_msgs::Marker
  Input Parameter: 
    visualization_msgs::Marker
*/
void DynamicMarker::PublishMarker(visualization_msgs::Marker& marker)
{
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER  
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  pub.publish(marker);
  
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "visible_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  
  // visualization_mags::Marker as local ptr 
  visualization_msgs::Marker marker_1, marker_2;
  marker_1.id = 0; // each marker should have unique id
  marker_2.id = 1;

  marker_parameters s_marker_1 = {"visualization_marker_1", +0.0, +0.0, (unsigned int)1.0f, (unsigned int)0.0f, (unsigned int)0.0f};
  marker_parameters s_marker_2 = {"visualization_marker_2", +5.0, +0.0, (unsigned int)0.0f, (unsigned int)0.0f, (unsigned int)1.0f};
  
  // DynamicMarker instance using unique_ptr 
  std::unique_ptr<DynamicMarker> marker_obj_1 = std::make_unique<DynamicMarker>(&n, s_marker_1.pub_topic_name);
  std::unique_ptr<DynamicMarker> marker_obj_2 = std::make_unique<DynamicMarker>(&n, s_marker_2.pub_topic_name);

  marker_obj_1->InitializeMarker(marker_1, &s_marker_1);
  
  marker_obj_2->InitializeMarker(marker_2, &s_marker_2);
  
  while (ros::ok())
  {

    marker_obj_1->PublishMarker(marker_1);
    marker_obj_2->PublishMarker(marker_2);

    // UpdateMarkerLocation() has be called after publishing marker
    marker_obj_1->UpdateMarkerLocation(marker_1); 
    marker_obj_2->UpdateMarkerLocation(marker_2); 
    
    /* 
      Its a fun feature, just uncomment below function calls to implement
      but keeping it commented as of now 
    */
    /* marker_obj_1->ChangeMarkerShape();
    marker_obj_2->ChangeMarkerShape();  */

    r.sleep();
  }

}