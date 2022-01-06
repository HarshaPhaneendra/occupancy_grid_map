#include "ogm_node/ogm_node.h"

// constructor 
OGM_Class::OGM_Class(image_transport::ImageTransport* image_transport):it_(*image_transport)
{
    ROS_INFO("ogm constructor is called.");
    
    pub = it_.advertise("OGM/ogm_image", 1);
}

//adding static vehicle to ogm 
void OGM_Class::AddStaticVehicles(
    const double& row_value,
        const double& col_value)
{
    // starting coordinates of vehicle 1
    vehicle.car_1.x = 10;
    vehicle.car_1.y = 10;

    vehicle.car_2.x = 10;
    vehicle.car_2.y = 20;

    vehicle.car_3.x = 10;
    vehicle.car_3.y = 30;

    vehicle.car_4.x = 10;
    vehicle.car_4.y = 40;

    OGM_Class::ChangeToOccupied(vehicle.car_1.x,vehicle.car_1.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_2.x,vehicle.car_2.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_3.x,vehicle.car_3.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_4.x,vehicle.car_4.y, row_value, col_value);
}

void OGM_Class::ChangeToOccupied(
    const double& vehicle_x, 
        const double& vehicle_y,
            const double& row_value,
                const double& col_value)
{
    if (vehicle_x < (row_value-6) && vehicle_y < (col_value-1))
    {
        for (int j = 0; j < 3; j++)
        {
            for (int i = 0; i < 6; i++)
            {
                ogm_vec[vehicle_x+i][vehicle_y+j] = 1;
            }
        }
    }
    
}


//loop function 
void OGM_Class::ColorCodeMap(
    cv::Mat& image, 
        const double& row_value, 
            const double& col_value)
{
    /* ogm_vec[10][50] = 1;
    ogm_vec[10][100] = 1;
    ogm_vec[10][147] = 1;
    ogm_vec[10][148] = 1;
    ogm_vec[10][149] = 1;
    ogm_vec[10][150] = 1; */

    for (double x = 0; x < row_value; x++)
    {
        for (double y = 0; y < col_value; y++)
        {
            // fetch pixel color 
            cv::Vec3b color = image.at<cv::Vec3b>(Point(y,x));

            if( ogm_vec[x][y] == 0)        // free cells as green pixels
            {
                color[0] = 50;
                color[1] = 255;     
                color[2] = 50;
            }
            else if (ogm_vec[x][y] > 0)   // obstacel cells as red pixels
            {
                color[0] = 50;
                color[1] = 50;
                color[2] = 255;     
            }
            

            // set pixel value back to the image
            image.at<cv::Vec3b>(Point(y,x)) = color;
        }
        
    }
}

void OGM_Class::PublishMsg(cv::Mat& image)
{
    // converting cv::mat to sensor_msgs::image to publish on ROS
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ogm_node");
    ros::NodeHandle nh_; // creating node handle
    image_transport::ImageTransport it_(nh_); 

    cv::Mat image(col_value, row_value, CV_8UC3);
    cv::waitKey(30); 
  
    OGM_Class ogm_obj(&it_);
    ros::Rate loop_rate(10);

    ogm_obj.AddStaticVehicles(row_value,col_value);
    ogm_obj.ColorCodeMap(image,row_value,col_value);
    
    while (ros::ok())
    {
        ogm_obj.PublishMsg(image);
        ros::spinOnce();
        loop_rate.sleep();

    }
    
    

    return 0;
}