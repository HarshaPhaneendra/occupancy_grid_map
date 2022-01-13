/* 
    "ogm_node" ROS Node generates 'occupancy grid map' for pre-found/ pre-assumed static objects
    
    Generated 'Occupancy grid map' is of 
        size 150m X 150m represented in 150X150 grid cells 
        Each cell resolution, 1m X 1m
    
    Assumtion: 
        Locations of Static objects (may it be person/vehicle) are know,
            henceforth hard coded - Represented by red pixels 
        Assuming whole/entire map region has been scaned and assumed free, 
            execpt the locations of static objects - Represented by green pixels
        'Occupancy grid map'- size and its resolutions are assumed.

    Output: 
        Published topic "OGM/ogm_image" can be visualized using rqt 
        Certain static-objects(cars) parked parallely and perpendicularly - have been represented in red block of pixels
        Rest of the map region has assumed free and represented by green pixels

 */
#include "ogm_node/ogm_node.h"

/*
    OGM_Class::OGM_Class() is constructor 
    Input Parameter:
        image_transport::ImageTransport* image_transport
*/
OGM_Class::OGM_Class(image_transport::ImageTransport* image_transport):it_(*image_transport)
{
    ROS_INFO("ogm constructor is called.");
    
    pub = it_.advertise("OGM/ogm_image", 1);
}

/* 
    OGM_Class::~OGM_Class() is a Destructor 
*/
OGM_Class::~OGM_Class()
{
    ROS_INFO("OGM_Class is Destroyed!");
}

/* 
    OGM_Class::AddStaticVehicles() is to add static vehicle's/objects location to OGM - hard coded
    Input Parameters:
        s_static_vehicle& vehicle,
        const double& row_value,
        const double& col_value
 */
void OGM_Class::AddStaticVehicles(
    s_static_vehicle& vehicle,
        const double& row_value,
            const double& col_value)
{
    parallel_cars = true;
    // starting coordinates of vehicle 1
    vehicle.car_2.x = 10;
    vehicle.car_2.y = 20;

    vehicle.car_3.x = 10;
    vehicle.car_3.y = 30;

    vehicle.car_4.x = 10;
    vehicle.car_4.y = 40;

    vehicle.car_5.x = 10;
    vehicle.car_5.y = 50;

    vehicle.car_6.x = 10;
    vehicle.car_6.y = 60;

    vehicle.car_7.x = 10;
    vehicle.car_7.y = 70;

    vehicle.car_8.x = 10;
    vehicle.car_8.y = 80;

    vehicle.car_1.x = 10;
    vehicle.car_1.y = 90;

    OGM_Class::ChangeToOccupied(vehicle.car_1.x,vehicle.car_1.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_2.x,vehicle.car_2.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_3.x,vehicle.car_3.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_4.x,vehicle.car_4.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_5.x,vehicle.car_5.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_6.x,vehicle.car_6.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_7.x,vehicle.car_7.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_8.x,vehicle.car_8.y, row_value, col_value);

}

/* 
    OGM_Class::AddPerpendicularVehicles() is to add perpendicularly parked static vehicels to OGM,
    locations are hard coded
    Input Parameters: 
        s_static_vehicle& vehicle, 
        const double& row_value,
        const double& col_value
 */
void OGM_Class::AddPerpendicularVehicles(
    s_static_vehicle& vehicle, 
        const double& row_value,
            const double& col_value)
{
    parallel_cars = false;
    vehicle.car_1.x = 20;
    vehicle.car_1.y = 10;

    vehicle.car_2.x = 30;
    vehicle.car_2.y = 10;

    vehicle.car_3.x = 40;
    vehicle.car_3.y = 10;

    vehicle.car_4.x = 50;
    vehicle.car_4.y = 10;

    vehicle.car_5.x = 60;
    vehicle.car_5.y = 10;

    vehicle.car_6.x = 70;
    vehicle.car_6.y = 10;

    vehicle.car_7.x = 80;
    vehicle.car_7.y = 10;

    vehicle.car_8.x = 90;
    vehicle.car_8.y = 10;

    OGM_Class::ChangeToOccupied(vehicle.car_1.x,vehicle.car_1.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_2.x,vehicle.car_2.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_3.x,vehicle.car_3.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_4.x,vehicle.car_4.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_5.x,vehicle.car_5.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_6.x,vehicle.car_6.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_7.x,vehicle.car_7.y, row_value, col_value);
    OGM_Class::ChangeToOccupied(vehicle.car_8.x,vehicle.car_8.y, row_value, col_value);

}


/* 
    OGM_Class::ChangeToOccupied() is to make bunch of pixels occupied, 
    given that starting cordinates of vehicle
    Input Parameters:
        const double& vehicle_x, 
        const double& vehicle_y,
        const double& row_value,
        const double& col_value)
 */
void OGM_Class::ChangeToOccupied(
    const double& vehicle_x, 
        const double& vehicle_y,
            const double& row_value,
                const double& col_value)
{
    if( parallel_cars)
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
    else
    {
        if (vehicle_x < (row_value-3) && vehicle_y < (col_value-5))
        {
            for (int j = 0; j < 3; j++)
            {
                for (int i = 0; i < 6; i++)
                {
                    ogm_vec[vehicle_x+j][vehicle_y+i] = 1;
                }
            }
        }

    }
    
    
}

/* 
    Color-code pixels of occupancy grid map using CV 
    Input Parameters:
        cv::Mat& image, 
        const double& row_value, 
        const double& col_value
*/
void OGM_Class::ColorCodeMap(
    cv::Mat& image, 
        const double& row_value, 
            const double& col_value)
{

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

/* 
    OGM_Class::PublishMsg() is used for convertion of cv::mat image to sensor_msgs::image 
    and to publish sensor_mags::image
    Input Parameters:
        cv::Mat& image
*/
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

    // create a CV::mat image with '8 unsigned integer image with 3 channels(RGB)'
    cv::Mat image(col_value, row_value, CV_8UC3);
    cv::waitKey(30); 
  
    // OGM_Class instance using unique_ptr
    std::unique_ptr<OGM_Class> ogm_obj = std::make_unique<OGM_Class>(&it_);
    ros::Rate loop_rate(10);

    s_static_vehicle vehicle; // struct instance
    s_static_vehicle perpendicular_vehicle;

    ogm_obj->AddStaticVehicles(vehicle,row_value,col_value);
    ogm_obj->AddPerpendicularVehicles(perpendicular_vehicle, row_value, col_value);
    ogm_obj->ColorCodeMap(image,row_value,col_value);
    
    while (ros::ok())
    {
        ogm_obj->PublishMsg(image);
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}