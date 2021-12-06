/*
    This program it to create Occupancy grid map in ROS system.

    As per the whole model/task concerns, 
    Sensor published data is been fed to 'sensor measurement model' to get 
    'sensor measurement values'. These values inturn fed to generate 
    "occupancy grid map" using 'inverse measurement model'. 

    I appoligize, its unfortunate that I couldn't converte series of complex mathematical equations 
    of 'sensor measurement model' to code. I will continue to figure this out.

    But at the meanwhile, I have taken a set of pre-stored 'sensor measurement values' 
    along with 'robot pose' values to generate 'occupancy grid map' using 'inverse measurement model'.
    Values of OGM is been stored in 2d vector, later map is been created using opencv::mat lib.
    At the end, cv::mat image is been converted to sensor_msgs/image to be published 
    on ROS topic "ogm/ogm_image".
*/

#include "ogm/ogm.h"

/* Inverse Sensor/Measurement Model decides on cell's state
    occupied 
    free 
    unknown 
*/
double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[])
{

    // Defining Sensor Characteristics
    double Zk, thetaK, sensorTheta;
    double minDelta = -1;
    double alpha = 200, beta = 20;

    //******************Compute r and phi**********************//
    double r = sqrt(pow(xi - x, 2) + pow(yi - y, 2));
    double phi = atan2(yi - y, xi - x) - theta;

    //Scaling Measurement to [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90]
    for (int i = 0; i < 8; i++) 
    {
        if (i == 0) {
            sensorTheta = -90 * (M_PI / 180);
        }
        else if (i == 1) {
            sensorTheta = -37.5 * (M_PI / 180);
        }
        else if (i == 6) {
            sensorTheta = 37.5 * (M_PI / 180);
        }
        else if (i == 7) {
            sensorTheta = 90 * (M_PI / 180);
        }
        else {
            sensorTheta = (-37.5 + (i - 1) * 15) * (M_PI / 180);
        }

        if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
            Zk = sensorData[i];
            thetaK = sensorTheta;
            minDelta = fabs(phi - sensorTheta);
        }
    }

    //******************Evaluate the three cases**********************//
    if (r > min((double)Zmax, Zk + alpha / 2) || fabs(phi - thetaK) > beta / 2 || Zk > Zmax || Zk < Zmin) 
    {
        return l0;
    }
    else if (Zk < Zmax && fabs(r - Zk) < alpha / 2) 
    {
        return locc;
    }
    else if (r <= Zk) 
    {
        return lfree;
    }

}

//Generating Occupancy Grid Mapping Algorithm using 'Inverse Sensor/Measurement Model'
void occupancyGridMapping(double Robotx, double Roboty, double Robottheta, double sensorData[])
{
    // OGM dimension mentioned at "ogm.h"
    for (int x = 0; x < mapWidth / gridWidth; x++) 
    {
    for (int y = 0; y < mapHeight / gridHeight; y++) 
    {
        double xi = x * gridWidth + gridWidth / 2 - robotXOffset;
        double yi = -(y * gridHeight + gridHeight / 2) + robotYOffset;
        if (sqrt(pow(xi - Robotx, 2) + pow(yi - Roboty, 2)) <= Zmax) 
        {
        l[x][y] = l[x][y] + inverseSensorModel(Robotx, Roboty, Robottheta, xi, yi, sensorData) - l0;
        }
    }
    }
    
}

// visualization of generated OGM using opencv
void visualization(cv::Mat& image, const double& row_value, const double& col_value)
{
    
    for (double x = 0; x < row_value; x++)
    {
        for (double y = 0; y < col_value; y++)
        {
            // fetch pixel color 
            cv::Vec3b color = image.at<cv::Vec3b>(Point(y,x));

            if( l[x][y] < 0)        // free cells as green pixels
            {
                color[0] = 50;
                color[1] = 255;     
                color[2] = 50;
            }
            else if (l[x][y] > 0)   // obstacel cells as red pixels
            {
                color[0] = 50;
                color[1] = 50;
                color[2] = 255;     
            }
            else                    // unknown cells as grey pixels
            {
                color[0] = 160;     // Blue channel 
                color[1] = 160;     // Green channel
                color[2] = 160;     // Red channel
            }

            // set pixel value back to the image
            image.at<cv::Vec3b>(Point(y,x)) = color;
        }
        
    }

}

// subscribed to "/camera/rgb/image_raw" topic 
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    /* Here comes the 'Sensor measurement model'
        input: Sensors published data
        output: Sensor measurement values, 
                which inturn feeds as input for
                generation of occupancy map */

    
    
}

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "ogm");
    ros::NodeHandle nh;

    // image publisher
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
    image_transport::Publisher pub = it.advertise("OGM/ogm_image", 1);
    ros::Rate loop_rate(10);

    // defining variables 
    double timeStamp;
    double measurementData[8];
    double robotX, robotY, robotTheta;

    // Fetch the pre-stored measurement sensor model and robot's pose files 
    // Change system dependent file paths  
    FILE* posesFile = fopen("/home/hp/ogm_generation/src/ogm/Data/poses.txt", "r");
    FILE* measurementFile = fopen("/home/hp/ogm_generation/src/ogm/Data/measurement.txt", "r");

    ROS_INFO("Successfully fetched measurement and pose files ...");

    // Scanning the files and retrieving measurement and poses at each timestamp
    while (fscanf(posesFile, "%lf %lf %lf %lf", &timeStamp, &robotX, &robotY, &robotTheta) != EOF) {
        fscanf(measurementFile, "%lf", &timeStamp);
        for (int i = 0; i < 8; i++) {
            fscanf(measurementFile, "%lf", &measurementData[i]);
        }
        occupancyGridMapping(robotX, robotY, (robotTheta / 10) * (M_PI / 180), measurementData);
    }
    ROS_INFO("Occupnacy map is been generated for per-stored measurement values!!! \n Please refer source file 'ogm.cpp' for further details.");

    // assigning row and col size for easier identification
    auto row_value = mapWidth/gridWidth;
    auto col_value = mapHeight/gridHeight;

    // creating an image of ogm's row and col values
    cv::Mat image(row_value, col_value, CV_8UC3);
    cv::waitKey(30);
    // Visualize the map, with color code
    visualization(image,row_value,col_value);
    // converting cv::mat to sensor_msgs::image to publish on ROS
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
   
    while (ros::ok())
    {
        pub.publish(msg); // publishing occupancy grid map generated
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}