//#pragma once
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <cmath>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>

#include "coordinates.hpp"
using namespace coords;

namespace controller
{
    class angles
    {
        public:
        float th1=0, th2=2.024319, th3=0;
        int div, divZ, divZ_start;
        float x_length, y_length, z_length;
        float x_start, y_start, z_start;
        int reverse;
        struct fann ****ann;
        int cubeX=-1, cubeY=-1, cubeZ=-1;
    
        void calcIK(point target); // calculate angles with IK equations
        void calcNN(point target); // calculate angles with neural networks
        void initNN(std::string ann_path); // open neural networks
    };

    class robot
    {
        public:
        angles FL, ML, BL, FR, MR, BR;
        float sensFL;
        point foot2button;

        void jointPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void forceSensCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void foot2buttonPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    };

    class images
    {
        public:
        cv::Mat imageL, imageR, genImg;
        point target;
        int distZ=0; 
        bool nearZ=false; 
        float threshZ=0.9;

        void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg);
        void imageRightCallback(const sensor_msgs::ImageConstPtr& msg);
        void generalImgCallback(const sensor_msgs::ImageConstPtr& msg);
        void match();
    };

    class CPG
    {
        public:
        float outputH1 = 0.001;
        float outputH2 = 0.001;
        float oH1, oH2;
        angles FL, ML, BL, FR, MR, BR;
        //bool stabilize=false;
        float k=0.3;
    
        void cyclic(images *stereo);
        void walk(images stereo);
    };

    float jointLimiter(float jointValue, float jointMin, float jointMax);
}