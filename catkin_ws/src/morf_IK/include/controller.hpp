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
        float th1, th2, th3;
    
        void calcIK(point target); // calculate angles with IK equations
    };

    class robot
    {
        public:
        angles FL, ML, BL, FR, MR, BR;

        void infoCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    };

    class CPG
    {
        public:
        float outputH1 = 0.001;
        float outputH2 = 0.001;
        float oH1, oH2;
        angles FL, ML, BL, FR, MR, BR;
    
        void cyclic();
        void walk();
    };

    class images
    {
        public:
        cv::Mat imageL, imageR;

        void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg);
        void imageRightCallback(const sensor_msgs::ImageConstPtr& msg);
        void match();
    };
}