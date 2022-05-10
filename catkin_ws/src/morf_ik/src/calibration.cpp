#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"


#include "../include/coordinates.hpp"
#include "../include/controller.hpp"
using namespace coords;
using namespace controller;


int main(int argc, char **argv)
{
    int l=4, w=4;
    float size=1/30;

    std::vector<cv::Point3f> posXadrez;
    for(int i=0; i<w; i++)
    {
        for(int j=0; j<l; j++)
            posXadrez.push_back(cv::Point3f(i*size,j*size,0));
    }

    std::vector<std::vector<cv::Point3f>> good_points;
    std::vector<std::vector<cv::Point2f>> all_cornersL, all_cornersR;

    cv::Mat imgL, imgR;

    for(int i=0; i<5; i++)
    {
        imgL = cv::imread("/home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_IK/calib_imgs/L0"+std::to_string(i+1)+".jpg");
        imgR = cv::imread("/home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_IK/calib_imgs/R0"+std::to_string(i+1)+".jpg");

        std::vector<cv::Point2f> cornersL, cornersR;
        bool doneL = cv::findChessboardCorners(imgL, cv::Size(l,w), cornersL);
        bool doneR = cv::findChessboardCorners(imgR, cv::Size(l,w), cornersR);

        if(doneL && doneR)
        {
            // cv::drawChessboardCorners(imgL, cv::Size(l,w), cornersL, doneL);
            good_points.push_back(posXadrez);
            all_cornersL.push_back(cornersL);
            all_cornersR.push_back(cornersR);

            std::cout << i << std::endl;
        }

        // cv::imshow("imgL",imgL);
	    // cv::waitKey(30);

    }

        cv::Mat cameraMatrixL, cameraMatrixR, distCoeffsL, distCoeffsR, R, T, E, F;

        cv::stereoCalibrate(good_points, all_cornersL, all_cornersR, cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, cv::Size(imgL.rows,imgL.cols), R, T, E, F);

        std::cout << cameraMatrixL << std::endl;
        std::cout << F << std::endl;



    /*robot morf;
	images stereo;
    ros::init(argc, argv, "IK_controller");

    ros::NodeHandle n;

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("joints_target", 1000);
    ros::Subscriber jointPos_sub = n.subscribe("joint_positions", 1000, &robot::infoCallback, &morf);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber imageLeft_sub = it.subscribe("imageLeft", 1, &images::imageLeftCallback, &stereo);
	image_transport::Subscriber imageRight_sub = it.subscribe("imageRight", 1, &images::imageRightCallback, &stereo);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

		if(!stereo.imageL.empty() && !stereo.imageR.empty())
		{
            cv::imshow("viewL", stereo.imageL);
            cv::imshow("viewR", stereo.imageR);
            cv::waitKey(30);
            imwrite("calib_imgs/L05.jpg", stereo.imageL);
            imwrite("calib_imgs/R05.jpg", stereo.imageR);
        }
    }*/


    return 0;
}
