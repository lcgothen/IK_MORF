// TO RUN
// . ~/tese/catkin_ws/devel/setup.bash (no longer needed)
// cd home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_IK
// ./main

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"


#include "../include/coordinates.hpp"
#include "../include/controller.hpp"
using namespace coords;
using namespace controller;

/*void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{	
		// detect features	
		cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
		std::vector<cv::KeyPoint> keypoints;
		cv::Mat input = cv_bridge::toCvShare(msg, "bgr8")->image;
		detector->detect(input, keypoints);
		
		// extract descriptors
		cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
		cv::Mat descriptors;
		extractor->compute(input, keypoints, descriptors);

		// Add results to image and save
		cv::Mat output;
		cv::drawKeypoints(input, keypoints, output);
		cv::imshow("view", output);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}*/

int main(int argc, char **argv)
{
    robot morf;
	images stereo;
    ros::init(argc, argv, "IK_controller");

    ros::NodeHandle n;

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("joints_target", 1000);
    ros::Subscriber jointPos_sub = n.subscribe("joint_positions", 1000, &robot::infoCallback, &morf);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber imageLeft_sub = it.subscribe("imageLeft", 1, &images::imageLeftCallback, &stereo);// imageLeftCallback);
	image_transport::Subscriber imageRight_sub = it.subscribe("imageRight", 1, &images::imageRightCallback, &stereo);

    cv::namedWindow("viewL");
	cv::namedWindow("viewR");

    // target coords for stabilizing with 4 legs
    point posFL, posFR, stableML, stableMR, stableBL, stableBR;
    
    posFL.x = 0;//-7.5776e-02;
    posFL.y = +1.3090e-01;
    posFL.z = -2.9912e-01;

    stableML.x = -7.5776e-02;
    stableML.y = +1.7632e-01;
    stableML.z = -1.6912e-01;

    stableBL.x = -7.5776e-02;
    stableBL.y = 0.22261;
    stableBL.z = 0.085760;

    posFR.x = 0;//-7.5776e-02;
    posFR.y = -1.3090e-01;
    posFR.z = -2.9912e-01;

    stableMR.x = -7.5776e-02;
    stableMR.y = -1.7632e-01;
    stableMR.z = -1.6912e-01;

    stableBR.x = -7.5776e-02;
    stableBR.y = -0.17579;
    stableBR.z = 0.085760;

    
    posFL = posFL.morf2FL(); //convert to front left leg frame
    stableML = stableML.morf2ML(); //convert to middle left leg frame
    stableBL = stableBL.morf2BL(); //convert to back left leg frame
    posFR = posFR.morf2FR(); //convert to front right leg frame
    stableMR = stableMR.morf2MR(); //convert to middle right leg frame
    stableBR = stableBR.morf2BR(); //convert to back right leg frame

    // ROS_INFO("%f, %f, %f", targetFL.x, targetFL.y, targetFL.z);



    angles FL, FR, ML, MR, BL, BR;
    FL.calcIK(posFL);
    ML.calcIK(stableML);
    BL.calcIK(stableBL);
    FR.calcIK(posFR);
    MR.calcIK(stableMR);
    BR.calcIK(stableBR);
    ROS_INFO("%f, %f, %f", BL.th1, BL.th2, BL.th3);


    ros::Rate loop_rate(10);
    
    std_msgs::Float32MultiArray msg;

    while(ros::ok())
    {
        //ROS_INFO("target: %f, %f, %f\n actual: %f, %f, %f", ML.th1, ML.th2, ML.th3, morf.ML.th1, morf.ML.th2, morf.ML.th3);
        msg.data.clear();
        
        // left leg angles
        msg.data.push_back(FL.th1);
        msg.data.push_back(FL.th2);
        msg.data.push_back(FL.th3);
        msg.data.push_back(ML.th1);
        msg.data.push_back(ML.th2);
        msg.data.push_back(ML.th3);
        msg.data.push_back(BL.th1);
        msg.data.push_back(BL.th2);
        msg.data.push_back(BL.th3);


        // right leg angles
        msg.data.push_back(FR.th1);
        msg.data.push_back(FR.th2);
        msg.data.push_back(FR.th3);
        msg.data.push_back(MR.th1);
        msg.data.push_back(MR.th2);
        msg.data.push_back(MR.th3);
        msg.data.push_back(BR.th1);
        msg.data.push_back(BR.th2);
        msg.data.push_back(BR.th3);

        // left leg angles
        /*
        msg.data.push_back(cpg.FL.th1);
        msg.data.push_back(cpg.FL.th2);
        msg.data.push_back(cpg.FL.th3);
        msg.data.push_back(cpg.ML.th1);
        msg.data.push_back(cpg.ML.th2);
        msg.data.push_back(cpg.ML.th3);
        msg.data.push_back(cpg.BL.th1);
        msg.data.push_back(cpg.BL.th2);
        msg.data.push_back(cpg.BL.th3);

        // right leg angles
        msg.data.push_back(cpg.FR.th1);
        msg.data.push_back(cpg.FR.th2);
        msg.data.push_back(cpg.FR.th3);
        msg.data.push_back(cpg.MR.th1);
        msg.data.push_back(cpg.MR.th2);
        msg.data.push_back(cpg.MR.th3);
        msg.data.push_back(cpg.BR.th1);
        msg.data.push_back(cpg.BR.th2);
        msg.data.push_back(cpg.BR.th3);
        */

        //ROS_INFO("%f, %f, %f", msg.data[0], msg.data[1], msg.data[2]);

        controller_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

		if(!stereo.imageL.empty() && !stereo.imageR.empty())
			stereo.match();
    }


    return 0;
}
