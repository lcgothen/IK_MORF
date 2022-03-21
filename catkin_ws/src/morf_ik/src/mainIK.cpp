// TO RUN
// . ~/tese/catkin_ws/devel/setup.bash (no longer needed)
// cd home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_IK
// ./main

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cmath>

#include "../include/coordinates.hpp"
#include "../include/controller.hpp"
using namespace coords;
using namespace controller;


int main(int argc, char **argv)
{
    robot morf;
    ros::init(argc, argv, "IK_controller");

    ros::NodeHandle n;

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("joints_target", 1000);
    ros::Subscriber controller_sub = n.subscribe("joint_positions", 1000, &robot::jointPosCallback, &morf);

    // target coords
    point target;
    
    /*target.x = -5.9516e-02;
    target.y = +1.9789e-01;
    target.z = -2.0904e-01;*/

    target.x = +3.7954e-02;
    target.y = +6.3691e-02;
    target.z = -3.4480e-01;

    
    
    point targetFL = target.morf2FL(); //convert to front left leg frame

    ROS_INFO("%f, %f, %f", targetFL.x, targetFL.y, targetFL.z);


    // target coords for stabilizing with 4 legs
    point posFL, posFR, stableML, stableMR, stableBL, stableBR;
    
    posFL.x = +3.9298e-02;
    posFL.y = +6.0723e-02;
    posFL.z = -3.0320e-01;

    stableML.x = -7.5776e-02;
    stableML.y = +1.7632e-01;
    stableML.z = -1.6912e-01;

    stableBL.x = -7.5776e-02;
    stableBL.y = 0.17579;
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

    
	// convert to leg frames
    posFL = posFL.morf2FL(); 
    stableML = stableML.morf2ML(); 
    stableBL = stableBL.morf2BL(); 
    posFR = posFR.morf2FR(); 
    stableMR = stableMR.morf2MR(); 
    stableBR = stableBR.morf2BR(); 


    angles FL, FR, ML, MR, BL, BR;
	// calculate IK parameters 
    FL.calcIK(posFL);
    ML.calcIK(stableML);
    BL.calcIK(stableBL);
    FR.calcIK(posFR);
    MR.calcIK(stableMR);
    BR.calcIK(stableBR);


    ros::Rate loop_rate(10);
    
    std_msgs::Float32MultiArray IK_order;
    
    float th2, th3;
    while (ros::ok())
    {
        th2=morf.FL.th2;
        th3=morf.FL.th3;
        //ROS_INFO("FL.th2 = %f, th2 = %f, FL.th3=%f", FL.th2, th2,FL.th3);

        IK_order.data.clear();


        // left leg angles
        IK_order.data.push_back(FL.th1);
        IK_order.data.push_back(FL.th2);
        IK_order.data.push_back(FL.th3);
        IK_order.data.push_back(ML.th1);
        IK_order.data.push_back(ML.th2);
        IK_order.data.push_back(ML.th3);
        IK_order.data.push_back(BL.th1);
        IK_order.data.push_back(BL.th2);
        IK_order.data.push_back(BL.th3);


        // right leg angles
        IK_order.data.push_back(FR.th1);
        IK_order.data.push_back(FR.th2);
        IK_order.data.push_back(FR.th3);
        IK_order.data.push_back(MR.th1);
        IK_order.data.push_back(MR.th2);
        IK_order.data.push_back(MR.th3);
        IK_order.data.push_back(BR.th1);
        IK_order.data.push_back(BR.th2);
        IK_order.data.push_back(BR.th3);

        controller_pub.publish(IK_order);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
