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
    ros::Subscriber controller_sub = n.subscribe("joint_positions", 1000, &robot::infoCallback, &morf);

    // target coords
    point target;
    
    target.x = -5.9516e-02;
    target.y = +1.9789e-01;
    target.z = -2.0904e-01;

    
    
    point targetFL = target.morf2FL(); //convert to front left leg frame

    ROS_INFO("%f, %f, %f", targetFL.x, targetFL.y, targetFL.z);


    angles FL;
    FL.calcIK(targetFL);
    ROS_INFO("%f, %f, %f", FL.th1, FL.th2, FL.th3);


    ros::Rate loop_rate(10);
    
    std_msgs::Float32MultiArray msg;
    
    float th2, th3;
    while (ros::ok())
    {
        th2=morf.FL.th2;
        th3=morf.FL.th3;
        //ROS_INFO("FL.th2 = %f, th2 = %f, FL.th3=%f", FL.th2, th2,FL.th3);
        msg.data.push_back(FL.th1);
        msg.data.push_back(FL.th2);
        msg.data.push_back(FL.th3);

        controller_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
