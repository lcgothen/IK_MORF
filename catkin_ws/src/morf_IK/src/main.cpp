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
    ros::init(argc, argv, "IK_controller");

    ros::NodeHandle n;

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("joints_target", 1000);

    // target coords
    point target;
    target.x = -5.9516e-02;
    target.y = +1.9789e-01;
    target.z = -2.0904e-01;

    point initialFL;
    
    initialFL.x = -5.9516e-02;
    initialFL.y = +1.9789e-01;
    initialFL.z = -2.0904e-01;
    
    
    point targetFL = target.morf2FL(); //convert to front left leg frame

    ROS_INFO("%f, %f, %f", targetFL.x, targetFL.y, targetFL.z);


    ros::Rate loop_rate(10);
    
    std_msgs::Float32MultiArray msg;
    angles FL;
    CPG cpg;
    DMP traj;
    traj.init(initialFL, 10);

    while (ros::ok())
    {
        traj.calc(initialFL, targetFL);
        FL.calcIK(traj.y);

        msg.data.push_back(FL.th1);
        msg.data.push_back(FL.th2);
        msg.data.push_back(FL.th3);

        controller_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
