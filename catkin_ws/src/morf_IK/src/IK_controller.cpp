// PARA CORRER
// . ~/tese/catkin_ws/devel/setup.bash
// cd home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_IK
// ./IK_controller

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
    
    
    point targetFL = target.morf2FL(); //convert to front left leg frame

    ROS_INFO("%f, %f, %f", targetFL.x, targetFL.y, targetFL.z);


    angles FL;
    FL.get_angles(targetFL);
    ROS_INFO("%f, %f, %f", FL.th1, FL.th2, FL.th3);

    /* COMM TEST */

    ros::Rate loop_rate(10);
    
    std_msgs::Float32MultiArray msg;
    CPG cpg;
    while (ros::ok())
    {
        cpg.cyclic();
        cpg.walk();
        msg.data.clear();

        // msg.data.push_back(FL.th1);
        // msg.data.push_back(FL.th2);
        // msg.data.push_back(FL.th3);

        // left leg angles
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

        //ROS_INFO("%f, %f, %f", msg.data[0], msg.data[1], msg.data[2]);

        controller_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
