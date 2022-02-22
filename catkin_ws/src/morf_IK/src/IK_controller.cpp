// PARA CORRER
// . ~/tese/catkin_ws/devel/setup.bash

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cmath>

class PointCoord
{
    public:
    float x, y, z;
    float Ox, Oy, Oz;
 
    PointCoord world2TC0() // change coordinates from world frame to TC0 frame
    {
        PointCoord aux;

        // find a way to get the values automatically
        aux.x = -y+0.26857;
        aux.y = x-0.10079;
        aux.z = z-0.043091;

        return aux;
    }

    PointCoord morf2TC0() // change coordinates from world frame to TC0 frame
    {
        PointCoord aux;

        // find a way to get the values automatically
        aux.x = -z-0.15904;
        aux.y = y-0.097885;
        aux.z = x+0.009518;

        return aux;
    }
};

class IK_angles
{
    public:
    float th1, th2, th3;
 
    void get_angles(PointCoord target) // calculate angles with IK equations, from target in TC0 frame
    {
        float xc1, yc1, zc1;
        float L0 = 0.01701, L1 = 0.039805, L2 = 0.070044, L3 = 0.11242;
        float offset2 = 1.5, offset3 = -2.25;

        th1 = atan2(-target.x, target.y);

        xc1 = target.y-L1*cos(th1);
        yc1 = target.z-L0;
        zc1 = target.x;

        th2 = acos((pow(L3,2)-pow(xc1,2)-pow(yc1,2)-pow(L2,2))/(-2*L2*sqrt(pow(xc1,2)+pow(yc1,2))))-atan2(-yc1,xc1)+offset2;
        th3 = acos((pow(xc1,2)+pow(yc1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3))+offset3;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IK_controller");

    ros::NodeHandle n;

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("joints_target", 1000);

    // target coords
    PointCoord target;
    target.x = -0.067814;
    target.y = 0.18821;
    target.z = -0.18;
    PointCoord targetTC0 = target.morf2TC0(); //convert to TC0 frame

    ROS_INFO("%f, %f, %f", targetTC0.x, targetTC0.y, targetTC0.z);

    //float aTC0, aCF0, aFT0; // angles

    // IK equations
    //aTC0 = atan2(-targetTC0.x, targetTC0.y);

    IK_angles leg0;
    leg0.get_angles(targetTC0);
    ROS_INFO("%f, %f, %f", leg0.th1, leg0.th2, leg0.th3);

    /* COMM TEST */

    ros::Rate loop_rate(10);
    
    std_msgs::Float32MultiArray msg;
    float angle = 0;
    while (ros::ok())
    {
        msg.data.clear();
        //msg.data.push_back(aTC0);
        msg.data.push_back(leg0.th1);
        msg.data.push_back(leg0.th2);
        msg.data.push_back(leg0.th3);

        //ROS_INFO("%f, %f, %f", msg.data[0], msg.data[1], msg.data[2]);

        controller_pub.publish(msg);
        angle = angle+0.1;

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}