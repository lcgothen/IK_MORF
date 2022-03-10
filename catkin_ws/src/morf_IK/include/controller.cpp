#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <cmath>
#include <random>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#include "coordinates.hpp"
using namespace coords;

#include "controller.hpp"
using namespace controller;

void robot::infoCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    FL.th1 = msg->data[0];
    FL.th2 = msg->data[1];
    FL.th3 = msg->data[2];
    ML.th1 = msg->data[3];
    ML.th2 = msg->data[4];
    ML.th3 = msg->data[5];
    BL.th1 = msg->data[6];
    BL.th2 = msg->data[7];
    BL.th3 = msg->data[8];
    FR.th1 = msg->data[9];
    FR.th2 = msg->data[10];
    FR.th3 = msg->data[11];
    MR.th1 = msg->data[12];
    MR.th2 = msg->data[13];
    MR.th3 = msg->data[14];
    BR.th1 = msg->data[15];
    BR.th2 = msg->data[16];
    BR.th3 = msg->data[17];
}

void angles::calcIK(point target) // calculate angles with IK equations
{
    float xc1, yc1, zc1;
    float L0 = 0.017011, L1 = 0.039805, L2 = 0.070075, L3=0.11542;// L3 = 0.11243;
    float offset2 = 0.8, offset3 = -2.5;

    th1 = atan2(-target.x, target.y);

    xc1 = cos(th1)*target.x+sin(th1)*target.y;
    yc1 = -sin(th1)*target.x+cos(th1)*target.y-L1;
    zc1 = target.z+L0;

    // th2 = acos((pow(L3,2)-pow(yc1,2)-pow(zc1,2)-pow(L2,2))/(-2*L2*sqrt(pow(yc1,2)+pow(zc1,2))))+atan2(zc1,yc1)+offset2;
    // th3 = acos((pow(yc1,2)+pow(zc1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3))+offset3; 

    th3 = acos((pow(yc1,2)+pow(zc1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3));
    th2 = atan2(zc1,yc1)+atan2(L3*sin(th3),L2+L3*cos(th3))+offset2;
    th3 += offset3;
}

void CPG::cyclic()
{
    float W12 = 0.4; 
    float W21 = -0.4; 
    float W11 = 1.5; 
    float W22 = 1.5;
    float BiasH1 = 0.01;
    float BiasH2 = 0.01;

    float activityH1 = W11*outputH1 + W12*outputH2 + BiasH1;
    float activityH2 = W22*outputH2 + W21*outputH1 + BiasH2;

    outputH1 = tanh(activityH1);
    outputH2 = tanh(activityH2);

    oH1 = outputH1*0.3;
    oH2 = outputH2*0.3;
}

void CPG::walk()
{
    float offset2 = 1.5, offset3 = -2.25/4;
    // back right
    BR.th1 = oH1; 
    BR.th2 = oH2+offset2; 
    BR.th3 = offset3; 

    // back left
    BL.th1 = oH1;
    BL.th2 = -oH2+offset2;
    BL.th3 = offset3;

    // middle right
    MR.th1 = -oH1;
    MR.th2 = -oH2+offset2;
    MR.th3 = offset3;
 
    // middle left
    ML.th1 = -oH1;
    ML.th2 = oH2+offset2;
    ML.th3 = offset3;

    // front right
    FR.th1 = oH1;
    FR.th2 = oH2+offset2;
    FR.th3 = offset3;

    // front left
    FL.th1 = oH1;
    FL.th2 = -oH2+offset2;
    FL.th3 = offset3;
}