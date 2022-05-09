#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <sys/stat.h>
#include <chrono>

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"



#include "../include/coordinates.hpp"
#include "../include/controller_real.hpp"
using namespace coords;
using namespace controller;

/********************* USAGE *************************

./main TYPE N_TRIALS

TYPE_: 0 means equations, 1 means neural network
N_TRIALS: number of trials

******************************************************/


int main(int argc, char **argv)
{
    // target coords for stabilizing with 4 legs
    point posFL, posFR, stableML, stableMR, stableBL, stableBR;
    angles FL, FR, ML, MR, BL, BR;
    angles auxML, auxMR, auxBL, auxBR;
    
    posFL.x = 0;//-7.5776e-02;
    posFL.y = +1.3090e-01;
    posFL.z = -2.9912e-01;

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

    // calculate IK parameters
    FL.calcIK(posFL);
    ML.calcIK(stableML);
    BL.calcIK(stableBL);
    FR.calcIK(posFR);
    MR.calcIK(stableMR);
    BR.calcIK(stableBR);

    ros::init(argc, argv, "morf_hw");
    ros::NodeHandle n;

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("multi_joint_commands", 1000);

    ros::Rate loop_rate(10);
    std_msgs::Float32MultiArray IK_order;

    while(ros::ok())
    {
        ROS_INFO("ROS is running");

        IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                            21, ML.th1, 22, ML.th2, 23, ML.th3,
                            31, BL.th1, 32, BL.th2, 33, BL.th3,
                            41, FR.th1, 42, FR.th2, 43, FR.th3,
                            51, MR.th1, 52, MR.th2, 53, MR.th3,
                            61, BR.th1, 62, BR.th2, 63, BR.th3};

        controller_pub.publish(IK_order);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
