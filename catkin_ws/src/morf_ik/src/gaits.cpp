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
#include "../include/controller.hpp"
using namespace coords;
using namespace controller;

/********************* USAGE *************************

./gaits TYPE

TYPE_: 1 means straight, 2 means transverse and 3 means swivel

******************************************************/


int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cout << "./gaits TYPE" << std::endl;
        exit(EXIT_FAILURE);
    }

    int type = std::stoi(argv[1]);

    robot morf;
    CPG cpg;

    ros::init(argc, argv, "IK_controller");
    ros::NodeHandle n;

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("joints_target", 1000);


    ros::Rate loop_rate(10);
    std_msgs::Float32MultiArray IK_order;

    while(ros::ok())
    {
        cpg.simple_cyclic();
        cpg.gait(type);

        IK_order.data.clear();
        IK_order.data =    {cpg.FL.th1, cpg.FL.th2, cpg.FL.th3,
                            cpg.ML.th1, cpg.ML.th2, cpg.ML.th3,
                            cpg.BL.th1, cpg.BL.th2, cpg.BL.th3,
                            cpg.FR.th1, cpg.FR.th2, cpg.FR.th3,
                            cpg.MR.th1, cpg.MR.th2, cpg.MR.th3,
                            cpg.BR.th1, cpg.BR.th2, cpg.BR.th3};
        controller_pub.publish(IK_order);
        
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
