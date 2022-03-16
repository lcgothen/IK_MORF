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


int main(int argc, char **argv)
{
    robot morf;
	images stereo;
    ros::init(argc, argv, "IK_controller");

    ros::NodeHandle n;

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("joints_target", 1000);
    ros::Subscriber jointPos_sub = n.subscribe("joint_positions", 1000, &robot::infoCallback, &morf);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber imageLeft_sub = it.subscribe("imageLeft", 1, &images::imageLeftCallback, &stereo);
	image_transport::Subscriber imageRight_sub = it.subscribe("imageRight", 1, &images::imageRightCallback, &stereo);


    // target coords for stabilizing with 4 legs
    point posFL, posFR, stableML, stableMR, stableBL, stableBR;
    
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


    angles FL, FR, ML, MR, BL, BR;
	// calculate IK parameters 
    FL.calcIK(posFL);
    ML.calcIK(stableML);
    BL.calcIK(stableBL);
    FR.calcIK(posFR);
    MR.calcIK(stableMR);
    BR.calcIK(stableBR);
    //ROS_INFO("%f, %f, %f", BL.th1, BL.th2, BL.th3);


    ros::Rate loop_rate(10);
    
    std_msgs::Float32MultiArray IK_order;
    CPG cpg;
    bool stabilized=false;

    while(ros::ok())
    {
        //std::cout << stabilized << std::endl;
        if(!stereo.imageL.empty() && !stereo.imageR.empty() && !stabilized)
        {
            //stereo.match();
			stereo.blob();
        }

        //ROS_INFO("target: %f, %f, %f\n actual: %f, %f, %f", ML.th1, ML.th2, ML.th3, morf.ML.th1, morf.ML.th2, morf.ML.th3);
        IK_order.data.clear();
        
        if(cpg.stabilize && !stabilized)
        {
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

            if(morf.FL.th1<FL.th1+0.001 && morf.FL.th1>FL.th1-0.001 && 
               morf.FL.th2<FL.th2+0.001 && morf.FL.th2>FL.th2-0.001 &&
               morf.FL.th3<FL.th3+0.001 && morf.FL.th3>FL.th3-0.001 &&
               morf.ML.th1<ML.th1+0.001 && morf.ML.th1>ML.th1-0.001 && 
               morf.ML.th2<ML.th2+0.001 && morf.ML.th2>ML.th2-0.001 &&
               morf.ML.th3<ML.th3+0.001 && morf.ML.th3>ML.th3-0.001 &&
               morf.BL.th1<BL.th1+0.001 && morf.BL.th1>BL.th1-0.001 && 
               morf.BL.th2<BL.th2+0.001 && morf.BL.th2>BL.th2-0.001 &&
               morf.BL.th3<BL.th3+0.001 && morf.BL.th3>BL.th3-0.001 &&
               morf.FR.th1<FR.th1+0.001 && morf.FR.th1>FR.th1-0.001 && 
               morf.FR.th2<FR.th2+0.001 && morf.FR.th2>FR.th2-0.001 &&
               morf.FR.th3<FR.th3+0.001 && morf.FR.th3>FR.th3-0.001 &&
               morf.MR.th1<MR.th1+0.001 && morf.MR.th1>MR.th1-0.001 && 
               morf.MR.th2<MR.th2+0.001 && morf.MR.th2>MR.th2-0.001 &&
               morf.MR.th3<MR.th3+0.001 && morf.MR.th3>MR.th3-0.001 &&
               morf.BR.th1<BR.th1+0.001 && morf.BR.th1>BR.th1-0.001 && 
               morf.BR.th2<BR.th2+0.001 && morf.BR.th2>BR.th2-0.001 &&
               morf.BR.th3<BR.th3+0.001 && morf.BR.th3>BR.th3-0.001)
               {
                    stabilized=true;
               }    
        }
        else if(stabilized)
        {
            posFL = stereo.target_avg.camLeft2morf(); 
            posFL = posFL.morf2FL(); 
            FL.calcIK(posFL);
            //ROS_INFO("%f, %f, %f", FL.th1, FL.th2, FL.th3);

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

            //std::cout << stereo.target.x << " , " << stereo.target.y <<  " , " << stereo.target.z << std::endl;
        }
        else if(!stereo.imageL.empty() && !stereo.imageR.empty())
        {
            cpg.cyclic();
            cpg.walk(stereo);

            // left leg angles
            IK_order.data.push_back(cpg.FL.th1);
            IK_order.data.push_back(cpg.FL.th2);
            IK_order.data.push_back(cpg.FL.th3);
            IK_order.data.push_back(cpg.ML.th1);
            IK_order.data.push_back(cpg.ML.th2);
            IK_order.data.push_back(cpg.ML.th3);
            IK_order.data.push_back(cpg.BL.th1);
            IK_order.data.push_back(cpg.BL.th2);
            IK_order.data.push_back(cpg.BL.th3);

            // right leg angles
            IK_order.data.push_back(cpg.FR.th1);
            IK_order.data.push_back(cpg.FR.th2);
            IK_order.data.push_back(cpg.FR.th3);
            IK_order.data.push_back(cpg.MR.th1);
            IK_order.data.push_back(cpg.MR.th2);
            IK_order.data.push_back(cpg.MR.th3);
            IK_order.data.push_back(cpg.BR.th1);
            IK_order.data.push_back(cpg.BR.th2);
            IK_order.data.push_back(cpg.BR.th3);

            stabilized=false;
        }
        else
        {
            // left leg angles
            IK_order.data.push_back(0);
            IK_order.data.push_back(2.024319);
            IK_order.data.push_back(0);
            IK_order.data.push_back(0);
            IK_order.data.push_back(2.024319);
            IK_order.data.push_back(0);
            IK_order.data.push_back(0);
            IK_order.data.push_back(2.024319);
            IK_order.data.push_back(0);

            // right leg angles
            IK_order.data.push_back(0);
            IK_order.data.push_back(2.024319);
            IK_order.data.push_back(0);
            IK_order.data.push_back(0);
            IK_order.data.push_back(2.024319);
            IK_order.data.push_back(0);
            IK_order.data.push_back(0);
            IK_order.data.push_back(2.024319);
            IK_order.data.push_back(0);

            stabilized=false;
        }

        controller_pub.publish(IK_order);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
