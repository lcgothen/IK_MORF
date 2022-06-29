#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <random>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

extern "C" {
    #include "extApi.h"
}


#include "../include/coordinates.hpp"
#include "../include/controller.hpp"
using namespace coords;
using namespace controller;


point foot2leg;

void foot2legPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    foot2leg.x = msg->data[0];
    foot2leg.y = msg->data[1];
    foot2leg.z = msg->data[2];
}


int main(int argc, char **argv)
{
    /*********************************** CREATE DATA ***********************************/

    std::random_device rand_dev;
    unsigned int seed = rand_dev();

    robot morf;

    angles default_, FL;

    default_.th1=0;
    default_.th2=2.024319;
    default_.th3=0;

    FL.th1 = -110*M_PI/180;
    FL.th2 = -5*M_PI/180;
    FL.th3 = -179*M_PI/180;


    std::ofstream allInputFile, allOutputFile;
    std::string allInputFile_name = std::string("./sim_data/input.data");
    std::string allOutputFile_name = std::string("./sim_data/output.data");

    std::default_random_engine rand_gen(seed);
    std::uniform_real_distribution<float> th1_interval(-110*M_PI/180, -20*M_PI/180);
    std::uniform_real_distribution<float> th2_interval(-5*M_PI/180, 130*M_PI/180);
    std::uniform_real_distribution<float> th3_interval(-180*M_PI/180, 0);

    ros::init(argc, argv, "IK_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("joints_target", 1000);
    ros::Subscriber jointPos_sub = n.subscribe("joint_positions", 1000, &robot::jointPosCallback, &morf);
    ros::Subscriber foot2button_sub = n.subscribe("foot2leg_position", 1000, &foot2legPosCallback);


    std_msgs::Float32MultiArray IK_order;

    simxInt clientID = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 2000, 5);

    int state = 0;
    float num = 40;
    float margin = 0.01, step_th1 = 70/num*M_PI/180, step_th2 = 135/num*M_PI/180, step_th3 = 179/num*M_PI/180;
    
    // create train data
    for(int i=0; i<num; i++)
    {
        simxStartSimulation(clientID, simx_opmode_oneshot);
        sleep(1);
        for(int j=0; j<num; j++)
        {
            for(int k=0; k<num; k++)
            {
                state = 0;

                while(state!=2)
                {
                    std::cout << "foot2leg: " << foot2leg.x << " " << foot2leg.y << " " << foot2leg.z << std::endl;
                    std::cout << "morf.FL: " << morf.FL.th1 << " , " << morf.FL.th2 << " , " << morf.FL.th3 << std::endl;
                    std::cout << "i, j, k: "  << i << " , " << j << " , " << k << std::endl;
                    std::cout << "FL: " << FL.th1 << " , " << FL.th2 << " , " << FL.th3 << std::endl;

                    if(state==0)
                    {
                        IK_order.data.clear();

                        // left leg angles
                        IK_order.data.push_back(FL.th1);
                        IK_order.data.push_back(FL.th2);
                        IK_order.data.push_back(FL.th3);
                        IK_order.data.push_back(default_.th1);
                        IK_order.data.push_back(default_.th2);
                        IK_order.data.push_back(default_.th3);
                        IK_order.data.push_back(default_.th1);
                        IK_order.data.push_back(default_.th2);
                        IK_order.data.push_back(default_.th3);

                        // right leg angles
                        IK_order.data.push_back(default_.th1);
                        IK_order.data.push_back(default_.th2);
                        IK_order.data.push_back(default_.th3);
                        IK_order.data.push_back(default_.th1);
                        IK_order.data.push_back(default_.th2);
                        IK_order.data.push_back(default_.th3);
                        IK_order.data.push_back(default_.th1);
                        IK_order.data.push_back(default_.th2);
                        IK_order.data.push_back(default_.th3);

                        controller_pub.publish(IK_order);
                    }
                    if(state==1)
                    {
                        controller_pub.publish(IK_order);

                        allInputFile.open(allInputFile_name,  std::ios_base::app | std::ios_base::in);
                        allOutputFile.open(allOutputFile_name,  std::ios_base::app | std::ios_base::in);

                        allInputFile << foot2leg.x << " " << foot2leg.y << " " << foot2leg.z << "\n";
                        allOutputFile << morf.FL.th1 << " " << morf.FL.th2<< " " << morf.FL.th3 << "\n";

                        allInputFile.close();
                        allOutputFile.close();
                    }

                    if(state == 0)
                        state = 1;
                    else if(state == 1 && morf.FL.th1<FL.th1+margin && morf.FL.th1>FL.th1-margin && 
                                morf.FL.th2<FL.th2+margin && morf.FL.th2>FL.th2-margin &&
                                morf.FL.th3<FL.th3+margin && morf.FL.th3>FL.th3-margin)
                        state = 2;
                    else if(state == 2)
                        state = 3;

                    ros::spinOnce();
                    loop_rate.sleep();
                }

                if(k<num-1)
                    FL.th3 += step_th3;
            }

            if(j<num-1)
                FL.th2 += step_th2;

            step_th3 = -step_th3;
        }

        FL.th1 += step_th1;
        step_th2 = -step_th2;

        simxStopSimulation(clientID, simx_opmode_oneshot);
        sleep(1);
    }

    simxFinish(clientID);

    return 0;
}
