#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <random>
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
    std::random_device rand_dev;
    unsigned int seed = rand_dev();

    robot morf;

    angles default_, FL;

    default_.th1=0;
    default_.th2=2.024319;
    default_.th3=0;

    std::default_random_engine rand_gen(seed);
    std::uniform_real_distribution<float> th1_interval(-110*M_PI/180, -20*M_PI/180);
    std::uniform_real_distribution<float> th2_interval(-5*M_PI/180, 130*M_PI/180);
    std::uniform_real_distribution<float> th3_interval(-180*M_PI/180, 0);

    simxInt clientID = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 2000, 5);

    ros::init(argc, argv, "IK_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("joints_target", 1000);
    ros::Subscriber jointPos_sub = n.subscribe("joint_positions", 1000, &robot::jointPosCallback, &morf);
    ros::Subscriber foot2button_sub = n.subscribe("foot2leg_position", 1000, &foot2legPosCallback);


    std::ofstream dataFile;
    std::string filename = std::string("./babbling_data/all.data");
    dataFile.open(filename, std::ios_base::app | std::ios_base::in);

    std_msgs::Float32MultiArray IK_order;

    float margin = 0.01;
    int state = 0;

    // create train data
    for(int i=0; i<500000; i++)
    {
        simxStartSimulation(clientID, simx_opmode_oneshot);
        state = 0;

        while(state!=3)
        {
            std::cout << "FL: " << FL.th1 << " , " << FL.th2 << " , " << FL.th3 << std::endl;
            std::cout << "morf.FL: " << morf.FL.th1 << " , " << morf.FL.th2 << " , " << morf.FL.th3 << std::endl;
            std:: cout << "state: " << state << i << std::endl;

            if(state==0)
            {
                FL.th1 = th1_interval(rand_gen);
                FL.th2 = th2_interval(rand_gen);
                FL.th3 = th3_interval(rand_gen);

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
            }
            else if(state==2)
            {
                dataFile << foot2leg.x << " " << foot2leg.y << " " << foot2leg.z << "\n";
                dataFile << morf.FL.th1 << " " << morf.FL.th2<< " " << morf.FL.th3 << "\n";
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

        simxStopSimulation(clientID, simx_opmode_oneshot);
        sleep(1);
    }

    simxFinish(clientID);

    return 0;
}
