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

    std::vector<point> input;
    std::vector<angles> output;

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


    std_msgs::Float32MultiArray IK_order;

    float margin = 0.01;
    int state = 0;

    float xmax=-1000, ymax=-1000, zmax=-1000;
    float xmin=1000, ymin=1000, zmin=1000;

    simxStartSimulation(clientID, simx_opmode_oneshot);
    
    // create train data
    for(int i=0; i<500000; i++)
    {
        state = 0;

        while(state!=2)
        {
            std::cout << "foot2leg: " << foot2leg.x << " " << foot2leg.y << " " << foot2leg.z << std::endl;
            std::cout << "morf.FL: " << morf.FL.th1 << " , " << morf.FL.th2 << " , " << morf.FL.th3 << std::endl;
            std::cout << "data count: " << i << std::endl;

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

                input.push_back(foot2leg);
                output.push_back(morf.FL); 
                
                if(foot2leg.x>xmax)
                    xmax=foot2leg.x;
                if(foot2leg.x<xmin)
                    xmin=foot2leg.x;
                if(foot2leg.y>ymax)
                    ymax=foot2leg.y;
                if(foot2leg.y<ymin)
                    ymin=foot2leg.y;
                if(foot2leg.z>zmax)
                    zmax=foot2leg.z;
                if(foot2leg.z<zmin)
                    zmin=foot2leg.z;
            }
            else if(state==2)
            {
                input.push_back(foot2leg);
                output.push_back(morf.FL); 

                if(foot2leg.x>xmax)
                    xmax=foot2leg.x;
                if(foot2leg.x<xmin)
                    xmin=foot2leg.x;
                if(foot2leg.y>ymax)
                    ymax=foot2leg.y;
                if(foot2leg.y<ymin)
                    ymin=foot2leg.y;
                if(foot2leg.z>zmax)
                    zmax=foot2leg.z;
                if(foot2leg.z<zmin)
                    zmin=foot2leg.z;
            }

            if(state == 0)
                state = 1;
            else if(state == 1 && morf.FL.th1<FL.th1+margin && morf.FL.th1>FL.th1-margin && 
                        morf.FL.th2<FL.th2+margin && morf.FL.th2>FL.th2-margin &&
                        morf.FL.th3<FL.th3+margin && morf.FL.th3>FL.th3-margin)
                state = 2;

            ros::spinOnce();
            loop_rate.sleep();
        }

    }

    simxStopSimulation(clientID, simx_opmode_oneshot);
    // sleep(1);

    simxFinish(clientID);


    /*********************************** SORT DATA ***********************************/

    std::ofstream auxFile;
    std::string aux_path = std::string("./babbling_data/aux/");

    int div=5;
    int count[div][div][div];
    std::fill(&(count[0][0][0]), &(count[div][div][div]), 0);

    std::cout << input.size() << std::endl;

    for(int i=0; i<input.size(); i++)
    {
        int posX=-1, posY=-1, posZ=-1;
        for(int j=0; j<div; j++)
        {
            if(input[i].x<=xmin+(xmax-xmin)/div*(j+1) && posX==-1)
                posX=j;
            else if(j==div-1 && posZ==-1)
                posX=j;

            if(input[i].y<=ymin+(ymax-ymin)/div*(j+1) && posY==-1)
                posY=j;
            else if(j==div-1 && posZ==-1)
                posY=j;

            if(input[i].z<=zmin+(zmax-zmin)/div*(j+1) && posZ==-1)
                posZ=j;
            else if(j==div-1 && posZ==-1)
                posZ=j;
        }
        
        std::string aux_filename = aux_path+std::to_string(posX)+std::to_string(posY)+std::to_string(posZ)+std::string(".data");
        auxFile.open(aux_filename,  std::ios_base::app | std::ios_base::in);

        auxFile << input[i].x << " " << input[i].y << " " << input[i].z << "\n";
        auxFile << output[i].th1 << " " << output[i].th2<< " " << output[i].th3 << "\n";
        count[posX][posY][posZ]++;

        auxFile.close();
    }

    std::ofstream dataFile;
    std::string filepath = std::string("./babbling_data/");

    for(int i=0; i<div; i++)
    {
        for(int j=0; j<div; j++)
        {
            for(int k=0; k<div; k++)
            {
                // std::cout << count[i][j][k] << std::endl;
                if(count[i][j][k]>0)
                {
                    std::ifstream auxFile2;
                    std::string aux_filename = aux_path+std::to_string(i)+std::to_string(j)+std::to_string(k)+std::string(".data");
                    auxFile2.open(aux_filename);

                    std::string filename = filepath+std::to_string(i)+std::to_string(j)+std::to_string(k)+std::string(".data");
                    dataFile.open(filename);

                    dataFile << count[i][j][k] << " 3 3\n";

                    std::string line_in, line_out;
                    while(std::getline(auxFile2, line_in) && std::getline(auxFile2, line_out))
                    {
                        dataFile << line_in << '\n';
                        dataFile << line_out << '\n';
                    }
                    
                    auxFile2.close();
                    dataFile.close();
                }
            }
        }
    }


    return 0;
}
