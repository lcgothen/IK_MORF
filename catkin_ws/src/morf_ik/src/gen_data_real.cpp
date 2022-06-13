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


#include "../include/coordinates_real.hpp"
#include "../include/controller_real.hpp"
using namespace coords;
using namespace controller;



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


    ros::init(argc, argv, "gen_data");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("/morf_hw/multi_joint_command", 1000);
    ros::Subscriber jointPos_sub = n.subscribe("/morf_hw/joint_positions", 1000, &robot::jointPosCallback, &morf);


    std_msgs::Float32MultiArray IK_order;

    int state = 0;
    float num = 40;
    float margin = 0.05, step_th1 = 70/num*M_PI/180, step_th2 = 135/num*M_PI/180, step_th3 = 179/num*M_PI/180;

    // sleep(5); // wait 5 seconds to synchronize
    
    // create train data
    for(int i=0; i<num; i++)
    {
        for(int j=0; j<num; j++)
        {
            for(int k=0; k<num; k++)
            {
                state = 0;

                while(state!=2)
                {
                    // std::cout << "FL: " << FL.th1 << " , " << FL.th2 << " , " << FL.th3 << std::endl;
                    // std::cout << "morf.FL: " << morf.FL.th1 << " , " << morf.FL.th2 << " , " << morf.FL.th3 << std::endl;
                    // std::cout << "i, j, k: "  << i << " , " << j << " , " << k << std::endl;

                    // std::cout << "FL.th2: "  << FL.th2 << std::endl;
                    // std::cout << "FL.th3: "  << FL.th3 << std::endl;

                    // if(std::cin.get() == 'q')
                    //     exit(EXIT_FAILURE);

                    // std::cout << "state: "  << state << std::endl;

                    if(state==0)
                    {
                        IK_order.data.clear();

                        IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3};

                        controller_pub.publish(IK_order);
                    }
                    if(state==1)
                    {
                        controller_pub.publish(IK_order);
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

                // finish testing this
                if(FL.th2<=0.94 && FL.th3>=abs(step_th2)*1.95/0.98*j-3.1 && step_th3>0 && step_th2>0) // limit th3 so as to not hit the table 
                    break;
                else if(FL.th2<=0.94 && FL.th3>=abs(step_th2)*1.95/0.98*(num-j)-3.1 && step_th3>0 && step_th2<0) // limit th3 so as to not hit the table 
                    break;
                else if(FL.th2<=0.94 && FL.th3<=-3.11 && step_th3<0) // limit th3 so as to not hit the table
                    break;
                else if(k<num-1)
                    FL.th3 += step_th3;
            }

            if(j<num-1)
                FL.th2 += step_th2;

            step_th3 = -step_th3;
        }

        FL.th1 += step_th1;
        step_th2 = -step_th2;
    }

    return 0;
}
