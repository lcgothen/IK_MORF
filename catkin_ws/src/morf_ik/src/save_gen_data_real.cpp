#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <ctime>
#include <chrono>
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
    /*********************************** SAVE DATA ***********************************/

    robot morf;


    std::ofstream allOutputFile;
    std::string allOutputFile_name = std::string("./real_data/output.data");

    ros::init(argc, argv, "save_data");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Subscriber jointPos_sub = n.subscribe("/morf_hw/joint_positions", 1000, &robot::jointPosCallback, &morf);


    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::milliseconds milliseconds;
    Clock::time_point init = Clock::now();
    std::chrono::milliseconds current = std::chrono::milliseconds(0); 

    bool loop_control=true;

    while(loop_control) 
    {
        if(std::cin.get() == 'q')
            loop_control=false;

        current = std::chrono::duration_cast<milliseconds>(Clock::now() - init);

        if(current % std::chrono::milliseconds(5) == std::chrono::milliseconds(0))
        {
            allOutputFile.open(allOutputFile_name,  std::ios_base::app | std::ios_base::in);
            allOutputFile << morf.FL.th1 << "," << morf.FL.th2<< "," << morf.FL.th3 << "\n";
            allOutputFile.close();
            std::cout << morf.FL.th1 << "," << morf.FL.th2<< "," << morf.FL.th3 << "\n";
        }
        std::chrono::milliseconds aux = current % std::chrono::milliseconds(5);
        std::cout << current.count() << "\t" << aux.count() << "\n";

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
