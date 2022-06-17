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

#include <sys/stat.h>



#include "../include/coordinates.hpp"
#include "../include/controller.hpp"
using namespace coords;
using namespace controller;


int main(int argc, char **argv)
{
    std::ifstream realInputFile;
    std::string realInputFile_name = std::string("./real_data/trial_008_import.csv");

    std::ofstream postProcInFile;
    std::string postProcInFile_name = std::string("./real_data/postProc.csv");

    std::string str, val;
    int footNo = 2;

    std::vector<point> input;
    point aux_in, leg, foot;

    realInputFile.open(realInputFile_name);

    std::getline(realInputFile, str);

    while(std::getline(realInputFile, str))
    {
        bool noGo = false, changed=false;
        std::stringstream ss;
        ss << str;

        int i=0;
        float time;

        while(std::getline(ss, val, ','))
        {
            if(!noGo)
            {
                std::cout << str << std::endl;
                if(i==1)
                    time = std::stof(val);
                else if(i==footNo)
                {
                    if(val.empty() && !changed)
                    {
                        footNo += 3;
                        changed = true;
                    }
                    else if(val.empty() && changed)
                    {
                        noGo = true;
                        footNo -= 3;
                    }
                    else
                    {
                        foot.x = std::stof(val);
                        changed = false;
                    }
                }
                else if(i==footNo+1)
                    foot.y = std::stof(val);
                else if(i==footNo+2)
                    foot.z = std::stof(val);
                else if(i==35)
                {
                    if(val.empty())
                        noGo = true;
                    else
                        leg.x = std::stof(val);
                }
                else if(i==36)
                    leg.y = std::stof(val);
                else if(i==37)
                    leg.z = std::stof(val);

                i++;
            }
        }

        if(!noGo)
        {
            aux_in.x = -(foot.z-leg.z);
            aux_in.y = -(foot.x-leg.x);
            aux_in.z = foot.y-leg.y;

            postProcInFile.open(postProcInFile_name, std::ios_base::app | std::ios_base::in);
            postProcInFile << time << "," << aux_in.x << "," << aux_in.y << "," << aux_in.z << "\n";
            postProcInFile.close();
        }
    }

    realInputFile.close();
    
    return 0;
}
