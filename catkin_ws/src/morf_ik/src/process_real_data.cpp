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
    std::string realInputFile_name = std::string("./real_data/trial_008_import.data");

    std::ofstream postProcInFile;
    std::string postProcInFile_name = std::string("./real_data/postProc.data");

    std::string str, val;
    int footNo = 2, oldFootNo;

    std::vector<point> input;
    point aux_in, leg, foot;

    realInputFile.open(realInputFile_name);

    while(std::getline(realInputFile, str))
    {
        bool noGo = false, changed=false;
        std::stringstream ss;
        ss << str;

        int i=0;
        std::string time;

        while(std::getline(ss, val, ','))
        {
            if(!noGo)
            {
                if(i==1)
                    time = val;
                else if(i==footNo)
                {
                    if(val.empty() && !changed)
                    {
                        if(footNo<32)
                        {
                            oldFootNo = footNo;
                            footNo += 3;
                            changed = true;
                        }
                        else
                        {
                            noGo = true;
                        }
                    }
                    else if(val.empty() && changed)
                    {
                        // noGo = true;
                        // footNo -= 3;
                        if(footNo<32)
                        {
                            footNo += 3;
                            changed = true;
                        }
                        else
                        {
                            noGo = true;
                            footNo = oldFootNo;
                        }
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
                    if(!val.empty())
                        leg.x = std::stof(val);
                }
                else if(i==36)
                {
                    if(!val.empty())
                        leg.y = std::stof(val);
                }
                else if(i==37)
                {
                    if(!val.empty())
                        leg.z = std::stof(val);
                }

                i++;
            }
        }

        if(!noGo)
        {
            std::cout << time << std::endl;
            aux_in.x = -(foot.z-leg.z);
            aux_in.y = -(foot.x-leg.x);
            aux_in.z = foot.y-leg.y;

            postProcInFile.open(postProcInFile_name, std::ios_base::app | std::ios_base::in);
            postProcInFile << time << "," << aux_in.x/1000.0 << "," << aux_in.y/1000.0 << "," << aux_in.z/1000.0 << "\n";
            postProcInFile.close();
        }
    }

    realInputFile.close();

    std::ifstream inFile, outFile;
    std::ofstream matchedInFile, matchedOutFile;

    std::string inFile_name = std::string("./real_data/postProc.data");
    std::string outFile_name = std::string("./real_data/output_trial008.data");
    std::string matchedInFile_name = std::string("./real_data/input.data");
    std::string matchedOutFile_name = std::string("./real_data/output.data");

    point in;
    angles out;

    std::string strIn, strOut;
    float tIn, tOut;
    float prev_tIn, prev_tOut;

    inFile.open(inFile_name);
    outFile.open(outFile_name);

    std::getline(inFile, strIn);
    std::getline(outFile, strOut);

    std::stringstream ssIn, ssOut;
    ssIn << strIn;
    ssOut << strOut;

    std::getline(ssIn, val, ',');
    prev_tIn = std::stof(val);

    std::getline(ssOut, val, ',');
    prev_tOut = std::stof(val);

    std::getline(ssIn, val, ',');
    in.x = std::stof(val);
    std::getline(ssIn, val, ',');
    in.y = std::stof(val);
    std::getline(ssIn, val, ',');
    in.z = std::stof(val);

    std::getline(ssOut, val, ',');
    out.th1 = std::stof(val);
    std::getline(ssOut, val, ',');
    out.th2 = std::stof(val);
    std::getline(ssOut, val, ',');
    out.th3 = std::stof(val);

    ssIn.str(std::string());
    ssIn.clear();
    ssOut.str(std::string());
    ssOut.clear();

    matchedInFile.open(matchedInFile_name, std::ios_base::app | std::ios_base::in);
    matchedInFile << in.x << " " << in.y << " " << in.z << "\n";
    matchedInFile.close();

    matchedOutFile.open(matchedOutFile_name, std::ios_base::app | std::ios_base::in);
    matchedOutFile << out.th1 << " " << out.th2 << " " << out.th3 << "\n";
    matchedOutFile.close();

    std::cout << std::fixed;
    std::cout << std::setprecision(3);

    while(std::getline(inFile, strIn) && std::getline(outFile, strOut))
    {
        ssIn << strIn;
        ssOut << strOut;

        std::getline(ssIn, val, ',');
        tIn = std::stof(val);

        std::getline(ssOut, val, ',');
        tOut = std::stof(val);

        // *********** CONDITIONS TO HOP OVER OUTPUTS IF THERE IS NO INPUT ***********

        if(tIn - prev_tIn > 0.007)
        {
            std::cout << tIn << " , " << tIn-prev_tIn << std::endl;
            float diff = tIn-prev_tIn;
            float numSkip = 1;

            while(0.005*numSkip+0.003<diff)
            {
                numSkip++;
            }

            for(int j=0; j<numSkip; j++)
            {
                std::getline(outFile, strOut);
            }

            std::getline(ssOut, val, ',');
            tOut = std::stof(val);
        }

        // ***************************************************************************

        std::getline(ssIn, val, ',');
        in.x = std::stof(val);
        std::getline(ssIn, val, ',');
        in.y = std::stof(val);
        std::getline(ssIn, val, ',');
        in.z = std::stof(val);

        std::getline(ssOut, val, ',');
        out.th1 = std::stof(val);
        std::getline(ssOut, val, ',');
        out.th2 = std::stof(val);
        std::getline(ssOut, val, ',');
        out.th3 = std::stof(val);

        matchedInFile.open(matchedInFile_name, std::ios_base::app | std::ios_base::in);
        matchedInFile << in.x << " " << in.y << " " << in.z << "\n";
        matchedInFile.close();

        matchedOutFile.open(matchedOutFile_name, std::ios_base::app | std::ios_base::in);
        matchedOutFile << out.th1 << " " << out.th2 << " " << out.th3 << "\n";
        matchedOutFile.close();

        prev_tIn = tIn;
        prev_tOut = tOut;

        ssIn.str(std::string());
        ssIn.clear();
        ssOut.str(std::string());
        ssOut.clear();
    }

    inFile.close();
    outFile.close();
    
    return 0;
}
