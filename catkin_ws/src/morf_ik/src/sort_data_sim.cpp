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
    std::ifstream allInputFile, allOutputFile;
    std::string allInputFile_name = std::string("./babbling_data/input.data");
    std::string allOutputFile_name = std::string("./babbling_data/output.data");

    float xmax=-1000, ymax=-1000, zmax=-1000;
    float xmin=1000, ymin=1000, zmin=1000;
    std::string str;

    std::vector<point> input;
    std::vector<angles> output;

    allInputFile.open(allInputFile_name);
    allOutputFile.open(allOutputFile_name);

    while(std::getline(allInputFile, str, ' '))
    {
        point aux_in;
        angles aux_out;


        aux_in.x = std::stof(str);

        std::getline(allInputFile, str, ' ');
        aux_in.y = std::stof(str);

        std::getline(allInputFile, str);
        aux_in.z = std::stof(str);

        input.push_back(aux_in);


        std::getline(allOutputFile, str, ' ');
        aux_out.th1 = std::stof(str);

        std::getline(allOutputFile, str, ' ');
        aux_out.th2 = std::stof(str);

        std::getline(allOutputFile, str);
        aux_out.th3 = std::stof(str);

        output.push_back(aux_out);


        if(aux_in.x>xmax)
            xmax=aux_in.x;
        if(aux_in.x<xmin)
            xmin=aux_in.x;
        if(aux_in.y>ymax)
            ymax=aux_in.y;
        if(aux_in.y<ymin)
            ymin=aux_in.y;
        if(aux_in.z>zmax)
            zmax=aux_in.z;
        if(aux_in.z<zmin)
            zmin=aux_in.z;
    }

    allInputFile.close();
    allOutputFile.close();

    std::cout << "x: " << xmin << " , " << xmax << std::endl;
    std::cout << "y: " << ymin << " , " << ymax << std::endl;
    std::cout << "z: " << zmin << " , " << zmax << std::endl;


    std::ofstream auxFile;
    std::string aux_path = std::string("./babbling_data/aux/");

    int div=3, divZ=div*2;
    int count[div][div][divZ];

    for(int i=0; i<div;i++)
    {
        for(int j=0; j<div; j++)
        {
            for(int k=0; k<divZ; k++)
            {
                count[i][j][k]=0;
            }
        }
    }

    std::cout << input.size() << std::endl;

    for(int i=0; i<input.size(); i++)
    {
        int posX=-1, posY=-1, posZ=-1;
        for(int j=0; j<div; j++)
        {
            if(input[i].x<=xmin+(xmax-xmin)/div*(j+1) && posX==-1)
                posX=j;
            else if(j==div-1 && posX==-1)
                posX=j;

            if(input[i].y<=ymin+(ymax-ymin)/div*(j+1) && posY==-1)
                posY=j;
            else if(j==div-1 && posY==-1)
                posY=j;
        }

        for(int j=0; j<divZ; j++)
        {
            if(input[i].z<=zmin+(zmax-zmin)/divZ*(j+1) && posZ==-1)
                posZ=j;
            else if(j==divZ-1 && posZ==-1)
                posZ=j;
        }
        
        std::string aux_filename = aux_path+std::to_string(posX)+std::to_string(posY)+std::to_string(posZ)+std::string(".data");
        auxFile.open(aux_filename,  std::ios_base::app | std::ios_base::in);

        auxFile << input[i].x << " " << input[i].y << " " << input[i].z << "\n";
        auxFile << output[i].th1 << " " << output[i].th2<< " " << output[i].th3 << "\n";
        count[posX][posY][posZ]++;

        auxFile.close();
    }


   

    // save sorted

    std::ofstream dataFile;
    std::string filepath = std::string("./babbling_data/")+std::to_string(div)+std::string("div/");
    mkdir(filepath.c_str(),0777);

    for(int i=0; i<div; i++)
    {
        for(int j=0; j<div; j++)
        {
            for(int k=0; k<divZ; k++)
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
