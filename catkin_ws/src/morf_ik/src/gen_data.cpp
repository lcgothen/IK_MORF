#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <random>
#include <fstream>
#include <vector>
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
    std::vector<point> input;
    std::vector<angles> output;

    std::random_device rand_dev;
    unsigned int seed = rand_dev();

    // -7.4826e-02 <= x <= 0.11527
    // +6.2794e-02 <= y <= +3.2290e-01
    // -3.8406e-01 <= z <= -1.6912e-01

    std::default_random_engine rand_gen(seed);
    std::uniform_real_distribution<float> x_interval(-7.4826e-02,0.11527);
    std::uniform_real_distribution<float> y_interval(+6.2794e-02,+3.2290e-01);
    std::uniform_real_distribution<float> z_interval(-3.8406e-01,-1.6912e-01);

    // create train data

    int count=0;

    for(int i=0; i<100000; i++)
    {
        point aux_in;
        aux_in.x = x_interval(rand_gen);
        aux_in.y = y_interval(rand_gen);
        aux_in.z = z_interval(rand_gen);

        point aux;
        angles aux_out;
        aux = aux_in.morf2FL();
        aux_out.calcIK(aux);

        if(!isnan(aux_out.th1) && !isnan(aux_out.th2) && !isnan(aux_out.th3))
        {
            count++;
            //std::cout << count << ": " << aux_out.th1 << "," << aux_out.th2 << "," << aux_out.th3 << std::endl;
            input.push_back(aux_in);
            // input[i].x = aux_in.x;
            // input[i].y = aux_in.y;
            // input[i].z = aux_in.z;

            output.push_back(aux_out);
            // output[i].th1 = aux_out.th1;
            // output[i].th2 = aux_out.th2;
            // output[i].th3 = aux_out.th3;
            //std::cout << count << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
        }
    }

    std::ofstream data;
    data.open("data/train.data");

    data << count << " 3 3\n";

    for(int i=0; i<count; i++)
    {
        data << input[i].x << " " << input[i].y << " " << input[i].z << "\n";
        data << output[i].th1 << " " << output[i].th2 << " " << output[i].th3 << "\n";
        std::cout << i << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
    }

    data.close();


    // create validation data

    count=0;

    for(int i=0; i<10000; i++)
    {
        point aux_in;
        aux_in.x = x_interval(rand_gen);
        aux_in.y = y_interval(rand_gen);
        aux_in.z = z_interval(rand_gen);

        point aux;
        angles aux_out;
        aux = aux_in.morf2FL();
        aux_out.calcIK(aux);

        if(!isnan(aux_out.th1) && !isnan(aux_out.th2) && !isnan(aux_out.th3))
        {
            count++;
            //std::cout << count << ": " << aux_out.th1 << "," << aux_out.th2 << "," << aux_out.th3 << std::endl;
            input.push_back(aux_in);
            // input[i].x = aux_in.x;
            // input[i].y = aux_in.y;
            // input[i].z = aux_in.z;

            output.push_back(aux_out);
            // output[i].th1 = aux_out.th1;
            // output[i].th2 = aux_out.th2;
            // output[i].th3 = aux_out.th3;
            //std::cout << count << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
        }
    }

    data.open("data/vali.data");

    data << count << " 3 3\n";

    for(int i=0; i<count; i++)
    {
        data << input[i].x << " " << input[i].y << " " << input[i].z << "\n";
        data << output[i].th1 << " " << output[i].th2 << " " << output[i].th3 << "\n";
        std::cout << i << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
    }

    data.close();

    return 0;
}
