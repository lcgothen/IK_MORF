#pragma once
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <cmath>

#include "coordinates.hpp"
using namespace coords;

namespace controller
{
    class angles
    {
        public:
        float th1, th2, th3;
    
        void calcIK(point target); // calculate angles with IK equations
    };

    class robot
    {
        public:
        angles FL, ML, BL, FR, MR, BR;

        void infoCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    };

    class CPG
    {
        public:
        float outputH1 = 0.001;
        float outputH2 = 0.001;
        float oH1, oH2;
        angles FL, ML, BL, FR, MR, BR;
    
        void cyclic();
        void walk();
    };
}