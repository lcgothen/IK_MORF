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
    
        void get_angles(point target); // calculate angles with IK equations
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