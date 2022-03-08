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

    class DMP
    {
        public:
        const static int N=5;
        float x=1, t=0, dt; 
        point y, prev_y, prev_dy, dy, ddy;
        float psi[N], w[N], h[N], c[N];
        float az=25, bz=az/4, ax=az/3;

        void init(point y0, float Hz);
        void calc(point y0, point goal);
    };
}