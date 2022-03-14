#pragma once
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <cmath>

namespace coords
{
    class point
    {
        public:
        float x, y, z;
        //float Ox, Oy, Oz;


        point world2FL(); // change coordinates from world frame to front left leg frame
        point morf2FL(); // change coordinates from morf frame to front left leg frame
        point morf2ML(); // change coordinates from morf frame to middle left leg frame
        point morf2BL(); // change coordinates from morf frame to back left leg frame
        point morf2FR(); // change coordinates from morf frame to front right leg frame
        point morf2MR(); // change coordinates from morf frame to middle right leg frame
        point morf2BR(); // change coordinates from morf frame to back right leg frame
    };

    point subtract(point a, point b);
    point sum(point a, point b);
    point mult_num(point a, float num);
    point div_num(point a, float num);
}