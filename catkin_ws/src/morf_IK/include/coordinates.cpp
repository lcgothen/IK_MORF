#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <cmath>

#include "coordinates.hpp"
using namespace coords;

 
point point::world2FL() // change coordinates from world frame to front left leg frame
{
    point aux;

    // find a way to get the values automatically
    aux.x = -y+0.26857;
    aux.y = x-0.10079;
    aux.z = z-0.043091;

    return aux;
}

point point::morf2FL() // change coordinates from morf frame to front left leg frame
{
    point aux, aux0, aux1;

    aux0.x=z+1.5904e-01;
    aux0.y=x+1.0487e-02;
    aux0.z=y-6.2794e-02;

    aux1.x=-aux0.x+7.3954e-05;
    aux1.y=-aux0.y-2.1711e-03;
    aux1.z=aux0.z-2.3107e-02;

    aux.x=aux1.x+6.9797e-05;
    aux.y=aux1.z-1.1983e-02;
    aux.z=-aux1.y-1.2019e-03;

    return aux;
}

point point::morf2ML() // change coordinates from morf frame to middle left leg frame
{
    point aux, aux0, aux1;

    aux0.x=z+1.5041e-02;
    aux0.y=x+1.0487e-02;
    aux0.z=y-6.2780e-02;

    aux1.x=-aux0.x+7.3954e-05;
    aux1.y=-aux0.y-2.1711e-03;
    aux1.z=aux0.z-2.3107e-02;

    aux.x=aux1.x+6.9797e-05;
    aux.y=aux1.z-1.1983e-02;
    aux.z=-aux1.y-1.2019e-03;

    return aux;
}

point point::morf2BL() // change coordinates from morf frame to back left leg frame
{
    point aux, aux0, aux1;

    aux0.x=z-1.2896e-01;
    aux0.y=x+1.0487e-02;
    aux0.z=y-6.2775e-02;

    aux1.x=-aux0.x+7.3954e-05;
    aux1.y=-aux0.y-2.1711e-03;
    aux1.z=aux0.z-2.3107e-02;

    aux.x=aux1.x+6.9797e-05;
    aux.y=aux1.z-1.1983e-02;
    aux.z=-aux1.y-1.2019e-03;

    return aux;
}

point point::morf2FR() // change coordinates from morf frame to front right leg frame
{
    point aux, aux0, aux1;

    aux0.x=-z-1.5942e-01;
    aux0.y=x+1.0488e-02;
    aux0.z=-y-6.2777e-02;

    aux1.x=-aux0.x+7.3954e-05;
    aux1.y=-aux0.y-2.1711e-03;
    aux1.z=aux0.z-2.3107e-02;

    aux.x=aux1.x+6.9648e-05;
    aux.y=aux1.z-1.1983e-02;
    aux.z=-aux1.y-1.2020e-03;

    return aux;
}

point point::morf2MR() // change coordinates from morf frame to middle right leg frame
{
    point aux, aux0, aux1;

    aux0.x=-z-1.4819e-02;
    aux0.y=x+1.0488e-02;
    aux0.z=-y-6.2778e-02;

    aux1.x=-aux0.x+7.3954e-05;
    aux1.y=-aux0.y-2.1711e-03;
    aux1.z=aux0.z-2.3107e-02;

    aux.x=aux1.x+6.9648e-05;
    aux.y=aux1.z-1.1983e-02;
    aux.z=-aux1.y-1.2020e-03;

    return aux;
}

point point::morf2BR() // change coordinates from morf frame to back right leg frame
{
    point aux, aux0, aux1;

    aux0.x=-z+1.2918e-01;
    aux0.y=x+1.0488e-02;
    aux0.z=-y-6.2779e-02;

    aux1.x=-aux0.x+7.3954e-05;
    aux1.y=-aux0.y-2.1711e-03;
    aux1.z=aux0.z-2.3107e-02;

    aux.x=aux1.x+6.9648e-05;
    aux.y=aux1.z-1.1983e-02;
    aux.z=-aux1.y-1.2020e-03;

    return aux;
}