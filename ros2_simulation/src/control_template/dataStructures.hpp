#ifndef DATA_STRUCTURES_HPP
#define DATA_STRUCTURES_HPP
 
#include <vector>


struct Waypoint{
    double x;
    double y;
};

struct lidarInformation{
    double angleMin;
    double angleMax;
    double angleIncrement;

    double rangeMin;
    double rangeMax;
};

#endif