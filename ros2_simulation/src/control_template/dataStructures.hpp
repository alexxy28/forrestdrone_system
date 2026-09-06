#ifndef DATA_STRUCTURES_HPP
#define DATA_STRUCTURES_HPP
 
#include <vector>

struct lidarData{
    std::vector<double> ranges;
    double angleMin;
    double angleMax;
    double angleIncrement;

    double rangeMin;
    double rangeMax;
};

#endif