#ifndef DATA_STRUCTURES_HPP
#define DATA_STRUCTURES_HPP
 
#include <vector>

struct LidarData{
    std::vector<double> ranges;
    double angleMin;
    double angleMax;
    double angleIncriment;

    double rangeMin;
    double rangeMax;
};

#endif