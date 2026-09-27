#ifndef TRACE_TERRAIN_MAP_HPP
#define TRACE_TERRAIN_MAP_HPP

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <utility>

class TerrainMap
{
public:
    TerrainMap(double resolution = 0.5,
               double size_x = 20.0,
               double size_y = 20.0);

    void updateFromPointCloud(
        const sensor_msgs::msg::PointCloud2 &cloud);

    void printStatistics() const;

    bool hasTerrain() const;

    bool worldToGrid(double x, double y,
                     int &gx, int &gy) const;

    std::pair<double, double>
    gridToWorld(int gx, int gy) const;

    bool isTraversable(int gx, int gy) const;

    double getSlope(int gx, int gy) const;

    int getWidth() const;
    int getHeight() const;

private:
    void calculateSlopes();

    int index(int gx, int gy) const;

    double resolution_;
    double size_x_;
    double size_y_;

    int width_;
    int height_;

    std::vector<double> elevation_;
    std::vector<double> slope_;

    bool terrain_received_;

    const double SAFE_SLOPE_DEG = 10.0;
    const double MAX_SLOPE_DEG = 25.0;
};

#endif
