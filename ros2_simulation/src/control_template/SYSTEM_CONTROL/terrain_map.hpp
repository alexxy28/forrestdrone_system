#ifndef TRACE_TERRAIN_MAP_HPP
#define TRACE_TERRAIN_MAP_HPP

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

/// 2.5D terrain grid. Each cell keeps a running mean (height) and variance
/// (roughness) of every LiDAR return that fell in it. The grid has an odd cell
/// count so cell (half_x, half_y) is centred exactly on the map origin; the
/// rqt terrain plugin uses the same convention.
class TerrainMap
{
public:
    TerrainMap(double resolution = 0.5,
               double size_x = 40.0,
               double size_y = 40.0);

    /// Accumulate one return (map frame) into its cell's running statistics.
    void addPoint(double x, double y, double z);

    /// Replace all cell statistics with a cloud produced by toCloud() on an
    /// identically sized map, then rebuild().
    void updateFromPointCloud(
        const sensor_msgs::msg::PointCloud2 &cloud);

    /// One point per observed cell: x, y, z (mean), sigma, count.
    sensor_msgs::msg::PointCloud2 toCloud(
        const std::string &frame_id,
        const builtin_interfaces::msg::Time &stamp) const;

    /// Recompute elevation, sigma (with hole filling) and slope from the cells.
    void rebuild();

    void printStatistics() const;

    bool hasTerrain() const;

    bool worldToGrid(double x, double y,
                     int &gx, int &gy) const;

    std::pair<double, double>
    gridToWorld(int gx, int gy) const;

    bool isTraversable(int gx, int gy) const;

    double getSlope(int gx, int gy) const;
    double getSigma(int gx, int gy) const;

    int getWidth() const;
    int getHeight() const;

    static constexpr double MAX_SLOPE_DEG = 25.0;
    // Returns spread more than this in one cell = tree/canopy over ground.
    static constexpr double MAX_SIGMA_M = 0.5;

private:
    struct Cell
    {
        float mean = 0.0f;
        float var = 0.0f;
        uint32_t count = 0;
    };

    void calculateSlopes();

    bool inside(int gx, int gy) const;
    int index(int gx, int gy) const;

    double resolution_;
    int half_x_;
    int half_y_;
    int width_;
    int height_;

    std::vector<Cell> cells_;

    // Derived by rebuild(); NaN = unknown.
    std::vector<double> elevation_;
    std::vector<double> sigma_;
    std::vector<double> slope_;

    bool terrain_received_;

    // A cell needs this many returns before its mean/sigma are trusted.
    static constexpr uint32_t MIN_COUNT = 3;
    // ponytail: past this many returns old data fades exponentially, so moved
    // trees clear out; raise for a static world, lower to forget faster.
    static constexpr uint32_t COUNT_CAP = 50;
};

#endif
