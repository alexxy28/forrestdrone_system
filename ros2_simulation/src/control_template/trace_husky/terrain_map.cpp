#include "terrain_map.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <iostream>

TerrainMap::TerrainMap(
    double resolution,
    double size_x,
    double size_y)
    : resolution_(resolution),
      size_x_(size_x),
      size_y_(size_y),
      terrain_received_(false)
{
    width_ =
        static_cast<int>(size_x_ / resolution_);

    height_ =
        static_cast<int>(size_y_ / resolution_);

    const double nan =
        std::numeric_limits<double>::quiet_NaN();

    elevation_.assign(
        width_ * height_, nan);

    slope_.assign(
        width_ * height_, nan);
}

int TerrainMap::index(
    int gx,
    int gy) const
{
    return gy * width_ + gx;
}

void TerrainMap::updateFromPointCloud(
    const sensor_msgs::msg::PointCloud2 &cloud)
{
    const double nan =
        std::numeric_limits<double>::quiet_NaN();

    std::fill(
        elevation_.begin(),
        elevation_.end(),
        nan);

    sensor_msgs::PointCloud2ConstIterator<float>
        iter_x(cloud, "x");

    sensor_msgs::PointCloud2ConstIterator<float>
        iter_y(cloud, "y");

    sensor_msgs::PointCloud2ConstIterator<float>
        iter_z(cloud, "z");

    for (; iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z)
    {
        const double x = *iter_x;
        const double y = *iter_y;
        const double z = *iter_z;

        if (!std::isfinite(x) ||
            !std::isfinite(y) ||
            !std::isfinite(z))
        {
            continue;
        }

        int gx;
        int gy;

        if (!worldToGrid(
                x, y,
                gx, gy))
        {
            continue;
        }

        double &cell =
            elevation_[index(gx, gy)];

        /*
         * V1 ground/elevation estimate.
         *
         * Retain the highest observed point in each
         * 0.5 m cell.
         */
        if (std::isnan(cell) ||
            z > cell)
        {
            cell = z;
        }
    }

    /*
     * LiDAR does not necessarily hit every 0.5 m cell.
     * A completely unknown cell would otherwise become
     * non-traversable and fragment the A* search space.
     *
     * Fill only small holes using neighbouring measured
     * elevation values. Two passes allow short gaps to
     * become connected without inventing an entire map.
     */
    for (int pass = 0;
         pass < 2;
         ++pass)
    {
        std::vector<double>
            filled = elevation_;

        for (int gy = 1;
             gy < height_ - 1;
             ++gy)
        {
            for (int gx = 1;
                 gx < width_ - 1;
                 ++gx)
            {
                const int current =
                    index(gx, gy);

                if (!std::isnan(
                        elevation_[current]))
                {
                    continue;
                }

                double sum = 0.0;
                int count = 0;

                for (int dy = -1;
                     dy <= 1;
                     ++dy)
                {
                    for (int dx = -1;
                         dx <= 1;
                         ++dx)
                    {
                        if (dx == 0 &&
                            dy == 0)
                        {
                            continue;
                        }

                        const double value =
                            elevation_[
                                index(
                                    gx + dx,
                                    gy + dy)];

                        if (std::isfinite(value))
                        {
                            sum += value;
                            ++count;
                        }
                    }
                }

                /*
                 * Require several surrounding observations.
                 * This fills local sampling holes rather
                 * than declaring large unseen areas safe.
                 */
                if (count >= 3)
                {
                    filled[current] =
                        sum /
                        static_cast<double>(count);
                }
            }
        }

        elevation_.swap(filled);
    }

    calculateSlopes();
    terrain_received_ = true;
}

void TerrainMap::calculateSlopes()
{
    const double nan =
        std::numeric_limits<double>::quiet_NaN();

    std::fill(
        slope_.begin(),
        slope_.end(),
        nan);

    const int neighbours[4][2] = {
        {1, 0},
        {-1, 0},
        {0, 1},
        {0, -1}
    };

    for (int gy = 1;
         gy < height_ - 1;
         ++gy)
    {
        for (int gx = 1;
             gx < width_ - 1;
             ++gx)
        {
            const double centre =
                elevation_[index(gx, gy)];

            if (!std::isfinite(centre))
            {
                continue;
            }

            double max_slope = 0.0;
            int neighbour_count = 0;

            for (const auto &n : neighbours)
            {
                const int nx =
                    gx + n[0];

                const int ny =
                    gy + n[1];

                const double neighbour =
                    elevation_[index(nx, ny)];

                if (!std::isfinite(neighbour))
                {
                    continue;
                }

                const double dz =
                    std::abs(
                        neighbour -
                        centre);

                const double angle =
                    std::atan2(
                        dz,
                        resolution_) *
                    180.0 / M_PI;

                max_slope =
                    std::max(
                        max_slope,
                        angle);

                ++neighbour_count;
            }

            /*
             * Require at least two neighbouring elevation
             * observations before accepting a slope value.
             */
            if (neighbour_count >= 2)
            {
                slope_[index(gx, gy)] =
                    max_slope;
            }
        }
    }
}

bool TerrainMap::worldToGrid(
    double x,
    double y,
    int &gx,
    int &gy) const
{
    gx =
        static_cast<int>(
            std::floor(
                (x + size_x_ / 2.0) /
                resolution_));

    gy =
        static_cast<int>(
            std::floor(
                (y + size_y_ / 2.0) /
                resolution_));

    return
        gx >= 0 &&
        gx < width_ &&
        gy >= 0 &&
        gy < height_;
}

std::pair<double, double>
TerrainMap::gridToWorld(
    int gx,
    int gy) const
{
    const double x =
        -size_x_ / 2.0 +
        (gx + 0.5) *
            resolution_;

    const double y =
        -size_y_ / 2.0 +
        (gy + 0.5) *
            resolution_;

    return {x, y};
}

bool TerrainMap::isTraversable(
    int gx,
    int gy) const
{
    if (gx < 0 ||
        gx >= width_ ||
        gy < 0 ||
        gy >= height_)
    {
        return false;
    }

    const double value =
        slope_[index(gx, gy)];

    if (!std::isfinite(value))
    {
        return false;
    }

    return value <= MAX_SLOPE_DEG;
}

double TerrainMap::getSlope(
    int gx,
    int gy) const
{
    if (gx < 0 ||
        gx >= width_ ||
        gy < 0 ||
        gy >= height_)
    {
        return MAX_SLOPE_DEG + 1.0;
    }

    return slope_[index(gx, gy)];
}

bool TerrainMap::hasTerrain() const
{
    return terrain_received_;
}

int TerrainMap::getWidth() const
{
    return width_;
}

int TerrainMap::getHeight() const
{
    return height_;
}


void TerrainMap::printStatistics() const
{
    int elevation_known = 0;
    int slope_known = 0;
    int traversable = 0;
    int too_steep = 0;
    int unknown_slope = 0;

    for (int gy = 0; gy < height_; ++gy)
    {
        for (int gx = 0; gx < width_; ++gx)
        {
            const int i = index(gx, gy);

            if (std::isfinite(elevation_[i]))
                ++elevation_known;

            if (!std::isfinite(slope_[i]))
            {
                ++unknown_slope;
                continue;
            }

            ++slope_known;

            if (slope_[i] <= MAX_SLOPE_DEG)
                ++traversable;
            else
                ++too_steep;
        }
    }

    std::cout
        << "\n===== TRACE TERRAIN STATISTICS =====\n"
        << "Grid cells:       " << width_ * height_ << "\n"
        << "Elevation known:  " << elevation_known << "\n"
        << "Slope known:      " << slope_known << "\n"
        << "Traversable:      " << traversable << "\n"
        << "Too steep (>25):  " << too_steep << "\n"
        << "Unknown slope:    " << unknown_slope << "\n"
        << "====================================\n"
        << std::endl;
}
