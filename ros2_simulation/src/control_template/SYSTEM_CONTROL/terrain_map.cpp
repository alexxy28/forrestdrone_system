#include "terrain_map.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

namespace
{
const double NaN = std::numeric_limits<double>::quiet_NaN();
}

TerrainMap::TerrainMap(
    double resolution,
    double size_x,
    double size_y)
    : resolution_(std::max(resolution, 1e-6)),
      terrain_received_(false)
{
    half_x_ = static_cast<int>(std::ceil(size_x / (2.0 * resolution_)));
    half_y_ = static_cast<int>(std::ceil(size_y / (2.0 * resolution_)));
    width_ = 2 * half_x_ + 1;
    height_ = 2 * half_y_ + 1;

    cells_.assign(width_ * height_, Cell{});
    elevation_.assign(width_ * height_, NaN);
    sigma_.assign(width_ * height_, NaN);
    slope_.assign(width_ * height_, NaN);
}

bool TerrainMap::inside(int gx, int gy) const
{
    return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

int TerrainMap::index(int gx, int gy) const
{
    return gy * width_ + gx;
}

void TerrainMap::addPoint(double x, double y, double z)
{
    int gx;
    int gy;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !worldToGrid(x, y, gx, gy))
    {
        return;
    }

    Cell &cell = cells_[index(gx, gy)];
    ++cell.count;

    /*
     * Running mean/variance with weight a = 1/n. Below COUNT_CAP this is the
     * exact mean and population variance of every return; above it the
     * weight stays at 1/COUNT_CAP (an exponential moving average).
     */
    const double a = 1.0 / std::min(cell.count, COUNT_CAP);
    const double d = z - cell.mean;
    cell.mean += a * d;
    cell.var = (1.0 - a) * (cell.var + a * d * d);
}

void TerrainMap::updateFromPointCloud(
    const sensor_msgs::msg::PointCloud2 &cloud)
{
    std::fill(cells_.begin(), cells_.end(), Cell{});

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_sigma(cloud, "sigma");
    sensor_msgs::PointCloud2ConstIterator<float> iter_count(cloud, "count");

    for (; iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z, ++iter_sigma, ++iter_count)
    {
        int gx;
        int gy;

        if (!std::isfinite(*iter_z) ||
            !worldToGrid(*iter_x, *iter_y, gx, gy))
        {
            continue;
        }

        Cell &cell = cells_[index(gx, gy)];
        cell.mean = *iter_z;
        cell.var = *iter_sigma * *iter_sigma;
        cell.count = static_cast<uint32_t>(*iter_count);
    }

    rebuild();
}

sensor_msgs::msg::PointCloud2 TerrainMap::toCloud(
    const std::string &frame_id,
    const builtin_interfaces::msg::Time &stamp) const
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    using sensor_msgs::msg::PointField;
    modifier.setPointCloud2Fields(
        5,
        "x", 1, PointField::FLOAT32,
        "y", 1, PointField::FLOAT32,
        "z", 1, PointField::FLOAT32,
        "sigma", 1, PointField::FLOAT32,
        "count", 1, PointField::FLOAT32);

    modifier.resize(std::count_if(
        cells_.begin(), cells_.end(),
        [](const Cell &c) { return c.count > 0; }));

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_sigma(cloud, "sigma");
    sensor_msgs::PointCloud2Iterator<float> iter_count(cloud, "count");

    for (int gy = 0; gy < height_; ++gy)
    {
        for (int gx = 0; gx < width_; ++gx)
        {
            const Cell &cell = cells_[index(gx, gy)];

            if (cell.count == 0)
                continue;

            const auto [x, y] = gridToWorld(gx, gy);
            *iter_x = x;
            *iter_y = y;
            *iter_z = cell.mean;
            *iter_sigma = std::sqrt(cell.var);
            *iter_count = static_cast<float>(cell.count);

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_sigma;
            ++iter_count;
        }
    }

    return cloud;
}

void TerrainMap::rebuild()
{
    terrain_received_ = false;

    for (size_t i = 0; i < cells_.size(); ++i)
    {
        if (cells_[i].count >= MIN_COUNT)
        {
            elevation_[i] = cells_[i].mean;
            sigma_[i] = std::sqrt(cells_[i].var);
            terrain_received_ = true;
        }
        else
        {
            elevation_[i] = NaN;
            sigma_[i] = NaN;
        }
    }

    /*
     * LiDAR does not necessarily hit every 0.5 m cell.
     * A completely unknown cell would otherwise become
     * non-traversable and fragment the A* search space.
     *
     * Fill only small holes using neighbouring measured
     * elevation and sigma values. Two passes allow short gaps
     * to become connected without inventing an entire map.
     */
    for (int pass = 0; pass < 2; ++pass)
    {
        std::vector<double> filled_elevation = elevation_;
        std::vector<double> filled_sigma = sigma_;

        for (int gy = 1; gy < height_ - 1; ++gy)
        {
            for (int gx = 1; gx < width_ - 1; ++gx)
            {
                const int current = index(gx, gy);

                if (!std::isnan(elevation_[current]))
                    continue;

                double elevation_sum = 0.0;
                double sigma_sum = 0.0;
                int count = 0;

                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        const int n = index(gx + dx, gy + dy);

                        if ((dx == 0 && dy == 0) ||
                            !std::isfinite(elevation_[n]))
                        {
                            continue;
                        }

                        elevation_sum += elevation_[n];
                        sigma_sum += sigma_[n];
                        ++count;
                    }
                }

                /*
                 * Require several surrounding observations.
                 * This fills local sampling holes rather
                 * than declaring large unseen areas safe.
                 */
                if (count >= 3)
                {
                    filled_elevation[current] = elevation_sum / count;
                    filled_sigma[current] = sigma_sum / count;
                }
            }
        }

        elevation_.swap(filled_elevation);
        sigma_.swap(filled_sigma);
    }

    calculateSlopes();
}

void TerrainMap::calculateSlopes()
{
    std::fill(slope_.begin(), slope_.end(), NaN);

    const int neighbours[4][2] = {
        {1, 0},
        {-1, 0},
        {0, 1},
        {0, -1}
    };

    for (int gy = 1; gy < height_ - 1; ++gy)
    {
        for (int gx = 1; gx < width_ - 1; ++gx)
        {
            const double centre = elevation_[index(gx, gy)];

            if (!std::isfinite(centre))
                continue;

            double max_slope = 0.0;
            int neighbour_count = 0;

            for (const auto &n : neighbours)
            {
                const double neighbour =
                    elevation_[index(gx + n[0], gy + n[1])];

                if (!std::isfinite(neighbour))
                    continue;

                const double angle =
                    std::atan2(std::abs(neighbour - centre), resolution_) *
                    180.0 / M_PI;

                max_slope = std::max(max_slope, angle);
                ++neighbour_count;
            }

            /*
             * Require at least two neighbouring elevation
             * observations before accepting a slope value.
             */
            if (neighbour_count >= 2)
                slope_[index(gx, gy)] = max_slope;
        }
    }
}

bool TerrainMap::worldToGrid(
    double x,
    double y,
    int &gx,
    int &gy) const
{
    gx = static_cast<int>(std::floor(x / resolution_ + half_x_ + 0.5));
    gy = static_cast<int>(std::floor(y / resolution_ + half_y_ + 0.5));

    return inside(gx, gy);
}

std::pair<double, double>
TerrainMap::gridToWorld(
    int gx,
    int gy) const
{
    return {(gx - half_x_) * resolution_,
            (gy - half_y_) * resolution_};
}

bool TerrainMap::isTraversable(
    int gx,
    int gy) const
{
    if (!inside(gx, gy))
        return false;

    const double slope = slope_[index(gx, gy)];
    const double sigma = sigma_[index(gx, gy)];

    // NaN fails both comparisons, so unknown cells are not traversable.
    return slope <= MAX_SLOPE_DEG && sigma <= MAX_SIGMA_M;
}

double TerrainMap::getSlope(
    int gx,
    int gy) const
{
    if (!inside(gx, gy))
        return MAX_SLOPE_DEG + 1.0;

    return slope_[index(gx, gy)];
}

double TerrainMap::getSigma(
    int gx,
    int gy) const
{
    if (!inside(gx, gy))
        return MAX_SIGMA_M + 1.0;

    return sigma_[index(gx, gy)];
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
    int observed = 0;
    int filled = 0;
    int traversable = 0;
    int too_steep = 0;
    int too_rough = 0;
    int unknown = 0;
    uint64_t samples = 0;
    uint32_t max_samples = 0;
    double z_min = std::numeric_limits<double>::infinity();
    double z_max = -z_min;
    std::vector<double> sigmas;

    for (size_t i = 0; i < cells_.size(); ++i)
    {
        const bool known = std::isfinite(elevation_[i]);

        if (cells_[i].count >= MIN_COUNT)
        {
            ++observed;
            samples += cells_[i].count;
            max_samples = std::max(max_samples, cells_[i].count);
            sigmas.push_back(sigma_[i]);
        }
        else if (known)
        {
            ++filled;
        }

        if (known)
        {
            z_min = std::min(z_min, elevation_[i]);
            z_max = std::max(z_max, elevation_[i]);
        }

        // Steepness is checked first, so a cell that is both counts as steep.
        if (!known || !std::isfinite(slope_[i]))
            ++unknown;
        else if (slope_[i] > MAX_SLOPE_DEG)
            ++too_steep;
        else if (sigma_[i] > MAX_SIGMA_M)
            ++too_rough;
        else
            ++traversable;
    }

    double sigma_median = NaN;
    double sigma_max = NaN;

    if (!sigmas.empty())
    {
        auto mid = sigmas.begin() + sigmas.size() / 2;
        std::nth_element(sigmas.begin(), mid, sigmas.end());
        sigma_median = *mid;
        sigma_max = *std::max_element(sigmas.begin(), sigmas.end());
    }

    std::cout
        << std::fixed << std::setprecision(2)
        << "\n===== TERRAIN STATISTICS =====\n"
        << "Grid:             " << width_ << " x " << height_
        << " cells @ " << resolution_ << " m\n"
        << "Observed cells:   " << observed
        << " (>= " << MIN_COUNT << " returns)\n"
        << "Hole-filled:      " << filled << "\n"
        << "Returns per cell: mean "
        << (observed ? static_cast<double>(samples) / observed : 0.0)
        << ", max " << max_samples << "\n"
        << "Height range:     " << z_min << " .. " << z_max << " m\n"
        << "Sigma:            median " << sigma_median
        << ", max " << sigma_max << " m\n"
        << "Traversable:      " << traversable << "\n"
        << "Too steep (>" << MAX_SLOPE_DEG << " deg): " << too_steep << "\n"
        << "Too rough (sigma>" << MAX_SIGMA_M << " m): " << too_rough << "\n"
        << "Unknown:          " << unknown << "\n"
        << "==============================" << std::endl;
}
