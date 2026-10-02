// Self-check: A* must route through the gap in a steep wall, and report no
// route when the wall has no gap. Exits non-zero on failure.
#include "terrain_map.hpp"
#include "terrain_planner.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cassert>
#include <cmath>
#include <iostream>

// Flat ground (z = 0) on a 20 m x 20 m grid with a 5 m high wall 1.5 m wide
// at x = 0. If `gap` is set, ground is left open for 7 m < y < 9 m.
static sensor_msgs::msg::PointCloud2 makeCloud(bool gap)
{
    const int cells = 40;
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(cells * cells);

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (int gy = 0; gy < cells; ++gy) {
        for (int gx = 0; gx < cells; ++gx, ++x, ++y, ++z) {
            *x = -10.0f + (gx + 0.5f) * 0.5f;
            *y = -10.0f + (gy + 0.5f) * 0.5f;
            const bool in_wall = std::fabs(*x) < 0.75f;
            const bool in_gap = gap && *y > 7.0f && *y < 9.0f;
            *z = (in_wall && !in_gap) ? 5.0f : 0.0f;
        }
    }
    return cloud;
}

int main()
{
    {
        TerrainMap map(0.5, 20.0, 20.0);
        map.updateFromPointCloud(makeCloud(true));
        TerrainPlanner planner(map);

        auto route = planner.plan(-5.0, -5.0, 5.0, -5.0);
        assert(!route.empty());

        // The only way across is the gap, so the route must climb to y > 7.
        double max_y = -100.0;
        for (const auto &p : route) max_y = std::max(max_y, p.second);
        assert(max_y > 7.0);
    }
    {
        TerrainMap map(0.5, 20.0, 20.0);
        map.updateFromPointCloud(makeCloud(false));
        TerrainPlanner planner(map);

        assert(planner.plan(-5.0, -5.0, 5.0, -5.0).empty());
    }
    std::cout << "planner_test passed" << std::endl;
    return 0;
}
