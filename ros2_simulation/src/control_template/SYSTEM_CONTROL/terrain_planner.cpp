#include "terrain_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <rclcpp/rclcpp.hpp>

struct SearchNode
{
    int x;
    int y;
    double f;

    bool operator>(const SearchNode &other) const
    {
        return f > other.f;
    }
};

TerrainPlanner::TerrainPlanner(
    TerrainMap &terrain_map)
    : terrain_map_(terrain_map)
{
}


double TerrainPlanner::heuristic(
    int x1, int y1,
    int x2, int y2) const
{
    return std::hypot(
        static_cast<double>(x2 - x1),
        static_cast<double>(y2 - y1));
}



std::vector<std::pair<double, double>>
TerrainPlanner::plan(
    double start_x,
    double start_y,
    double goal_x,
    double goal_y,
    bool optimistic)
{
    int sx, sy, gx, gy;

    if (!terrain_map_.worldToGrid(
            start_x, start_y, sx, sy))
    {
        RCLCPP_WARN(
            rclcpp::get_logger("terrain_planner"),
            "Route start (%.2f, %.2f) is outside the terrain map",
            start_x, start_y);
        return {};
    }

    if (!terrain_map_.worldToGrid(
            goal_x, goal_y, gx, gy))
    {
        RCLCPP_WARN(
            rclcpp::get_logger("terrain_planner"),
            "Route goal (%.2f, %.2f) is outside the terrain map",
            goal_x, goal_y);
        return {};
    }

    // Goal = any drivable cell in a stand-off ring around the casualty
    // (GOAL_MIN_STANDOFF_M .. GOAL_SEARCH_RADIUS_M). Never the casualty's own
    // cell: a person lying down can look "drivable" in the terrain map, and
    // Nav2 then refuses the goal because the person is an obstacle (seen in
    // Gazebo, 27 Oct). A* stops at the first ring cell it reaches, i.e. the
    // closest stand-off point on the Husky's side.
    // Unknown = inside the map but no slope/roughness yet (NaN). Outside the
    // map getSlope() is finite and too steep, so that never counts.
    auto drivable = [&](int x, int y) {
        if (terrain_map_.isTraversable(x, y))
            return true;
        if (!optimistic)
            return false;
        const double slope = terrain_map_.getSlope(x, y);
        const double sigma = terrain_map_.getSigma(x, y);
        if (!std::isfinite(slope) || !std::isfinite(sigma))
            return true;   // unknown (outside the map both are finite)
        return slope <= OPTIMISTIC_SLOPE_DEG && sigma <= OPTIMISTIC_SIGMA_M;
    };
    const double inner_cells = GOAL_MIN_STANDOFF_M / terrain_map_.resolution();
    const double outer_cells = GOAL_SEARCH_RADIUS_M / terrain_map_.resolution();
    const int cx = gx;
    const int cy = gy;
    auto in_ring = [&](int x, int y) {
        const double d = std::hypot(x - cx, y - cy);
        return d >= inner_cells && d <= outer_cells;
    };
    {
        bool any = false;
        const int r = static_cast<int>(std::ceil(outer_cells));
        for (int dy = -r; dy <= r && !any; ++dy)
            for (int dx = -r; dx <= r && !any; ++dx)
                any = in_ring(cx + dx, cy + dy) && drivable(cx + dx, cy + dy);
        if (!any) {
            RCLCPP_WARN(rclcpp::get_logger("terrain_planner"),
                        "No drivable cell %.1f-%.1f m from the casualty", GOAL_MIN_STANDOFF_M,
                        GOAL_SEARCH_RADIUS_M);
            return {};
        }
    }
    // Admissible heuristic: every step costs at least 1 cell, and the ring
    // starts outer_cells from the casualty.
    auto ring_heuristic = [&](int x, int y) {
        return std::max(0.0, std::hypot(x - cx, y - cy) - outer_cells);
    };
    const int start_clear_cells =
        static_cast<int>(std::ceil(START_CLEAR_RADIUS_M / terrain_map_.resolution()));

    const int width =
        terrain_map_.getWidth();

    const int height =
        terrain_map_.getHeight();

    const int total =
        width * height;

    const double inf =
        std::numeric_limits<double>::infinity();

    std::vector<double>
        g_score(total, inf);

    std::vector<int>
        parent(total, -1);

    auto index =
        [width](int x, int y)
        {
            return y * width + x;
        };

    std::priority_queue<
        SearchNode,
        std::vector<SearchNode>,
        std::greater<SearchNode>>
        open;

    g_score[index(sx, sy)] = 0.0;

    open.push({
        sx,
        sy,
        ring_heuristic(sx, sy)
    });

    const int directions[8][2] = {
        {1, 0},
        {-1, 0},
        {0, 1},
        {0, -1},
        {1, 1},
        {1, -1},
        {-1, 1},
        {-1, -1}
    };

    bool found = false;

    while (!open.empty())
    {
        SearchNode current =
            open.top();

        open.pop();

        if (in_ring(current.x, current.y) &&
            drivable(current.x, current.y))
        {
            found = true;
            gx = current.x;   // the stand-off cell reached: the route ends here
            gy = current.y;
            break;
        }

        for (const auto &direction :
             directions)
        {
            const int nx =
                current.x + direction[0];

            const int ny =
                current.y + direction[1];

            // Cells under/around the Husky are its own body in the drone's
            // map (~0.5 m high), so they read as steep. Let it drive out.
            const bool near_start =
                std::hypot(nx - sx, ny - sy) <= start_clear_cells;
            if (!near_start && !drivable(nx, ny))
                continue;

            const bool diagonal =
                direction[0] != 0 &&
                direction[1] != 0;

            const double distance_cost =
                diagonal ? std::sqrt(2.0) : 1.0;

            // Near the start the slope/sigma are the Husky itself; cap them.
            const double slope_cost = std::isfinite(terrain_map_.getSlope(nx, ny))
                ? std::min(terrain_map_.getSlope(nx, ny) / TerrainMap::MAX_SLOPE_DEG, 1.0) : 1.0;

            const double sigma_cost = std::isfinite(terrain_map_.getSigma(nx, ny))
                ? std::min(terrain_map_.getSigma(nx, ny) / TerrainMap::MAX_SIGMA_M, 1.0) : 1.0;

            const double movement_cost =
                DISTANCE_WEIGHT *
                    distance_cost +
                SLOPE_WEIGHT *
                    slope_cost +
                SIGMA_WEIGHT *
                    sigma_cost;

            const int current_index =
                index(current.x, current.y);

            const int neighbour_index =
                index(nx, ny);

            const double tentative =
                g_score[current_index] +
                movement_cost;

            if (tentative <
                g_score[neighbour_index])
            {
                g_score[neighbour_index] =
                    tentative;

                parent[neighbour_index] =
                    current_index;

                const double f =
                    tentative +
                    ring_heuristic(nx, ny);

                open.push({
                    nx,
                    ny,
                    f
                });
            }
        }
    }

    if (!found)
        return {};

    std::vector<std::pair<double, double>>
        path;

    int current =
        index(gx, gy);

    const int start =
        index(sx, sy);

    while (current != start)
    {
        const int x =
            current % width;

        const int y =
            current / width;

        path.push_back(
            terrain_map_.gridToWorld(x, y));

        current =
            parent[current];

        if (current < 0)
            return {};
    }

    path.push_back(
        terrain_map_.gridToWorld(sx, sy));

    std::reverse(
        path.begin(),
        path.end());

    return path;
}
