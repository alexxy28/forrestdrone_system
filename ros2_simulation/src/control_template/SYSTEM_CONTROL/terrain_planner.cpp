#include "terrain_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

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
    double goal_y)
{
    int sx, sy, gx, gy;

    if (!terrain_map_.worldToGrid(
            start_x, start_y, sx, sy))
        return {};

    if (!terrain_map_.worldToGrid(
            goal_x, goal_y, gx, gy))
        return {};

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
        heuristic(sx, sy, gx, gy)
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

        if (current.x == gx &&
            current.y == gy)
        {
            found = true;
            break;
        }

        for (const auto &direction :
             directions)
        {
            const int nx =
                current.x + direction[0];

            const int ny =
                current.y + direction[1];

            if (!terrain_map_
                     .isTraversable(nx, ny))
                continue;

            const bool diagonal =
                direction[0] != 0 &&
                direction[1] != 0;

            const double distance_cost =
                diagonal ? std::sqrt(2.0) : 1.0;

            const double slope_cost =
                terrain_map_.getSlope(nx, ny) /
                TerrainMap::MAX_SLOPE_DEG;

            const double sigma_cost =
                terrain_map_.getSigma(nx, ny) /
                TerrainMap::MAX_SIGMA_M;

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
                    heuristic(nx, ny,
                              gx, gy);

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
