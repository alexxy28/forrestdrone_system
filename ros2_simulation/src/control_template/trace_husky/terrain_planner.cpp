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
TerrainPlanner::planDemo2(
    double start_x,
    double start_y,
    double goal_x,
    double goal_y)
{
    const double resolution = 0.5;

    const double margin = 3.0;

    const double min_x =
        std::min(start_x, goal_x) - margin;

    const double min_y =
        std::min(start_y, goal_y) - margin;

    const double max_x =
        std::max(start_x, goal_x) + margin;

    const double max_y =
        std::max(start_y, goal_y) + margin;

    const int width =
        static_cast<int>(
            std::ceil((max_x - min_x) /
                      resolution)) + 1;

    const int height =
        static_cast<int>(
            std::ceil((max_y - min_y) /
                      resolution)) + 1;

    auto worldToGrid =
        [&](double x, double y)
        {
            return std::pair<int, int>(
                static_cast<int>(
                    std::round(
                        (x - min_x) /
                        resolution)),
                static_cast<int>(
                    std::round(
                        (y - min_y) /
                        resolution)));
        };

    auto gridToWorld =
        [&](int x, int y)
        {
            return std::pair<double, double>(
                min_x + x * resolution,
                min_y + y * resolution);
        };

    const auto start =
        worldToGrid(start_x, start_y);

    const auto goal =
        worldToGrid(goal_x, goal_y);

    const int total = width * height;

    const double inf =
        std::numeric_limits<double>::infinity();

    std::vector<double> g(total, inf);
    std::vector<int> parent(total, -1);
    std::vector<bool> blocked(total, false);

    auto idx =
        [width](int x, int y)
        {
            return y * width + x;
        };

    /*
     * Demo obstacle:
     *
     * Put a short wall halfway between
     * start and casualty.
     *
     * This forces A* to demonstrate that
     * it is generating a route instead of
     * simply returning a straight line.
     */
    const double wall_x =
        (start_x + goal_x) / 2.0;

    const double centre_y =
        (start_y + goal_y) / 2.0;

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const auto world =
                gridToWorld(x, y);

            if (std::abs(world.first -
                         wall_x) < 0.35 &&
                std::abs(world.second -
                         centre_y) < 0.5)
            {
                blocked[idx(x, y)] = true;
            }
        }
    }

    // Never block start or casualty.
    blocked[idx(start.first,
                start.second)] = false;

    blocked[idx(goal.first,
                goal.second)] = false;

    struct Node
    {
        int x;
        int y;
        double f;

        bool operator>(
            const Node &other) const
        {
            return f > other.f;
        }
    };

    std::priority_queue<
        Node,
        std::vector<Node>,
        std::greater<Node>>
        open;

    g[idx(start.first,
          start.second)] = 0.0;

    open.push({
        start.first,
        start.second,
        heuristic(
            start.first,
            start.second,
            goal.first,
            goal.second)
    });

    const int directions[8][2] = {
        {1,0}, {-1,0},
        {0,1}, {0,-1},
        {1,1}, {1,-1},
        {-1,1}, {-1,-1}
    };

    bool found = false;

    while (!open.empty())
    {
        const Node current = open.top();
        open.pop();

        if (current.x == goal.first &&
            current.y == goal.second)
        {
            found = true;
            break;
        }

        for (const auto &d : directions)
        {
            const int nx =
                current.x + d[0];

            const int ny =
                current.y + d[1];

            if (nx < 0 || ny < 0 ||
                nx >= width ||
                ny >= height)
                continue;

            if (blocked[idx(nx, ny)])
                continue;

            const bool diagonal =
                d[0] != 0 && d[1] != 0;

            const double move =
                diagonal ?
                    std::sqrt(2.0) : 1.0;

            const double tentative =
                g[idx(current.x,
                      current.y)] +
                move;

            if (tentative <
                g[idx(nx, ny)])
            {
                g[idx(nx, ny)] =
                    tentative;

                parent[idx(nx, ny)] =
                    idx(current.x,
                        current.y);

                const double f =
                    tentative +
                    heuristic(
                        nx, ny,
                        goal.first,
                        goal.second);

                open.push({nx, ny, f});
            }
        }
    }

    if (!found)
        return {};

    std::vector<
        std::pair<double, double>>
        path;

    int current =
        idx(goal.first, goal.second);

    const int start_index =
        idx(start.first, start.second);

    while (current != start_index)
    {
        const int x =
            current % width;

        const int y =
            current / width;

        path.push_back(
            gridToWorld(x, y));

        current = parent[current];

        if (current < 0)
            return {};
    }

    path.push_back(
        {start_x, start_y});

    std::reverse(
        path.begin(),
        path.end());

    return path;
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

            const double slope =
                terrain_map_.getSlope(nx, ny);

            const double slope_cost =
                slope / 25.0;

            const double movement_cost =
                DISTANCE_WEIGHT *
                    distance_cost +
                SLOPE_WEIGHT *
                    slope_cost;

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
