#ifndef TRACE_TERRAIN_PLANNER_HPP
#define TRACE_TERRAIN_PLANNER_HPP

#include "terrain_map.hpp"

#include <utility>
#include <vector>

class TerrainPlanner
{
public:
    explicit TerrainPlanner(TerrainMap &terrain_map);

    // Terrain-aware A*: distance plus slope and roughness cost; empty if no route.
    std::vector<std::pair<double, double>>
    plan(double start_x,
         double start_y,
         double goal_x,
         double goal_y);

private:
    TerrainMap &terrain_map_;

    double heuristic(int x1, int y1,
                     int x2, int y2) const;

    const double DISTANCE_WEIGHT = 1.0;
    const double SLOPE_WEIGHT = 4.0;
    const double SIGMA_WEIGHT = 4.0;
    // Husky footprint is ~1.0 x 0.7 m; clear its own cells plus a margin.
    const double START_CLEAR_RADIUS_M = 1.0;
    // Stop in a ring this far from the casualty: close enough to deliver
    // supplies, never on top of the person.
    const double GOAL_MIN_STANDOFF_M = 1.6;   // body reaches ~0.9 m from its centre
    const double GOAL_SEARCH_RADIUS_M = 2.5;
};

#endif
