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
    // optimistic: also drive through cells the map has not measured yet, and
    // cells only a little over the limits (more returns could bring them
    // under). No route even then means more mapping cannot help: the measured
    // terrain itself cuts the casualty off.
    std::vector<std::pair<double, double>>
    plan(double start_x,
         double start_y,
         double goal_x,
         double goal_y,
         bool optimistic = false);

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
    // Optimistic plan limits. A 0.1-0.3 m crease on the way to pose 2 reads as
    // 26-34 deg over one cell, yet the Husky drove over it in Gazebo; the
    // rock at pose 6 has 45-80 deg walls.
    const double OPTIMISTIC_SLOPE_DEG = 35.0;
    const double OPTIMISTIC_SIGMA_M = 0.75;
};

#endif
