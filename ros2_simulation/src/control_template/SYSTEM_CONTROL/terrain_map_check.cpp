// Self-check for TerrainMap. Build, then: ./build/control_template/terrain_map_check
#include "terrain_map.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

static bool near(double a, double b, double tol = 1e-4) { return std::abs(a - b) < tol; }

int main()
{
    // Grid: odd cell count, origin is the centre cell, round trip is exact.
    TerrainMap map(0.5, 40.0, 40.0);
    assert(map.getWidth() == 81 && map.getHeight() == 81);
    int gx, gy;
    assert(map.worldToGrid(0.0, 0.0, gx, gy) && gx == 40 && gy == 40);
    for (int g : {0, 17, 80}) {
        const auto [x, y] = map.gridToWorld(g, g);
        assert(map.worldToGrid(x, y, gx, gy) && gx == g && gy == g);
    }

    // Running mean/variance == two-pass mean/population variance (below cap).
    std::mt19937 rng(1);
    std::normal_distribution<double> noise(2.0, 0.1);
    std::vector<double> z;
    for (int i = 0; i < 20; ++i) {
        z.push_back(noise(rng));
        map.addPoint(1.0, 1.0, z.back());
    }
    double mean = 0, var = 0;
    for (double v : z) mean += v / z.size();
    for (double v : z) var += (v - mean) * (v - mean) / z.size();

    // A rough cell (ground + canopy, mean 2 m like its neighbours so only sigma
    // can block it), surrounded by smooth ground at 2 m.
    for (int i = 0; i < 20; ++i) map.addPoint(2.0, 1.0, i % 2 ? 0.5 : 3.5);
    for (double x = -1.0; x <= 3.0; x += 0.5)
        for (double y = 0.0; y <= 2.0; y += 0.5)
            if (!(x == 2.0 && y == 1.0) && !(x == 1.0 && y == 1.0))
                for (int i = 0; i < 3; ++i) map.addPoint(x, y, 2.0);

    // toCloud -> updateFromPointCloud on a second map keeps every cell.
    TerrainMap copy(0.5, 40.0, 40.0);
    copy.updateFromPointCloud(map.toCloud("odom", builtin_interfaces::msg::Time()));
    map.rebuild();
    assert(map.worldToGrid(1.0, 1.0, gx, gy));
    assert(near(map.getSigma(gx, gy), std::sqrt(var)));
    assert(near(copy.getSigma(gx, gy), map.getSigma(gx, gy)));
    for (const TerrainMap *m : {&map, &copy}) {
        assert(m->hasTerrain());
        assert(m->worldToGrid(2.0, 1.0, gx, gy));
        assert(near(m->getSigma(gx, gy), 1.5) && near(m->getSlope(gx, gy), 0.0, 1e-3));
        assert(!m->isTraversable(gx, gy));                                      // too rough
        assert(m->worldToGrid(0.0, 1.5, gx, gy) && m->isTraversable(gx, gy));   // flat, smooth
    }

    // Past the count cap the mean follows a step change instead of freezing.
    TerrainMap step(0.5, 40.0, 40.0);
    for (int i = 0; i < 1000; ++i) step.addPoint(0.0, 0.0, 0.0);
    for (int i = 0; i < 300; ++i) step.addPoint(0.0, 0.0, 5.0);
    const auto cloud = step.toCloud("odom", builtin_interfaces::msg::Time());
    const float stepped = *reinterpret_cast<const float *>(&cloud.data[cloud.fields[2].offset]);
    assert(stepped > 4.9f);

    std::puts("terrain_map_check: OK");
    return 0;
}
