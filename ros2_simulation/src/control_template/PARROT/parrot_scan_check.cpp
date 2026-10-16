// Self-check for Parrot::clusterScan. Build, then: ./build/control_template/parrot_scan_check
#include "PARROT/parrot.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

int main()
{
    const int n = 360;
    const double inc = 2.0 * M_PI / n;
    const double amin = -M_PI;
    const float none = std::numeric_limits<float>::infinity();
    auto cluster = [&](const std::vector<float>& r) {
        return Parrot::clusterScan(r, amin, inc, 0.35, 8.0, 0.5, 3);
    };

    // Post right behind the drone, across the +-180 deg seam: one object, not two.
    std::vector<float> seam(n, none);
    for (int i : {0, 1, 2, n - 3, n - 2, n - 1}) seam[i] = 3.0f;
    auto c = cluster(seam);
    assert(c.size() == 1 && c[0].size() == 6);
    assert(Parrot::clusterWidth(c[0]) < 2.0);

    // Wall all the way round (what the old down lidar fed in): one wide object, not a tiny tree.
    std::vector<float> ring(n, 3.0f);
    c = cluster(ring);
    assert(c.size() == 1);
    assert(Parrot::clusterWidth(c[0]) > 5.0);

    // Two posts at 4 m, ~1.4 m apart: two objects.
    std::vector<float> two(n, none);
    for (int i : {178, 179, 180, 200, 201, 202}) two[i] = 4.0f;
    assert(cluster(two).size() == 2);

    // Under min_points is noise; beyond range_max is ignored.
    std::vector<float> noise(n, none);
    noise[90] = noise[91] = 2.0f;
    for (int i = 10; i < 20; i++) noise[i] = 9.0f;
    assert(cluster(noise).empty());

    std::puts("parrot_scan_check: all passed");
}
