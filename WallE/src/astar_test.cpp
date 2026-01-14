#include <iostream>
#include "../include/astar.h"
#include "../include/field_map.h"

void astar_debug_run() {
    FieldMap map;
    map.populateStandardField();
    double sx = 0.0, sy = 0.0;
    double gx = 120.0, gy = 120.0;
    auto path = astar::findPath(map, sx, sy, gx, gy, 5.0, 10.0, 2.0);
    std::cout << "Path size: " << path.size() << "\n";
    for (auto &p : path) std::cout << p.first << "," << p.second << "\n";
}
