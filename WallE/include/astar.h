#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <string>
#include <utility>
#include <cstdint>
#include <limits>
#include "field_map.h"

// Lightweight A* API and Node definition adapted to the project's FieldMap API.
// Coordinates are in centimeters.

namespace astar {

struct Node {
    int x; // grid x (column)
    int y; // grid y (row)
    double g; // cost from start
    double h; // heuristic to goal
    double f; // g + h
    int parent_x; // parent grid x
    int parent_y; // parent grid y

    Node() : x(0), y(0), g(0), h(0), f(0), parent_x(std::numeric_limits<int>::min()), parent_y(std::numeric_limits<int>::min()) {}
    Node(int _x, int _y) : x(_x), y(_y), g(0), h(0), f(0), parent_x(std::numeric_limits<int>::min()), parent_y(std::numeric_limits<int>::min()) {}
    std::string toString() const;
};

// Path is returned as vector of (x_cm,y_cm) points
using Point = std::pair<double,double>;

// Find a path using grid-based A*.
// - map: FieldMap reference for obstacle queries
// - sx,sy: start in cm
// - gx,gy: goal in cm
// - resolution_cm: cell size in cm
// - robot_radius_cm: clearing radius (approx)
// Returns empty vector on failure.
std::vector<Point> findPath(const FieldMap &map,
                            double sx, double sy,
                            double gx, double gy,
                            double resolution_cm = 5.0,
                            double robot_radius_cm = 10.0,
                            double extra_margin_cm = 2.0);

} // namespace astar

#endif // ASTAR_H
