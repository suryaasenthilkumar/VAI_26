#include "../include/astar.h"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <cstdint>
#include <cstdio>

using namespace astar;

static inline int64_t keyFor(int ix, int iy) {
    return ( (int64_t)ix << 32 ) ^ (uint32_t)iy;
}

std::string Node::toString() const {
    char buf[64];
    sprintf(buf, "[%d,%d] g=%.2f h=%.2f f=%.2f", x, y, g, h, f);
    return std::string(buf);
}

static inline double heuristic(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1; double dy = y2 - y1; return std::sqrt(dx*dx + dy*dy);
}

std::vector<Point> astar::findPath(const FieldMap &map,
                                   double sx, double sy,
                                   double gx, double gy,
                                   double resolution_cm,
                                   double robot_radius_cm,
                                   double extra_margin_cm) {
    std::vector<Point> empty;
    // Quick rejects
    if (map.isPointInObstacle(sx, sy) || map.isPointInObstacle(gx, gy)) return empty;

    // Search bounds padding
    double pad = std::max(200.0, robot_radius_cm + extra_margin_cm + 50.0);
    double min_x = std::min(sx, gx) - pad;
    double min_y = std::min(sy, gy) - pad;
    double max_x = std::max(sx, gx) + pad;
    double max_y = std::max(sy, gy) + pad;

    int cols = std::max(3, (int)std::ceil((max_x - min_x) / resolution_cm));
    int rows = std::max(3, (int)std::ceil((max_y - min_y) / resolution_cm));

    auto cellCenter = [&](int ix, int iy)->Point {
        double x = min_x + (ix + 0.5) * resolution_cm;
        double y = min_y + (iy + 0.5) * resolution_cm;
        return {x,y};
    };

    // Precompute blocked cells with robot footprint inflation
    std::vector<char> blocked(cols * rows, 0);
    const int ROBOT_SAMPLES = 8; // perimeter sample points
    const double TWO_PI = 2.0 * std::acos(-1.0);
    const double total_clearance = std::max(0.0, robot_radius_cm + extra_margin_cm);
    for (int iy = 0; iy < rows; ++iy) {
        for (int ix = 0; ix < cols; ++ix) {
            Point c = cellCenter(ix, iy);
            double cx = c.first, cy = c.second;
            bool block = false;

            // If the robot center is in an obstacle, the cell is blocked
            if (map.isPointInObstacle(cx, cy)) block = true;

            // If not yet blocked, sample around the robot's perimeter at this cell
            if (!block && total_clearance > 0.0) {
                for (int i = 0; i < ROBOT_SAMPLES; i++) {
                    double angle = (TWO_PI * i) / ROBOT_SAMPLES;
                    double checkX = cx + total_clearance * std::cos(angle);
                    double checkY = cy + total_clearance * std::sin(angle);
                    if (map.isPointInObstacle(checkX, checkY)) { block = true; break; }
                }
            }

            // Keep original corner checks as a backstop
            if (!block) {
                double half = resolution_cm * 0.5;
                const double offs[4][2] = {{-half,-half},{half,-half},{half,half},{-half,half}};
                for (int k=0;k<4;k++){
                    double sxp = cx + offs[k][0];
                    double syp = cy + offs[k][1];
                    if (map.isPointInObstacle(sxp, syp)) { block = true; break; }
                }
            }

            blocked[iy * cols + ix] = block ? 1 : 0;
        }
    }

    auto toIndex = [&](double x, double y)->std::pair<int,int> {
        int ix = (int)std::floor((x - min_x) / resolution_cm);
        int iy = (int)std::floor((y - min_y) / resolution_cm);
        ix = std::max(0, std::min(cols-1, ix));
        iy = std::max(0, std::min(rows-1, iy));
        return {ix, iy};
    };

    auto sidx = toIndex(sx, sy);
    auto gidx = toIndex(gx, gy);

    auto findNearbyFree = [&](int ix, int iy)->std::pair<int,int>{
        if (!blocked[iy*cols + ix]) return {ix,iy};
        const int R = 3;
        for (int r=1;r<=R;r++){
            for (int dy=-r; dy<=r; ++dy) for (int dx=-r; dx<=r; ++dx) {
                int nx = ix + dx, ny = iy + dy;
                if (nx<0||nx>=cols||ny<0||ny>=rows) continue;
                if (!blocked[ny*cols + nx]) return {nx,ny};
            }
        }
        return {ix,iy};
    };

    sidx = findNearbyFree(sidx.first, sidx.second);
    gidx = findNearbyFree(gidx.first, gidx.second);

    if (blocked[sidx.second * cols + sidx.first] || blocked[gidx.second * cols + gidx.first]) return empty;

    struct PQNode { int ix, iy; double f; };
    struct PQComp { bool operator()(PQNode const &a, PQNode const &b) const { return a.f > b.f; } };
    std::priority_queue<PQNode, std::vector<PQNode>, PQComp> open;

    std::unordered_map<int64_t, double> gScore;
    std::unordered_map<int64_t, int64_t> cameFrom;

    int64_t start_key = keyFor(sidx.first, sidx.second);
    int64_t goal_key = keyFor(gidx.first, gidx.second);

    gScore[start_key] = 0.0;
    Point scenter = cellCenter(sidx.first, sidx.second);
    Point gcenter = cellCenter(gidx.first, gidx.second);
    double h0 = heuristic(scenter.first, scenter.second, gcenter.first, gcenter.second);
    open.push({sidx.first, sidx.second, h0});

    const int dirs = 8;
    const int dxs[8] = {1,1,0,-1,-1,-1,0,1};
    const int dys[8] = {0,1,1,1,0,-1,-1,-1};

    // Helper: swept collision check for robot footprint along a move between cell centers
    auto sweptIntersects = [&](double x1, double y1, double x2, double y2) -> bool {
        const double TWO_PI = 2.0 * std::acos(-1.0);
        const int PERIM_SAMPLES = 8;
        const double clearance = std::max(0.0, robot_radius_cm + extra_margin_cm);
        // If we have no clearance, fall back to original thin-segment check
        if (clearance <= 0.0) {
            return map.lineIntersectsObstacles(x1, y1, x2, y2);
        }
        double dx = x2 - x1, dy = y2 - y1;
        double dist = std::sqrt(dx*dx + dy*dy);
        int steps = std::max(3, (int)std::ceil(dist / std::max(1.0, resolution_cm * 0.5)));
        for (int i=0; i<=steps; ++i) {
            double t = (double)i / (double)steps;
            double px = x1 + t*dx;
            double py = y1 + t*dy;
            // Check center point
            if (map.isPointInObstacle(px, py)) return true;
            // Check perimeter around the robot footprint at this pose
            for (int k=0; k<PERIM_SAMPLES; ++k) {
                double ang = (TWO_PI * k) / PERIM_SAMPLES;
                double cx = px + clearance * std::cos(ang);
                double cy = py + clearance * std::sin(ang);
                if (map.isPointInObstacle(cx, cy)) return true;
            }
        }
        return false;
    };

    bool found = false;
    int max_iters = cols * rows * 10;
    int iters = 0;
    while(!open.empty() && iters++ < max_iters) {
        PQNode cur = open.top(); open.pop();
        int cx = cur.ix, cy = cur.iy;
        int64_t curk = keyFor(cx,cy);
        if (curk == goal_key) { found = true; break; }

        double gcur = gScore[curk];
        for (int d=0; d<dirs; ++d) {
            int nx = cx + dxs[d];
            int ny = cy + dys[d];
            if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
            if (blocked[ny * cols + nx]) continue;
            Point cc = cellCenter(cx,cy);
            Point nc = cellCenter(nx,ny);
            if (sweptIntersects(cc.first, cc.second, nc.first, nc.second)) continue;

            double moveCost = ( (dxs[d]==0 || dys[d]==0) ? resolution_cm : resolution_cm * 1.41421356237 );
            int64_t nk = keyFor(nx,ny);
            double tentative = gcur + moveCost;
            auto itg = gScore.find(nk);
            if (itg == gScore.end() || tentative < itg->second) {
                gScore[nk] = tentative;
                cameFrom[nk] = curk;
                double h = heuristic(nc.first, nc.second, gcenter.first, gcenter.second);
                open.push({nx, ny, tentative + h});
            }
        }
    }

    if (!found) return empty;

    // reconstruct path
    std::vector<Point> path;
    int64_t curk = goal_key;
    while (true) {
        int ix = (int)(curk >> 32);
        int iy = (int)(curk & 0xffffffff);
        path.push_back(cellCenter(ix, iy));
        if (curk == start_key) break;
        auto it = cameFrom.find(curk);
        if (it == cameFrom.end()) break;
        curk = it->second;
    }
    std::reverse(path.begin(), path.end());
    return path;
}
