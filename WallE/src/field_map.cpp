#include "../include/field_map.h"
#include <cmath>
#include <algorithm>

static double deg2rad(double d) { return d * M_PI / 180.0; }

Rect::Rect() : name(""), cx(0), cy(0), width(0), height(0), rotation_deg(0) {}
Rect::Rect(const std::string &n, double x, double y, double w, double h, double rot_deg)
    : name(n), cx(x), cy(y), width(w), height(h), rotation_deg(rot_deg) {}

// rotate point (px,py) around center (ox,oy) by -rotation_deg (to bring into rect's local frame)
static void rotatePointToLocal(double ox, double oy, double rot_deg, double px, double py, double &out_x, double &out_y) {
    double theta = -deg2rad(rot_deg); // negative to rotate point into rectangle local frame
    double s = sin(theta);
    double c = cos(theta);
    // translate
    double tx = px - ox;
    double ty = py - oy;
    // rotate
    out_x = tx * c - ty * s;
    out_y = tx * s + ty * c;
}

bool Rect::contains(double x, double y) const {
    double lx, ly;
    rotatePointToLocal(cx, cy, rotation_deg, x, y, lx, ly);
    double hw = width / 2.0;
    double hh = height / 2.0;
    return (lx >= -hw && lx <= hw && ly >= -hh && ly <= hh);
}

// helper: orientation driven segment intersection
static int orient(double ax, double ay, double bx, double by, double cx, double cy) {
    double v = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (fabs(v) < 1e-9) return 0;
    return (v > 0) ? 1 : -1;
}

static bool onSegment(double ax, double ay, double bx, double by, double px, double py) {
    return (std::min(ax, bx) - 1e-9 <= px && px <= std::max(ax, bx) + 1e-9) &&
           (std::min(ay, by) - 1e-9 <= py && py <= std::max(ay, by) + 1e-9);
}

static bool segmentIntersects(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
    int o1 = orient(x1,y1,x2,y2,x3,y3);
    int o2 = orient(x1,y1,x2,y2,x4,y4);
    int o3 = orient(x3,y3,x4,y4,x1,y1);
    int o4 = orient(x3,y3,x4,y4,x2,y2);

    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && onSegment(x1,y1,x2,y2,x3,y3)) return true;
    if (o2 == 0 && onSegment(x1,y1,x2,y2,x4,y4)) return true;
    if (o3 == 0 && onSegment(x3,y3,x4,y4,x1,y1)) return true;
    if (o4 == 0 && onSegment(x3,y3,x4,y4,x2,y2)) return true;
    return false;
}

bool Rect::intersectsSegment(double x1, double y1, double x2, double y2) const {
    // early-out if either endpoint inside
    if (contains(x1, y1) || contains(x2, y2)) return true;

    // compute rectangle corners in world frame (counter-clockwise)
    double hw = width / 2.0;
    double hh = height / 2.0;
    // local corners
    double local[4][2] = {{-hw,-hh},{hw,-hh},{hw,hh},{-hw,hh}};
    double world[4][2];
    double theta = deg2rad(rotation_deg);
    double s = sin(theta), c = cos(theta);
    for (int i=0;i<4;i++) {
        double lx = local[i][0];
        double ly = local[i][1];
        double wx = c*lx - s*ly + cx;
        double wy = s*lx + c*ly + cy;
        world[i][0] = wx; world[i][1] = wy;
    }
    // check each edge
    for (int i=0;i<4;i++) {
        int j = (i+1)%4;
        if (segmentIntersects(x1,y1,x2,y2, world[i][0], world[i][1], world[j][0], world[j][1])) return true;
    }
    return false;
}

// FieldMap implementation
// Constructor: initializes boundary walls as enabled by default.
// m_boundaryAsObstacle is set to true so that populateStandardField()
// will automatically add thin wall obstacles along the field edges.
FieldMap::FieldMap() : m_hasBounds(false), m_minX(0), m_minY(0), m_maxX(0), m_maxY(0), m_boundaryAsObstacle(true), m_boundaryThicknessCm(5.0) {}

void FieldMap::setFieldBounds(double min_x, double min_y, double max_x, double max_y) {
    m_minX = min_x;
    m_minY = min_y;
    m_maxX = max_x;
    m_maxY = max_y;
    m_hasBounds = true;
}

void FieldMap::setFieldSizeCentered(double width_cm, double height_cm) {
    double halfW = width_cm * 0.5;
    double halfH = height_cm * 0.5;
    setFieldBounds(-halfW, -halfH, halfW, halfH);
}

bool FieldMap::hasBounds() const { return m_hasBounds; }

void FieldMap::getFieldBounds(double &min_x, double &min_y, double &max_x, double &max_y) const {
    min_x = m_minX; min_y = m_minY; max_x = m_maxX; max_y = m_maxY;
}

void FieldMap::setBoundaryAsObstacle(bool enabled, double thickness_cm) {
    // Boundary walls are always enabled for this field. Only allow updating thickness.
    if (thickness_cm > 0) m_boundaryThicknessCm = thickness_cm;
}

bool FieldMap::isBoundaryAsObstacleEnabled() const { return m_boundaryAsObstacle; }

double FieldMap::getBoundaryThickness() const { return m_boundaryThicknessCm; }

// Implementation: addObstacle
// This is the definition of `addObstacle()` declared in the header.
// It simply appends the provided `Rect` to the internal `m_obstacles`
// list. Other components (A*, collision checks) query `m_obstacles`
// via `getObstacles()` or helper methods like `isPointInObstacle()` and
// `lineIntersectsObstacles()` to treat these rectangles as forbidden areas.
void FieldMap::addObstacle(const Rect &r) {
    m_obstacles.push_back(r);
}

void FieldMap::addGoal(const Rect &r) {
    addObstacle(r);
}

const std::vector<Rect>& FieldMap::getObstacles() const {
    return m_obstacles;
}

bool FieldMap::isPointInObstacle(double x, double y) const {
    // If bounds set, points outside bounds are considered blocked
    if (m_hasBounds) {
        if (x < m_minX || x > m_maxX || y < m_minY || y > m_maxY) return true;
    }
    for (const auto &r : m_obstacles) {
        if (r.contains(x,y)) return true;
    }
    return false;
}

bool FieldMap::lineIntersectsObstacles(double x1, double y1, double x2, double y2) const {
    // If bounds set, treat any segment with an endpoint outside bounds as intersecting
    if (m_hasBounds) {
        if (x1 < m_minX || x1 > m_maxX || y1 < m_minY || y1 > m_maxY) return true;
        if (x2 < m_minX || x2 > m_maxX || y2 < m_minY || y2 > m_maxY) return true;
    }
    for (const auto &r : m_obstacles) {
        if (r.intersectsSegment(x1,y1,x2,y2)) return true;
    }
    return false;
}

// Populate the field with reasonable default rectangles for goals.
// These are approximate and intended as a starting point — tune sizes/positions
// to match your real field measurements.
void FieldMap::populateStandardField() {
    m_obstacles.clear();

    // Default field bounds: 144in x 144in -> convert to cm
    const double field_inches = 144.0;
    const double field_cm = field_inches * 2.54; // 365.76 cm
    setFieldSizeCentered(field_cm, field_cm);

    // Build rectangles from measured midpoints and dimensions provided by user.
    // Format per-entry: name, x1,y1 (blue midpoint), x2,y2 (red midpoint), long_cm, short_cm, margin_cm
    struct Entry { const char *name; double x1,y1,x2,y2,long_cm,short_cm,margin; } entries[] = {
        // {"LongGoal1", 63.0, 121.0, -63.0, 121.0, 124.0, 16.0, 5.0},
        // {"LongGoal2", 63.0, -121.0, -63.0, -121.0, 124.0, 16.0, 5.0},
        {"LongGoal1", 63.0, 150.0, -63.0, 150.0, 124.0, 62.0, 5.0},
        {"LongGoal2", 63.0, -150.0, -63.0, -150.0, 124.0, 62.0, 5.0},
        // Combine middle X-shaped pair into one square obstacle centered at (0,0)
        // Sized to cover both rotated middles (~66cm span); give a bit of margin.
        {"MiddleSquare", -27.0, 0.0, 27.0, 0.0, 54.0, 54.0, 5.0},
        {"BluePark", 135.0, 0.0, 183.0, 0.0, 48.0, 48.0, 5.0},
        {"RedPark", -135.0, 0.0, -183.0, 0.0, 48.0, 48.0, 5.0},
        {"MLB1", 183.0, 122.0, 164.0, 122.0, 19.0, 16.0, 5.0},
        {"MLB2", 183.0, -122.0, 164.0, -122.0, 19.0, 16.0, 5.0},
        {"MLR1", -183.0, 122.0, -164.0, 122.0, 19.0, 16.0, 5.0},
        {"MLR2", -183.0, -122.0, -164.0, -122.0, 19.0, 16.0, 5.0},

    };

    // Create Rect objects for each goal and add them as obstacles.
    // These represent static goal geometry (long/mid goals) so path
    // planners will avoid driving through goal areas.
    for (auto &e : entries) {
        double cx = (e.x1 + e.x2) / 2.0;
        double cy = (e.y1 + e.y2) / 2.0;
        double dx = e.x2 - e.x1;
        double dy = e.y2 - e.y1;
        double rot_deg = atan2(dy, dx) * 180.0 / M_PI;
        double w = e.long_cm + 2.0 * e.margin;
        double h = e.short_cm + 2.0 * e.margin;
        // here: addGoal() calls addObstacle() under the hood, inserting
        // this goal rectangle into the FieldMap obstacle list so A* and
        // other checks treat the goal as an obstacle.
        addGoal(Rect(std::string(e.name), cx, cy, w, h, rot_deg));
    }

    // **BOUNDARY WALLS MADE INTO OBSTACLES HERE**
    // When m_boundaryAsObstacle is enabled (true by default), create four thin
    // wall Rect obstacles positioned along the inside edges of the field bounds.
    // Each wall is centered along its respective edge with thickness m_boundaryThicknessCm.
    // These walls are added via addObstacle() and become part of m_obstacles,
    // ensuring A* and collision checks treat the field boundary as impassable.
    if (m_hasBounds && m_boundaryAsObstacle) {
        double fieldW = m_maxX - m_minX;
        double fieldH = m_maxY - m_minY;
        double t = m_boundaryThicknessCm;
        // Left wall: center at (minX + t/2, 0)
        addObstacle(Rect(std::string("BoundaryLeft"), m_minX + t*0.5, 0.0, t, fieldH, 0.0));
        // Right wall
        addObstacle(Rect(std::string("BoundaryRight"), m_maxX - t*0.5, 0.0, t, fieldH, 0.0));
        // Top wall
        addObstacle(Rect(std::string("BoundaryTop"), 0.0, m_maxY - t*0.5, fieldW, t, 0.0));
        // Bottom wall
        addObstacle(Rect(std::string("BoundaryBottom"), 0.0, m_minY + t*0.5, fieldW, t, 0.0));
    }

    // You can add more obstacles manually with addObstacle()
}
