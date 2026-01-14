#ifndef FIELD_MAP_H
#define FIELD_MAP_H

#include <vector>
#include <string>

struct Rect {
    std::string name;
    double cx; // center x (cm)
    double cy; // center y (cm)
    double width; // cm (along local X)
    double height; // cm (along local Y)
    double rotation_deg; // degrees clockwise

    Rect();
    Rect(const std::string &n, double x, double y, double w, double h, double rot_deg = 0.0);

    // tests
    bool contains(double x, double y) const;
    bool intersectsSegment(double x1, double y1, double x2, double y2) const;
};

class FieldMap {
public:
    FieldMap();

    // add generic obstacle (rectangle)
    //
    // Declaration: this method is declared here in the header so other
    // modules can add obstacles to the field map. It accepts a `Rect`
    // describing the obstacle geometry (center x/y in cm, width, height,
    // rotation in degrees) and appends it to the internal obstacle list.
    //
    // Definition: the implementation is in `src/field_map.cpp` where the
    // rectangle is pushed into `m_obstacles`.
    void addObstacle(const Rect &r);

    // convenience: addGoal is same as addObstacle but kept for readability
    void addGoal(const Rect &r);

    // populate a set of default goal rectangles for the standard field
    // Positions/sizes are placeholders and should be tuned to match your field
    void populateStandardField();

    // queries
    const std::vector<Rect>& getObstacles() const;
    bool isPointInObstacle(double x, double y) const;
    bool lineIntersectsObstacles(double x1, double y1, double x2, double y2) const;

    // Field bounds support. If bounds are set, points outside the bounds are
    // considered blocked by the field boundary.
    void setFieldBounds(double min_x, double min_y, double max_x, double max_y);
    void setFieldSizeCentered(double width_cm, double height_cm); // centered at 0,0
    bool hasBounds() const;
    void getFieldBounds(double &min_x, double &min_y, double &max_x, double &max_y) const;
    // Treat the field boundary as thin obstacles (walls) inside the field.
    // When enabled, four thin Rect obstacles are added along the edges so
    // planners that only consider obstacles can't pass outside the field.
    void setBoundaryAsObstacle(bool enabled, double thickness_cm = 5.0);
    bool isBoundaryAsObstacleEnabled() const;
    double getBoundaryThickness() const;

private:
    // List of obstacle rectangles in world coordinates (cm).
    // Each `Rect` represents a static area the robot must avoid (goals, walls, field boundary walls, user-added obstacles).
    // Path planners and collision checks query m_obstacles via isPointInObstacle() and lineIntersectsObstacles() todetermine forbidden regions.
    std::vector<Rect> m_obstacles;
    // optional field bounding box (cm)
    bool m_hasBounds;
    double m_minX;
    double m_minY;
    double m_maxX;
    double m_maxY;
    // Boundary wall control flags (defined here, initialized in constructor).
    // m_boundaryAsObstacle: when true, thin wall Rects are added along field edges in populateStandardField() so planners treat the boundary as
    // an obstacle and will not attempt to path outside the field.
    // m_boundaryThicknessCm: controls the thickness of the wall rectangles.
    bool m_boundaryAsObstacle;
    double m_boundaryThicknessCm;
};

#endif // FIELD_MAP_H
