/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Helper movement functions for VEX AI program              */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "vex.h"
#include "ai_functions.h"
#include "astar.h"
#include "field_map.h"
#include <cmath>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
using namespace vex;
using namespace std;

// Set default PID constants for the chassis
void default_constants() {
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

// Calculates the distance to the coordinates from the current robot position
double distanceTo(double target_x, double target_y){
    double distance = sqrt(pow((target_x - GPS.xPosition()), 2) + pow((target_y - GPS.yPosition()), 2));
    return distance;
}

// Calculates the bearing to drive to the target coordinates in a straight line aligned with global coordinate/heading system.
double calculateBearing(double currX, double currY, double targetX, double targetY) {
    // Calculate the difference in coordinates
    double dx = targetX - currX;
    double dy = targetY - currY;

    // Calculate the bearing in radians
    double bearing_rad = atan2(dy, dx);

    // Convert to degrees
    double bearing_deg = bearing_rad * 180 / M_PI;

    // Normalize to the range 0 to 360
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    // Convert from mathematical to navigation coordinates
    bearing_deg = fmod(90 - bearing_deg, 360);
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    return bearing_deg;
}

// Turns the robot to face the angle specified, taking into account a tolerance and speed of turn.
void turnTo(double angle, int tolerance, int speed){
    double current_heading = GPS.heading();
    double angle_to_turn = angle - current_heading;

    // Normalize the angle to the range [-180, 180]
    while (angle_to_turn > 180) angle_to_turn -= 360;
    while (angle_to_turn < -180) angle_to_turn += 360;

    // Determine the direction to turn (left or right)
    turnType direction = angle_to_turn > 0 ? turnType::right : turnType::left;
    Drivetrain.turn(direction, speed, velocityUnits::pct);
    while (1) {
    
        current_heading = GPS.heading();
        // Check if the current heading is within a tolerance of degrees to the target
        if (current_heading > (angle - tolerance) && current_heading < (angle + tolerance)) {
            break;
        }

    }
    Drivetrain.stop();
}

// Moves the robot toward the target at the specificed heading, for a distance at a given speed.
void driveFor(int heading, double distance, int speed){
    // Determine the smallest degree of turn
    double angle_to_turn = heading - GPS.heading();
    while (angle_to_turn > 180) angle_to_turn -= 360;
    while (angle_to_turn < -180) angle_to_turn += 360;

    // Decide whether to move forward or backward
    // Allos for a 5 degree margin of error that defaults to forward
    directionType direction = fwd;
    if (std::abs(angle_to_turn) > 105) {
        angle_to_turn += angle_to_turn > 0 ? -180 : 180;
        direction = directionType::rev;
    } else if (std::abs(angle_to_turn) < 75) {
        angle_to_turn += angle_to_turn > 0 ? 180 : -180;
        direction = directionType::fwd;
    }

    Drivetrain.driveFor(direction, distance, vex::distanceUnits::cm, speed, velocityUnits::pct);
}

// Method that moves to a given (x,y) position and a desired target theta to finish movement facing
void moveToPosition(double target_x, double target_y, double target_theta = -1) {
    // Calculate the angle to turn to face the target
    double initialHeading = calculateBearing(GPS.xPosition(), GPS.yPosition(), target_x, target_y);
    // Turn to face the target
    //turnTo(intialHeading, 3, 10);
    Drivetrain.turnToHeading(initialHeading, rotationUnits::deg, 10, velocityUnits::pct);
    double distance = distanceTo(target_x, target_y);
    // Move to the target, only 30% of total distance to account for error
    driveFor(initialHeading, distance*0.3, 30);

    // Recalculate the heading and distance to the target
    double heading = calculateBearing(GPS.xPosition(), GPS.yPosition(), target_x, target_y);
    //turnTo(heading, 3, 10);
    Drivetrain.turnToHeading(heading, rotationUnits::deg, 10, velocityUnits::pct);
    distance = distanceTo(target_x, target_y);
    // Move to the target, completing the remaining distance
    driveFor(heading, distance, 20);

    // Turn to the final target heading if specified, otherwise use current heading
    if (target_theta == -1){
        target_theta = GPS.heading();
    }
    //turnTo(target_theta, 2, 10);
    Drivetrain.turnToHeading(target_theta, rotationUnits::deg, 10, velocityUnits::pct);
}

// Map OBJECT to Jetson classID (Jetson uses 0=Blue, 1=Red)
static int jetsonClassIdFor(OBJECT type) {
    switch (type) {
        case OBJECT::BallBlue: return 0;
        case OBJECT::BallRed:  return 1;
        default: return -1;
    }
}

// Function to find the target object based on type and return its record
DETECTION_OBJECT findTarget(OBJECT type){
    DETECTION_OBJECT target;
    // 1) Pull the latest Jetson detections (camera + map coordinates)
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    int classId = jetsonClassIdFor(type);
    // Iterate through detected objects to find the closest target of the specified type
    for(int i = 0; i < local_map.detectionCount; i++) {
        double distance = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
        if (distance < lowestDist && classId >= 0 && local_map.detections[i].classID == classId) {
            target = local_map.detections[i];
            lowestDist = distance;
        }
    }
    return target;
}

// Function to drive to an object based on detection
void goToObject(OBJECT type){
    DETECTION_OBJECT target = findTarget(type);
    // If no target found, turn and try to find again
    if (target.mapLocation.x == 0 && target.mapLocation.y == 0){
        //Drivetrain.turnFor(45, rotationUnits::deg, 50, velocityUnits::pct);
        Drivetrain.turn(turnType::left);
        wait(2, sec);
        Drivetrain.stop();
        target = findTarget(type);
    }
    // Move to the detected target's position
    moveToPosition(target.mapLocation.x*100, target.mapLocation.y*100);
}

void runIntake(vex::directionType dir) {
    FirstStage.spin(dir);
    SecondStage.spin(dir);
}

void runIntake(vex::directionType dir, int rotations, bool driveForward = false) {
    FirstStage.spinFor(dir, rotations, vex::rotationUnits::rev, false);
    SecondStage.spinFor(dir, rotations, vex::rotationUnits::rev, !driveForward);
    if (driveForward)
        Drivetrain.driveFor(directionType::fwd, 70, vex::distanceUnits::cm, 40, velocityUnits::pct);
}

void stopIntake() {
    FirstStage.stop();
    SecondStage.stop();
}

void emergencyStop() {
    chassis.drive_with_voltage(0, 0);
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
}

void goToGoal() {
    int closestGoalX = 0;
    int closestGoalY = 0;
    int heading = 0;

    if (distanceTo(122, 0) < distanceTo(-122, 0)) {
        closestGoalX = 122;
        heading = 90;
    } else {
        closestGoalX = -122;
        heading = 270;
    }
    if (distanceTo(0, 122) < distanceTo(0, -122)) {
        closestGoalY = 122;
    } else {
        closestGoalY = -122;
    }

    moveToPosition(closestGoalX, closestGoalY, heading);
}

// Scored ball record for ranking pickup targets
struct ScoredBall {
    DETECTION_OBJECT det;
    double score;
    double dist_cm;
    double turn_deg;
};

static std::vector<ScoredBall> getScoredBalls(OBJECT type) {
    // 1) Pull the latest Jetson detections (camera + map coordinates)
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);

    // 2) Get robot pose from Jetson map (cm + heading deg)
    double robotX_cm = local_map.pos.x * 100.0;
    double robotY_cm = local_map.pos.y * 100.0;
    double robotHeading = local_map.pos.az;

    // 3) If Jetson pose is invalid, fall back to GPS
    if (std::isnan(robotX_cm) || std::isnan(robotY_cm) || std::isnan(robotHeading)) {
        robotX_cm = GPS.xPosition();
        robotY_cm = GPS.yPosition();
        robotHeading = GPS.heading();
    }

    std::vector<ScoredBall> candidates;
    candidates.reserve(local_map.detectionCount);

    // Score weights and filters
    const double turnWeight = 1.0;  // weight for turn difficulty vs distance
    const double minProb = 0.35;    // ignore very low-confidence detections

    int classId = jetsonClassIdFor(type);
    for (int i = 0; i < local_map.detectionCount; i++) {
        // Only keep detections that match the requested color and confidence
        const DETECTION_OBJECT& det = local_map.detections[i];
        if (classId < 0 || det.classID != classId) continue;
        if (det.probability < minProb) continue;

        // Convert ball position to cm (Jetson map is in meters)
        double ballX_cm = det.mapLocation.x * 100.0;
        double ballY_cm = det.mapLocation.y * 100.0;

        // Distance from robot to ball
        double dx = ballX_cm - robotX_cm;
        double dy = ballY_cm - robotY_cm;
        double dist_cm = sqrt(dx * dx + dy * dy);

        // Required turn angle to face the ball (deg)
        double targetAngleMath = atan2(dy, dx) * 180.0 / M_PI;
        double targetAngleNav = 90.0 - targetAngleMath;
        while (targetAngleNav < 0) targetAngleNav += 360.0;
        while (targetAngleNav >= 360.0) targetAngleNav -= 360.0;

        double angleErr = targetAngleNav - robotHeading;
        while (angleErr > 180.0) angleErr -= 360.0;
        while (angleErr < -180.0) angleErr += 360.0;

        double turn_deg = fabs(angleErr);
        // Score = distance + (turnWeight * turn angle)
        double score = dist_cm + (turnWeight * turn_deg);

        candidates.push_back({det, score, dist_cm, turn_deg});
    }

    // Sort by easiest (lowest score)
    std::sort(candidates.begin(), candidates.end(),
        [](const ScoredBall& a, const ScoredBall& b) { return a.score < b.score; });

    return candidates;
}

// Print top 3 easiest balls of the requested color on the controller screen
// Easiest = lowest score based on distance + turn angle needed
void printBestBalls(OBJECT type) {
    // 1) Get scored candidates (already sorted by easiest)
    auto candidates = getScoredBalls(type);

    // 2) Display top 3 on controller
    Controller.Screen.clearScreen();

    if (candidates.empty()) {
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("No balls found");
        cout << "No balls of requested color found\n";
        return;
    }

    int count = (candidates.size() > 3) ? 3 : (int)candidates.size();
    for (int i = 0; i < count; i++) {
        double x_cm = candidates[i].det.mapLocation.x * 100.0;
        double y_cm = candidates[i].det.mapLocation.y * 100.0;
        Controller.Screen.setCursor(i + 1, 1);
        Controller.Screen.print("Ball %d: X:%.0f Y:%.0f", i + 1, x_cm, y_cm);
        cout << "Ball " << (i + 1) << ": X=" << x_cm << " Y=" << y_cm
             << " dist=" << candidates[i].dist_cm
             << " turn=" << candidates[i].turn_deg
             << " score=" << candidates[i].score << "\n";
    }
}

// Navigate to and pick up the top 3 easiest balls using A* + Pure Pursuit
void collectBestBalls(OBJECT type) {
    // Get ranked list for navigation
    auto candidates = getScoredBalls(type);
    if (candidates.empty()) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("No balls found");
        cout << "No balls of requested color found\n";
        return;
    }

    // Print top 3 on controller
    Controller.Screen.clearScreen();
    int displayCount = (candidates.size() > 3) ? 3 : (int)candidates.size();
    for (int i = 0; i < displayCount; i++) {
        double x_cm = candidates[i].det.mapLocation.x * 100.0;
        double y_cm = candidates[i].det.mapLocation.y * 100.0;
        Controller.Screen.setCursor(i + 1, 1);
        Controller.Screen.print("Ball %d: X:%.0f Y:%.0f", i + 1, x_cm, y_cm);
        cout << "Ball " << (i + 1) << ": X=" << x_cm << " Y=" << y_cm
             << " dist=" << candidates[i].dist_cm
             << " turn=" << candidates[i].turn_deg
             << " score=" << candidates[i].score << "\n";
    }

    FieldMap fieldMap;
    fieldMap.populateStandardField();

    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5; // ~17.1 cm
    const double safety_margin_cm = 5.0;
    const double grid_resolution_cm = 15.0;

    int collectedCount = 0;
    for (size_t i = 0; i < candidates.size() && collectedCount < 3; i++) {
        double target_x = candidates[i].det.mapLocation.x * 100.0;
        double target_y = candidates[i].det.mapLocation.y * 100.0;

        // Skip balls inside obstacles
        if (fieldMap.isPointInObstacle(target_x, target_y)) {
            continue;
        }

        double curr_x = GPS.xPosition();
        double curr_y = GPS.yPosition();
        double curr_h = GPS.heading();

        if ((curr_x == 0.0 && curr_y == 0.0) || std::isnan(curr_x) || std::isnan(curr_y) || std::isnan(curr_h)) {
            continue;
        }

        std::vector<astar::Point> path = astar::findPath(
            fieldMap, curr_x, curr_y, target_x, target_y,
            grid_resolution_cm, robot_radius_cm, safety_margin_cm
        );

        if (path.empty()) {
            continue;
        }

        // Adjust path for pure pursuit (skip start cell, use exact target)
        std::vector<astar::Point> adjustedPath;
        if (path.size() <= 2) {
            adjustedPath.push_back({target_x, target_y});
        } else {
            for (size_t j = 1; j < path.size(); j++) {
                adjustedPath.push_back(path[j]);
            }
            adjustedPath.back() = {target_x, target_y};
        }

        // Follow path using pure pursuit
        bool success = purePursuitFollowPath(adjustedPath, 4.0f, 25.0f, 5.0f, -1.0f, true);
        if (!success) {
            continue;
        }

        // Intake to pick up the ball once reached
        runIntake(directionType::fwd);
        wait(800, msec);
        stopIntake();

        collectedCount++;
    }
}

// Find, announce, path plan, and collect the top 3 easiest balls
void collectBestBallsWithStatus(OBJECT type) {
    // Step 1: announce search and show top 3
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("finding top 3 balls");
    printBestBalls(type);

    // Give time to read the results
    wait(1000, msec);

    // Step 2: get ranked list (same ordering as printBestBalls/collectBestBalls)
    auto candidates = getScoredBalls(type);
    if (candidates.empty()) {
        return;
    }

    FieldMap fieldMap;
    fieldMap.populateStandardField();

    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5; // ~17.1 cm
    const double safety_margin_cm = 5.0;
    const double grid_resolution_cm = 15.0;

    int collectedCount = 0;
    for (size_t i = 0; i < candidates.size() && collectedCount < 3; i++) {
        double target_x = candidates[i].det.mapLocation.x * 100.0;
        double target_y = candidates[i].det.mapLocation.y * 100.0;

        // Skip balls inside obstacles
        if (fieldMap.isPointInObstacle(target_x, target_y)) {
            continue;
        }

        // Step 3: plan path
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("path planning with pure pursuit");

        double curr_x = GPS.xPosition();
        double curr_y = GPS.yPosition();
        double curr_h = GPS.heading();
        if ((curr_x == 0.0 && curr_y == 0.0) || std::isnan(curr_x) || std::isnan(curr_y) || std::isnan(curr_h)) {
            continue;
        }

        std::vector<astar::Point> path = astar::findPath(
            fieldMap, curr_x, curr_y, target_x, target_y,
            grid_resolution_cm, robot_radius_cm, safety_margin_cm
        );

        if (path.empty()) {
            continue;
        }

        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("path planned");
        wait(1000, msec);

        // Step 4: follow path to the ball
        std::vector<astar::Point> adjustedPath;
        if (path.size() <= 2) {
            adjustedPath.push_back({target_x, target_y});
        } else {
            for (size_t j = 1; j < path.size(); j++) {
                adjustedPath.push_back(path[j]);
            }
            adjustedPath.back() = {target_x, target_y};
        }

        bool success = purePursuitFollowPath(adjustedPath, 4.0f, 25.0f, 5.0f, -1.0f, true);
        if (!success) {
            continue;
        }

        // Step 5: run intake to collect
        runIntake(directionType::fwd);
        wait(800, msec);
        stopIntake();

        collectedCount++;

        //-----
        chassis.drive_with_voltage(0, 0);
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("ball %d collected", collectedCount);
        wait(1000, msec);
        //-----
    }
}

// Test A* path planning and waypoint following
void testPathPlanning() {
    Controller.Screen.clearScreen();
    Controller.Screen.print("A* Path Test");
    wait(200, msec);

    // Ensure GPS is calibrated before accepting a target
    static bool gpsCalibrated = false;
    if (!gpsCalibrated) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Calibrating GPS...");
        GPS.calibrate();
        waitUntil(!GPS.isCalibrating());
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("GPS Ready");
        wait(300, msec);
        gpsCalibrated = true;
    }
    
    // Initialize target coordinates (can be adjusted with arrows)
    double target_x = 0.0;  // cm
    double target_y = 0.0;   // cm
    
    // Allow user to adjust target coordinates dynamically
    bool coordinatesLocked = false;
    
    while (!coordinatesLocked) {
        // Display current target coordinates
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("X: %.1f", target_x);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Y: %.1f", target_y);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("A=Lock");
        
        // Up arrow: increase X
        if (Controller.ButtonUp.pressing()) {
            target_x += 1.0;
        }
        
        // Down arrow: decrease X
        if (Controller.ButtonDown.pressing()) {
            target_x -= 1.0;
        }
        
        // Left arrow: decrease Y
        if (Controller.ButtonLeft.pressing()) {
            target_y -= 1.0;
        }
        
        // Right arrow: increase Y
        if (Controller.ButtonRight.pressing()) {
            target_y += 1.0;
        }
        
        // Button A: lock coordinates and start pathfinding
        if (Controller.ButtonA.pressing()) {
            waitUntil(!Controller.ButtonA.pressing());
            coordinatesLocked = true;
            wait(200, msec);
        }
        
        wait(20, msec);
    }
    
    // Coordinates locked - now get current position and start pathfinding
    Controller.Screen.clearScreen();
    Controller.Screen.print("Getting GPS...");
    wait(300, msec);
    
    double curr_x = GPS.xPosition();
    double curr_y = GPS.yPosition();
    double curr_h = GPS.heading();
    
    if ((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y) || isnan(curr_h)) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Bad GPS - abort");
        wait(800, msec);
        return;
    }
    
    // Display current position and target
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Current: %.1f,%.1f", curr_x, curr_y);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Target: %.1f,%.1f", target_x, target_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Planning...");
    wait(500, msec);
    
    // Create and populate FieldMap with obstacles
    FieldMap fieldMap;
    fieldMap.populateStandardField();
    
    // Call A* to find path using actual robot geometry (13.5in x 13.5in)
    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5; // ~17.145 cm
    const double safety_margin_cm = 0.0;                        // tune 4–10 cm
    const double grid_resolution_cm = 60.96/2;                    // keep coarse grid as-is

    std::vector<astar::Point> path = astar::findPath(
        fieldMap,
        curr_x, curr_y,
        target_x, target_y,
        grid_resolution_cm,
        robot_radius_cm,
        safety_margin_cm
    );
    
    // Check if path was found
    if (path.empty()) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("No path found!");
        wait(2000, msec);
        return;
    }
    
    // Print entire path to console BEFORE robot moves
    cout << "===== A* PATH PLANNED =====\n";
    cout << "Start: (" << curr_x << "," << curr_y << ") H=" << curr_h << "\n";
    cout << "Target: (" << target_x << "," << target_y << ")\n";
    cout << "Total Waypoints: " << path.size() << "\n";
    cout << "Full Path:\n";
    for (size_t i = 0; i < path.size(); i++) {
        cout << "  WP" << i << ": (" << path[i].first << "," << path[i].second << ")\n";
    }
    cout << "===========================\n";
    
    // Display path info on controller in requested order
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Path planned");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("List all waypoints");
    wait(700, msec);
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Waypoints: %d", (int)path.size());
    wait(1000, msec);
    
    // Follow the path waypoint by waypoint
    for (size_t i = 0; i < path.size(); i++) {
        double wp_x = path[i].first;
        double wp_y = path[i].second;
        
        // Get current position
        double robot_x = GPS.xPosition();
        double robot_y = GPS.yPosition();
        double robot_h = GPS.heading();
        
        if ((robot_x == 0.0 && robot_y == 0.0) || isnan(robot_x) || isnan(robot_y)) {
            cout << "Bad GPS at WP" << i << ", breaking\n";
            break;
        }
        
        // Calculate bearing to waypoint
        double bearing = calculateBearing(robot_x, robot_y, wp_x, wp_y);
        double dx = wp_x - robot_x;
        double dy = wp_y - robot_y;
        double dist_cm = sqrt(dx*dx + dy*dy);
        
        // Skip if already at waypoint
        if (dist_cm < 5.0) {
            cout << "Waypoint " << (int)(i+1) << " (" << wp_x << "," << wp_y << ") reached\n";
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Waypoint %d/%d (%.1f, %.1f) reached", (int)(i+1), (int)path.size(), wp_x, wp_y);
            continue;
        }
        
        // Display waypoint info
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Moving to WP %d/%d", (int)(i+1), (int)path.size());
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Dist: %.1f cm", dist_cm);
        
        // Turn and drive to waypoint
        double dist_in = dist_cm / 2.54;
        chassis.set_heading(robot_h);
        chassis.turn_to_angle(bearing);
        //wait(20, msec);
        chassis.drive_distance(dist_in);
        //wait(20, msec);
        
        // Print waypoint reached to terminal and controller
        cout << "Waypoint " << (int)(i+1) << " (" << wp_x << "," << wp_y << ") reached\n";
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Waypoint %d/%d (%.1f, %.1f) reached", (int)(i+1), (int)path.size(), wp_x, wp_y);
    }
    
    // Final stop and report
    emergencyStop();
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);
    
    double final_x = GPS.xPosition();
    double final_y = GPS.yPosition();
    double final_h = GPS.heading();
    double err = sqrt(pow(final_x - target_x, 2) + pow(final_y - target_y, 2));
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Arrived!");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Pos: %.1f,%.1f", final_x, final_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Error: %.1f cm", err);
    
    wait(3000, msec);
}

// PID performance test: drives forward 6 times, 20 inches each
void pidtest() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("PID Test Starting...");
    wait(1000, msec);

    const double distance_in = 20.0; // 20 inches per segment
    const int segments = 6;

    Brain.Timer.reset();
    double last_time = 0.0;

    for (int i = 1; i <= segments; i++) {
        // Drive forward 20 inches
        chassis.drive_distance(distance_in);

        // Get elapsed time since last segment
        double current_time = Brain.Timer.time(msec) / 1000.0; // convert to seconds
        double segment_time = current_time - last_time;
        last_time = current_time;

        // Display on controller
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Segment %d/6", i);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Time: %.2f sec", segment_time);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("Total: %.2f sec", current_time);

        wait(1500, msec);
    }

    // Final summary
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Test Complete!");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Total: %.2f sec", last_time);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Avg: %.2f sec", last_time / segments);
    wait(3000, msec);
}

// Variablized turn PID tuning test
void turnpidtest() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Turn PID Tuning");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Starting...");
    wait(500, msec);

    // PID variables with default values of 0
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    
    const float kp_step = 0.005;
    const float ki_step = 0.005;
    const float kd_step = 0.005;
    
    bool tuning = true;
    
    while (tuning) {
        // Display current KP, KI, KD values
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("KP: %.3f", kp);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("KI: %.3f", ki);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("KD: %.3f", kd);
        
        // Up: increase KP
        if (Controller.ButtonUp.pressing()) {
            kp += kp_step;
            wait(50, msec);
        }
        
        // Down: decrease KP
        if (Controller.ButtonDown.pressing()) {
            kp -= kp_step;
            if (kp < 0) kp = 0;
            wait(50, msec);
        }
        
        // Right: increase KI
        if (Controller.ButtonRight.pressing()) {
            ki += ki_step;
            wait(50, msec);
        }
        
        // Left: decrease KI
        if (Controller.ButtonLeft.pressing()) {
            ki -= ki_step;
            if (ki < 0) ki = 0;
            wait(50, msec);
        }
        
        // X: increase KD
        if (Controller.ButtonX.pressing()) {
            kd += kd_step;
            wait(50, msec);
        }
        
        // B: decrease KD
        if (Controller.ButtonB.pressing()) {
            kd -= kd_step;
            if (kd < 0) kd = 0;
            wait(50, msec);
        }
        
        // A: lock values and turn 90 degrees
        if (Controller.ButtonA.pressing()) {
            wait(100, msec);
            while (Controller.ButtonA.pressing()) {
                wait(10, msec);
            }
            wait(100, msec);
            
            // Set new PID constants
            chassis.set_turn_constants(8, kp, ki, kd, 0);
            chassis.set_swing_constants(8, kp, ki, kd, 0);
            
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Turning 90 deg...");
            wait(500, msec);

            // Zero heading as baseline
            chassis.Gyro.setRotation(0, degrees);
            wait(100, msec);

            // Turn to 90 degrees from zero baseline
            chassis.turn_to_angle(90);

            // Measure final heading and compute error
            float final_heading = chassis.Gyro.heading();
            float error = 90.0f - final_heading;
            
            // Display results: KP, KI, KD and error
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("KP: %.3f KI: %.3f", kp, ki);
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("KD: %.3f", kd);
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("Error: %.2f deg", error);
            
            wait(3000, msec);
            
            tuning = false;
        }
        
        wait(50, msec);
    }
}

// Variablized drive PID tuning test
void drivepidtest() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Drive PID Tuning");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Starting...");
    wait(500, msec);

    // PID variables with default values of 0
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    
    const float kp_step = 0.010;
    const float ki_step = 0.010;
    const float kd_step = 0.010;
    
    bool tuning = true;
    
    while (tuning) {
        // Display current KP, KI, KD values
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("KP: %.3f", kp);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("KI: %.3f", ki);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("KD: %.3f", kd);
        
        // Up: increase KP
        if (Controller.ButtonUp.pressing()) {
            kp += kp_step;
            wait(50, msec);
        }
        
        // Down: decrease KP
        if (Controller.ButtonDown.pressing()) {
            kp -= kp_step;
            if (kp < 0) kp = 0;
            wait(50, msec);
        }
        
        // Right: increase KI
        if (Controller.ButtonRight.pressing()) {
            ki += ki_step;
            wait(50, msec);
        }
        
        // Left: decrease KI
        if (Controller.ButtonLeft.pressing()) {
            ki -= ki_step;
            if (ki < 0) ki = 0;
            wait(50, msec);
        }
        
        // X: increase KD
        if (Controller.ButtonX.pressing()) {
            kd += kd_step;
            wait(50, msec);
        }
        
        // B: decrease KD
        if (Controller.ButtonB.pressing()) {
            kd -= kd_step;
            if (kd < 0) kd = 0;
            wait(50, msec);
        }
        
        // A: lock values and drive 30 inches
        if (Controller.ButtonA.pressing()) {
            wait(100, msec);
            while (Controller.ButtonA.pressing()) {
                wait(10, msec);
            }
            wait(100, msec);
            
            // Set new PID constants for drive
            chassis.set_drive_constants(12, kp, ki, kd, 0);
            
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Driving 30 in...");
            wait(500, msec);
            
            // Get odometry start position
            double start_pos = GPS.xPosition(); // or use odom if available
            
            // Drive forward 30 inches
            chassis.drive_distance(30.0);
            
            // Wait for all motion to fully stop
            // wait(300, msec);
            
            // Get odometry end position
            double end_pos = GPS.xPosition();
            double actual_distance = fabs(end_pos - start_pos) * 0.394; // Convert cm to inches
            float error = 30.0f - actual_distance;
            
            // Display results: KP, KI, KD and error
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("KP: %.3f KI: %.3f", kp, ki);
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("KD: %.3f", kd);
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("Error: %.2f in", error);
            
            wait(3000, msec);
            
            tuning = false;
        }
        
        wait(50, msec);
    }
}

// ============================================================================
// PURE PURSUIT PATH FOLLOWER
// Uses motor encoders for position tracking, IMU for heading
// GPS only used for initial position
// ============================================================================

// Find the lookahead point on the path at approximately lookahead distance ahead
static std::pair<double, double> findLookaheadPoint(
    const std::vector<std::pair<double,double>>& path,
    double robotX, double robotY,
    float lookaheadDist,
    size_t& nearestIdx)
{
    // Find the closest point on path to robot
    double minDist = 1e9;
    size_t closestIdx = 0;
    for (size_t i = 0; i < path.size(); i++) {
        double dx = path[i].first - robotX;
        double dy = path[i].second - robotY;
        double d = sqrt(dx*dx + dy*dy);
        if (d < minDist) {
            minDist = d;
            closestIdx = i;
        }
    }
    nearestIdx = closestIdx;
    
    // Search forward from closest point to find lookahead intersection
    for (size_t i = closestIdx; i < path.size() - 1; i++) {
        double x1 = path[i].first;
        double y1 = path[i].second;
        double x2 = path[i+1].first;
        double y2 = path[i+1].second;
        
        // Vector from robot to segment start
        double dx = x1 - robotX;
        double dy = y1 - robotY;
        
        // Segment direction vector
        double fx = x2 - x1;
        double fy = y2 - y1;
        
        // Quadratic coefficients for circle-line intersection
        double a = fx*fx + fy*fy;
        double b = 2.0 * (dx*fx + dy*fy);
        double c = dx*dx + dy*dy - lookaheadDist*lookaheadDist;
        
        double discriminant = b*b - 4*a*c;
        
        if (discriminant >= 0 && a > 1e-6) {
            double sqrtDisc = sqrt(discriminant);
            double t = (-b + sqrtDisc) / (2*a);
            
            if (t >= 0.0 && t <= 1.0) {
                return {x1 + t*fx, y1 + t*fy};
            }
        }
    }
    
    // If no intersection found, return the last point on path
    return path.back();
}

// Convert navigation heading (0=north, CW positive) to math angle (0=east, CCW positive) in radians
static double navToMathRad(double navDeg) {
    return (90.0 - navDeg) * M_PI / 180.0;
}

// Pure pursuit path follower with sensor fusion (encoders + GPS)
// endHeading: final heading in degrees (0-360), or -1 to skip final turn
// useGPS: true = sensor fusion (GPS corrects encoder drift), false = encoders only
bool purePursuitFollowPath(const std::vector<std::pair<double,double>>& path,
                           float baseVelocity,
                           float lookaheadDist,
                           float endTolerance,
                           float endHeading,
                           bool useGPS)
{
    if (path.empty()) {
        return false;
    }
    
    // Robot parameters
    const int maxIterations = 3000;  // ~30 seconds at 10ms loop
    const float minVelocity = 2.0f;
    const float steeringDeadband = 1.5f;  // Small deadband to reduce wobble on straights
    
    // Speed ramping: fast for first 70%, slow for last 30%
    const float fastVelocity = 8.0f;    // Cruise speed (V)
    const float slowdownPoint = 0.7f;   // Start slowing at 70% progress
    const float minDistForFast = 60.0f * 2.54f;  // 60 inches in cm (~152cm) - below this, use slow speed only
    
    // Calculate total path distance for progress tracking
    double totalPathDist = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        double dx = path[i].first - path[i-1].first;
        double dy = path[i].second - path[i-1].second;
        totalPathDist += sqrt(dx*dx + dy*dy);
    }
    
    // For short paths, skip fast phase entirely
    bool useSlowOnly = (totalPathDist < minDistForFast);
    if (useSlowOnly) {
        cout << "Pure Pursuit: Short path (" << totalPathDist << "cm < " << minDistForFast << "cm), using slow speed only\n";
    }
    
    // Sensor fusion parameters
    const int gpsUpdateInterval = 10;   // Blend GPS every N iterations (100ms at 10ms loop)
    const float gpsBlendFactor = 0.3f;  // How much to trust GPS (0.3 = 30% GPS, 70% encoders)
    
    // Get initial position from GPS
    double robotX = GPS.xPosition();
    double robotY = GPS.yPosition();
    double initHeading = GPS.heading();
    double gpsQuality = GPS.quality();
    
    // Check for bad initial GPS
    if (isnan(robotX) || isnan(robotY) || isnan(initHeading)) {
        chassis.drive_with_voltage(0, 0);
        cout << "Pure Pursuit: Bad GPS at start (NaN values)\n";
        Controller.Screen.clearScreen();
        Controller.Screen.print("Bad GPS - NaN");
        return false;
    }
    
    // Warn if GPS quality is low (robot might be in corner or obstructed)
    if (gpsQuality < 90) {
        cout << "Pure Pursuit: WARNING - GPS quality low: " << gpsQuality << "%\n";
        Controller.Screen.clearScreen();
        Controller.Screen.print("GPS Quality: %.0f%%", gpsQuality);
        wait(500, msec);
    }
    
    // Sync chassis IMU heading with GPS heading at start
    chassis.set_heading(initHeading);
    task::sleep(20);  // Allow IMU to sync
    
    cout << "Pure Pursuit Start: X=" << robotX << " Y=" << robotY << " H=" << initHeading;
    cout << " Quality=" << gpsQuality << "% GPS=" << (useGPS ? "ON" : "OFF") << "\n";
    
    // Store initial encoder positions (inches)
    float lastLeftPos = chassis.get_left_position_in();
    float lastRightPos = chassis.get_right_position_in();
    
    // End point is the LAST point in path (should be actual target, not cell center)
    double endX = path.back().first;
    double endY = path.back().second;
    
    int iteration = 0;
    size_t nearestIdx = 0;
    bool singlePointPath = (path.size() == 1);
    
    // Stuck detection variables (using GPS, not encoders)
    // Made less aggressive to avoid false positives during slow turns
    int stuckCounter = 0;
    const int stuckThreshold = 100;      // ~1 second of no GPS movement = stuck
    const int stuckCheckInterval = 10;   // Check GPS every 10 iterations (100ms)
    double lastGpsX = robotX;
    double lastGpsY = robotY;
    const double stuckDistThreshold = 0.5;  // Must move at least 0.5cm per check (very lenient)
    
    while (iteration++ < maxIterations) {
        // Get heading from IMU (faster and smoother than GPS)
        double robotHeadingNav = chassis.get_absolute_heading();
        
        // Get current encoder positions
        float leftPos = chassis.get_left_position_in();
        float rightPos = chassis.get_right_position_in();
        
        // Calculate distance traveled since last iteration
        float deltaLeft = leftPos - lastLeftPos;
        float deltaRight = rightPos - lastRightPos;
        float distanceTraveled_in = (deltaLeft + deltaRight) / 2.0f;
        float distanceTraveled_cm = distanceTraveled_in * 2.54f;
        
        // Stuck detection: use GPS to see if robot is actually moving
        // (encoders can spin even if robot is stuck against wall)
        if (iteration % stuckCheckInterval == 0) {
            double currentGpsX = GPS.xPosition();
            double currentGpsY = GPS.yPosition();
            
            if (!isnan(currentGpsX) && !isnan(currentGpsY)) {
                double gpsDelta = sqrt(pow(currentGpsX - lastGpsX, 2) + pow(currentGpsY - lastGpsY, 2));
                
                if (gpsDelta < stuckDistThreshold) {
                    // GPS shows we haven't moved much
                    stuckCounter++;
                    if (stuckCounter >= (stuckThreshold / stuckCheckInterval)) {
                        chassis.drive_with_voltage(0, 0);
                        cout << "Pure Pursuit: STUCK - GPS shows no movement\n";
                        cout << "  Last pos: (" << lastGpsX << "," << lastGpsY << ")\n";
                        cout << "  Curr pos: (" << currentGpsX << "," << currentGpsY << ")\n";
                        Controller.Screen.clearScreen();
                        Controller.Screen.print("STUCK! GPS no move");
                        return false;
                    }
                } else {
                    stuckCounter = 0;  // Reset if GPS shows movement
                }
                
                lastGpsX = currentGpsX;
                lastGpsY = currentGpsY;
            }
        }
        
        // Update position using dead reckoning (encoder + IMU)
        double headingRad = navToMathRad(robotHeadingNav);
        robotX += distanceTraveled_cm * cos(headingRad);
        robotY += distanceTraveled_cm * sin(headingRad);
        
        // Detect if robot is turning (left/right wheels moving different amounts)
        float turnRate = fabs(deltaLeft - deltaRight);  // High = turning sharply
        bool isTurning = (turnRate > 0.05f);  // threshold in inches
        
        // Store for next iteration
        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        
        // Check distance to end for adaptive GPS blending
        double distToEnd = sqrt(pow(endX - robotX, 2) + pow(endY - robotY, 2));
        
        // Sensor fusion: periodically blend in GPS to correct encoder drift
        // BUT only when going straight - GPS lag hurts turning accuracy
        // Use MORE aggressive blending when close to target (reduce drift at end)
        if (useGPS && (iteration % gpsUpdateInterval == 0) && !isTurning) {
            double gpsX = GPS.xPosition();
            double gpsY = GPS.yPosition();
            
            // Only blend if GPS reading is valid
            if (!isnan(gpsX) && !isnan(gpsY)) {
                // Adaptive blend factor: trust GPS more when close to target
                float adaptiveBlend = gpsBlendFactor;
                if (distToEnd < 20.0) {
                    adaptiveBlend = 0.8f;  // 80% GPS when within 20cm
                } else if (distToEnd < 40.0) {
                    adaptiveBlend = 0.65f; // 65% GPS when within 40cm
                } else if (distToEnd < 70.0) {
                    adaptiveBlend = 0.5f;  // 50% GPS when within 70cm
                }
                
                // Weighted average: new_pos = blend*gps + (1-blend)*encoder
                robotX = adaptiveBlend * gpsX + (1.0 - adaptiveBlend) * robotX;
                robotY = adaptiveBlend * gpsY + (1.0 - adaptiveBlend) * robotY;
            }
        }
        if (distToEnd < endTolerance) {
            chassis.drive_with_voltage(0, 0);
            
            // Report final position from GPS
            task::sleep(100);
            double finalX = GPS.xPosition();
            double finalY = GPS.yPosition();
            double finalDist = sqrt(pow(endX - finalX, 2) + pow(endY - finalY, 2));
            cout << "Pure Pursuit: Arrived! GPS dist to target: " << finalDist << "cm\n";
            
            /* === GPS-BASED FINAL POSITION CORRECTION (DISABLED) ===
            // Uncomment this block if pure pursuit alone isn't precise enough
            // Continuous GPS feedback loop - drive while checking GPS
            const int maxCorrectionTime = 3000;  // 3 second timeout
            const float positionTolerance = 3.0f;  // Stop when within 3cm
            int correctionStart = Brain.Timer.system();
            int lastDebug = 0;
            
            while (Brain.Timer.system() - correctionStart < maxCorrectionTime) {
                double gpsX = GPS.xPosition();
                double gpsY = GPS.yPosition();
                double gpsH = GPS.heading();
                
                // Bad GPS reading - stop and exit
                if (isnan(gpsX) || isnan(gpsY) || isnan(gpsH)) {
                    cout << "  GPS correction: Bad reading, stopping\n";
                    break;
                }
                
                double gpsDist = sqrt(pow(endX - gpsX, 2) + pow(endY - gpsY, 2));
                
                // Debug every 200ms
                int elapsed = Brain.Timer.system() - correctionStart;
                if (elapsed - lastDebug >= 200) {
                    cout << "  GPS correction: dist=" << gpsDist << "cm, elapsed=" << elapsed << "ms\n";
                    lastDebug = elapsed;
                }
                
                // Success - within tolerance
                if (gpsDist < positionTolerance) {
                    cout << "  GPS correction: Target reached! dist=" << gpsDist << "cm\n";
                    break;
                }
                
                // Calculate direction to target
                double dx = endX - gpsX;
                double dy = endY - gpsY;
                double targetAngle = 90.0 - (atan2(dy, dx) * 180.0 / M_PI);
                while (targetAngle < 0) targetAngle += 360;
                while (targetAngle >= 360) targetAngle -= 360;
                
                // Angle error
                double angleErr = targetAngle - gpsH;
                while (angleErr > 180) angleErr -= 360;
                while (angleErr < -180) angleErr += 360;
                
                // Determine if we need to reverse (target is behind us)
                bool shouldReverse = fabs(angleErr) > 90.0;
                if (shouldReverse) {
                    // Flip angle error for reverse driving
                    angleErr = (angleErr > 0) ? angleErr - 180 : angleErr + 180;
                }
                
                // Proportional steering
                float turnCorr = 0.06f * (float)angleErr;
                turnCorr = fmax(-2.0f, fmin(2.0f, turnCorr));
                
                // Speed based on distance - very slow for precision
                float correctionVel;
                if (gpsDist > 10.0f) {
                    correctionVel = 2.0f;
                } else if (gpsDist > 5.0f) {
                    correctionVel = 1.5f;
                } else {
                    correctionVel = 1.2f;  // Minimum to overcome friction
                }
                if (shouldReverse) correctionVel = -correctionVel;
                
                // Apply drive
                float leftVel = correctionVel + turnCorr;
                float rightVel = correctionVel - turnCorr;
                chassis.drive_with_voltage(leftVel, rightVel);
                
                task::sleep(20);  // 50Hz update rate
            }
            chassis.drive_with_voltage(0, 0);
            
            // Final position report
            task::sleep(100);
            finalX = GPS.xPosition();
            finalY = GPS.yPosition();
            finalDist = sqrt(pow(endX - finalX, 2) + pow(endY - finalY, 2));
            cout << "  Final position error: " << finalDist << "cm\n";
            */ // END GPS POSITION CORRECTION
            
            // === GPS-BASED FINAL HEADING CORRECTION ===
            if (endHeading >= 0) {
                cout << "Pure Pursuit: GPS heading correction to " << endHeading << "\n";
                
                // Use GPS to turn precisely
                for (int hCorr = 0; hCorr < 20; hCorr++) {  // Max 20 iterations (~2 sec)
                    task::sleep(50);
                    double gpsH = GPS.heading();
                    if (isnan(gpsH)) break;
                    
                    double headingError = endHeading - gpsH;
                    while (headingError > 180) headingError -= 360;
                    while (headingError < -180) headingError += 360;
                    
                    // If within 2°, heading is good enough
                    if (fabs(headingError) < 2.0) {
                        cout << "  Heading achieved: " << gpsH << " (error: " << headingError << ")\n";
                        break;
                    }
                    
                    // Proportional turn
                    float turnVel = 0.06f * (float)headingError;
                    turnVel = fmax(-4.0f, fmin(4.0f, turnVel));
                    // Minimum voltage to move
                    if (fabs(turnVel) < 1.5f && fabs(turnVel) > 0.1f) {
                        turnVel = (turnVel > 0) ? 1.5f : -1.5f;
                    }
                    
                    chassis.drive_with_voltage(turnVel, -turnVel);
                }
                chassis.drive_with_voltage(0, 0);
                
                // Report final heading
                task::sleep(100);
                cout << "  Final GPS heading: " << GPS.heading() << "\n";
            }
            
            return true;
        }
        
        // Single point path OR final approach - USE GPS DIRECTLY for precision
        if (singlePointPath || (nearestIdx >= path.size() - 1 && distToEnd < lookaheadDist * 1.5)) {
            // Switch to GPS position for final approach (encoders drift)
            double gpsX = GPS.xPosition();
            double gpsY = GPS.yPosition();
            double gpsH = GPS.heading();
            
            // Use GPS if valid, otherwise fall back to encoder estimate
            double approachX = (!isnan(gpsX)) ? gpsX : robotX;
            double approachY = (!isnan(gpsY)) ? gpsY : robotY;
            double approachH = (!isnan(gpsH)) ? gpsH : robotHeadingNav;
            
            double dx = endX - approachX;
            double dy = endY - approachY;
            double gpsDist = sqrt(dx*dx + dy*dy);
            
            // Calculate target angle in navigation coords
            double targetAngleMath = atan2(dy, dx);
            double targetAngleNav = 90.0 - (targetAngleMath * 180.0 / M_PI);
            while (targetAngleNav < 0) targetAngleNav += 360.0;
            while (targetAngleNav >= 360) targetAngleNav -= 360.0;
            
            double angleError = targetAngleNav - approachH;
            while (angleError > 180) angleError -= 360;
            while (angleError < -180) angleError += 360;
            
            // Debug
            if (iteration % 50 == 0) {
                cout << "PP Final(GPS): dist=" << gpsDist << " err=" << angleError << "\n";
            }
            
            // Check if we're actually at target using GPS (tighter tolerance)
            if (gpsDist < endTolerance * 0.5f) {
                // We're really close per GPS - check encoder estimate agrees
                if (distToEnd < endTolerance * 1.5f) {
                    chassis.drive_with_voltage(0, 0);
                    cout << "Pure Pursuit: Target reached! GPS dist=" << gpsDist << "cm\n";
                    
                    // Do heading correction inline if needed
                    if (endHeading >= 0) {
                        cout << "Pure Pursuit: GPS heading correction to " << endHeading << "\n";
                        for (int hCorr = 0; hCorr < 20; hCorr++) {
                            task::sleep(50);
                            double hGps = GPS.heading();
                            if (isnan(hGps)) break;
                            
                            double hErr = endHeading - hGps;
                            while (hErr > 180) hErr -= 360;
                            while (hErr < -180) hErr += 360;
                            
                            if (fabs(hErr) < 2.0) {
                                cout << "  Heading achieved: " << hGps << "\n";
                                break;
                            }
                            
                            float tVel = 0.06f * (float)hErr;
                            tVel = fmax(-4.0f, fmin(4.0f, tVel));
                            if (fabs(tVel) < 1.5f && fabs(tVel) > 0.1f) {
                                tVel = (tVel > 0) ? 1.5f : -1.5f;
                            }
                            chassis.drive_with_voltage(tVel, -tVel);
                        }
                        chassis.drive_with_voltage(0, 0);
                        cout << "  Final GPS heading: " << GPS.heading() << "\n";
                    }
                    return true;
                }
            }
            
            float absErr = fabs((float)angleError);
            float turnGain = 0.12f;  // Same aggressive gain as main loop
            float turnOutput = turnGain * (float)angleError;
            turnOutput = fmax(-baseVelocity, fmin(baseVelocity, turnOutput));
            
            // Progressive slowdown based on GPS distance - very slow when close
            float speedFactor;
            if (gpsDist < 5.0f) {
                speedFactor = 0.2f;  // Crawl for final 5cm
            } else if (gpsDist < 10.0f) {
                speedFactor = 0.3f;  // Very slow for 5-10cm
            } else if (gpsDist < 15.0f) {
                speedFactor = 0.4f;  // Slow for 10-15cm
            } else if (gpsDist < 25.0f) {
                speedFactor = 0.6f;  // Medium for 15-25cm
            } else {
                speedFactor = 0.8f;  // Approaching
            }
            
            // Also slow down for sharp turns
            if (absErr > 30.0f) speedFactor *= 0.4f;
            else if (absErr > 15.0f) speedFactor *= 0.6f;
            
            float driveVel = baseVelocity * speedFactor;
            if (driveVel < minVelocity) driveVel = minVelocity;
            
            // Positive error = turn right = left faster
            float leftVel = driveVel + turnOutput;
            float rightVel = driveVel - turnOutput;
            
            chassis.drive_with_voltage(leftVel, rightVel);
            task::sleep(10);
            continue;
        }
        
        // Calculate progress through path (0.0 to 1.0)
        float progress = (totalPathDist > 0) ? (float)(1.0 - distToEnd / totalPathDist) : 1.0f;
        progress = fmax(0.0f, fmin(1.0f, progress));  // Clamp to [0, 1]
        
        // Determine if we're in "fast" phase (only if path is long enough and < 70% progress)
        bool inFastPhase = !useSlowOnly && (progress < slowdownPoint);
        
        // Select velocity based on progress: fast for first 70%, slow for last 30%
        float currentVelocity = inFastPhase ? fastVelocity : baseVelocity;
        
        // Dynamic lookahead: larger when fast (see turns earlier), smaller when slow (tighter tracking)
        float dynamicLookahead = inFastPhase ? lookaheadDist * 1.5f : lookaheadDist;
        
        // Find lookahead point with dynamic distance
        auto lookahead = findLookaheadPoint(path, robotX, robotY, dynamicLookahead, nearestIdx);
        double lookaheadX = lookahead.first;
        double lookaheadY = lookahead.second;
        
        // Calculate angle to lookahead point (in math coords: 0=east, CCW positive)
        double dx = lookaheadX - robotX;
        double dy = lookaheadY - robotY;
        double targetAngleMath = atan2(dy, dx);  // radians, math convention
        
        // Convert target angle to navigation (0=north, CW positive) for comparison with IMU
        double targetAngleNav = 90.0 - (targetAngleMath * 180.0 / M_PI);
        // Normalize to [0, 360)
        while (targetAngleNav < 0) targetAngleNav += 360.0;
        while (targetAngleNav >= 360) targetAngleNav -= 360.0;
        
        // Calculate angle error in navigation coords
        double angleError = targetAngleNav - robotHeadingNav;
        // Normalize to [-180, 180]
        while (angleError > 180) angleError -= 360;
        while (angleError < -180) angleError += 360;
        
        // Debug output every 50 iterations
        if (iteration % 50 == 0) {
            cout << "PP: pos=(" << robotX << "," << robotY << ") h=" << robotHeadingNav 
                 << " progress=" << (int)(progress*100) << "% vel=" << currentVelocity << "V"
                 << " LA=" << dynamicLookahead << " err=" << angleError << "\n";
        }
        
        // Steering - more aggressive at high speed
        float absError = fabs((float)angleError);
        float turnOutput = 0.0f;
        
        // Only apply small deadband to reduce wobble on straight sections
        if (absError > steeringDeadband) {
            // Scale turn gain with velocity: higher speed = more aggressive turns
            float turnGain = inFastPhase ? 0.15f : 0.12f;  // More aggressive when fast
            turnOutput = turnGain * (float)angleError;
            
            // Allow stronger turn output
            float maxTurn = currentVelocity * 1.0f;  // Can use full voltage differential
            turnOutput = fmax(-maxTurn, fmin(maxTurn, turnOutput));
        }
        
        // Slow down more when turning sharply (gives time to turn before hitting things)
        float speedFactor = (absError > 30.0f) ? 0.25f : (absError > 15.0f) ? 0.5f : 0.8f;
        float driveVel = currentVelocity * speedFactor;
        
        // Apply differential steering
        // Positive turnOutput (turn right) -> left faster, right slower
        float leftVel = driveVel + turnOutput;
        float rightVel = driveVel - turnOutput;
        
        // Minimum voltage to overcome friction
        if (fabs(leftVel) < minVelocity && fabs(leftVel) > 0.1f) {
            leftVel = (leftVel > 0) ? minVelocity : -minVelocity;
        }
        if (fabs(rightVel) < minVelocity && fabs(rightVel) > 0.1f) {
            rightVel = (rightVel > 0) ? minVelocity : -minVelocity;
        }
        
        // Clamp to max voltage
        leftVel = fmax(-12.0f, fmin(12.0f, leftVel));
        rightVel = fmax(-12.0f, fmin(12.0f, rightVel));
        
        chassis.drive_with_voltage(leftVel, rightVel);
        task::sleep(10);
    }
    
    // Timeout
    chassis.drive_with_voltage(0, 0);
    cout << "Pure Pursuit: Timeout\n";
    return false;
}

// Test function: A* planning + pure pursuit following
void testPurePursuit() {
    Controller.Screen.clearScreen();
    Controller.Screen.print("Pure Pursuit Test");
    wait(200, msec);
    
    // Initialize target coordinates
    double target_x = 0.0;
    double target_y = 0.0;
    
    // Allow user to adjust target coordinates
    bool coordinatesLocked = false;
    
    while (!coordinatesLocked) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("X: %.1f", target_x);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Y: %.1f", target_y);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("A=Lock B=Cancel");
        
        if (Controller.ButtonUp.pressing()) {
            target_x += 10.0;
        }
        if (Controller.ButtonDown.pressing()) {
            target_x -= 10.0;
        }
        if (Controller.ButtonLeft.pressing()) {
            target_y -= 10.0;
        }
        if (Controller.ButtonRight.pressing()) {
            target_y += 10.0;
        }
        if (Controller.ButtonA.pressing()) {
            waitUntil(!Controller.ButtonA.pressing());
            coordinatesLocked = true;
            wait(200, msec);
        }
        if (Controller.ButtonB.pressing()) {
            waitUntil(!Controller.ButtonB.pressing());
            return;
        }
        
        wait(20, msec);
    }
    
    // Get current position
    Controller.Screen.clearScreen();
    Controller.Screen.print("Getting GPS...");
    wait(300, msec);
    
    double curr_x = GPS.xPosition();
    double curr_y = GPS.yPosition();
    double curr_h = GPS.heading();
    
    if ((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y) || isnan(curr_h)) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Bad GPS - abort");
        wait(800, msec);
        return;
    }
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("From: %.0f,%.0f", curr_x, curr_y);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("To: %.0f,%.0f", target_x, target_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Planning...");
    wait(500, msec);
    
    // Create field map and plan path
    FieldMap fieldMap;
    fieldMap.populateStandardField();
    
    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5;
    const double safety_margin_cm = 5.0;            // Keep path 5cm away from obstacles
    const double grid_resolution_cm = 15.0;         // Smaller cells = more waypoints = smoother turns
    
    std::vector<astar::Point> path = astar::findPath(
        fieldMap,
        curr_x, curr_y,
        target_x, target_y,
        grid_resolution_cm,
        robot_radius_cm,
        safety_margin_cm
    );
    
    // If no path found, robot might be inside obstacle - back up until clear
    if (path.empty()) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("No path - backing up");
        cout << "No path found - robot may be in obstacle, backing up...\n";
        
        const int maxBackupTime = 3000;  // 3 seconds max
        const float backupVel = -2.5f;   // Slow reverse
        int backupStart = Brain.Timer.system();
        
        while (Brain.Timer.system() - backupStart < maxBackupTime) {
            // Drive backwards slowly
            chassis.drive_with_voltage(backupVel, backupVel);
            task::sleep(50);
            
            // Check GPS position
            double gpsX = GPS.xPosition();
            double gpsY = GPS.yPosition();
            
            if (isnan(gpsX) || isnan(gpsY)) continue;
            
            // Check if we're now outside all obstacles
            if (!fieldMap.isPointInObstacle(gpsX, gpsY)) {
                // Also check with robot radius margin
                bool clearOfObstacles = true;
                for (const auto& obs : fieldMap.getObstacles()) {
                    // Check if robot center + radius would still hit obstacle
                    double dx = gpsX - obs.cx;
                    double dy = gpsY - obs.cy;
                    double dist = sqrt(dx*dx + dy*dy);
                    if (dist < robot_radius_cm + 10.0) {  // 10cm extra margin
                        clearOfObstacles = false;
                        break;
                    }
                }
                
                if (clearOfObstacles) {
                    chassis.drive_with_voltage(0, 0);
                    cout << "Cleared obstacle at (" << gpsX << "," << gpsY << "), retrying path...\n";
                    Controller.Screen.setCursor(2, 1);
                    Controller.Screen.print("Clear! Retrying...");
                    task::sleep(200);
                    
                    // Retry path planning from new position
                    path = astar::findPath(
                        fieldMap,
                        gpsX, gpsY,
                        target_x, target_y,
                        grid_resolution_cm,
                        robot_radius_cm,
                        safety_margin_cm
                    );
                    
                    // Update current position for path following
                    curr_x = gpsX;
                    curr_y = gpsY;
                    curr_h = GPS.heading();
                    break;
                }
            }
            
            // Debug every 500ms
            if ((Brain.Timer.system() - backupStart) % 500 < 50) {
                cout << "  Backing up... GPS: (" << gpsX << "," << gpsY << ")\n";
            }
        }
        chassis.drive_with_voltage(0, 0);
        
        // If still no path after backup, give up
        if (path.empty()) {
            Controller.Screen.clearScreen();
            Controller.Screen.print("Still no path!");
            cout << "Failed to find path even after backup\n";
            wait(2000, msec);
            return;
        }
    }
    
    // Skip the first waypoint (it's the cell we're starting in)
    // and add the actual target as the final point
    std::vector<astar::Point> adjustedPath;
    
    // If path is very short (start and end in same or adjacent cells), 
    // just go directly to target
    if (path.size() <= 2) {
        adjustedPath.push_back({target_x, target_y});
    } else {
        // Skip first waypoint, keep intermediate ones
        for (size_t i = 1; i < path.size(); i++) {
            adjustedPath.push_back(path[i]);
        }
        // Replace last waypoint (cell center) with actual target
        adjustedPath.back() = {target_x, target_y};
    }
    
    // Log path
    cout << "===== PURE PURSUIT PATH =====\n";
    cout << "Start: (" << curr_x << "," << curr_y << ") H=" << curr_h << "\n";
    cout << "Target: (" << target_x << "," << target_y << ")\n";
    cout << "Original A* waypoints: " << path.size() << "\n";
    cout << "Adjusted waypoints: " << adjustedPath.size() << "\n";
    for (size_t i = 0; i < adjustedPath.size(); i++) {
        cout << "  WP" << i << ": (" << adjustedPath[i].first << "," << adjustedPath[i].second << ")\n";
    }
    cout << "=============================\n";
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Following path...");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("WPs: %d", (int)adjustedPath.size());
    
    // Follow path with pure pursuit + GPS sensor fusion
    // Lookahead MUST be >= grid resolution to see upcoming turns!
    // Grid = 15cm, so lookahead = 25cm to see ~2 waypoints ahead
    // endHeading: -1 = no final turn, or specify 0-360 for final heading
    // useGPS: true = sensor fusion (encoders + GPS correction)
    // endTolerance reduced to 3cm - pure pursuit should land precisely
    bool success = purePursuitFollowPath(adjustedPath, 4.0f, 25.0f, 3.0f, -1.0f, true);
    
    // === TIMEOUT RECOVERY: Reverse and re-plan ===
    if (!success) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Timeout - Reversing");
        cout << "Pure Pursuit TIMEOUT - initiating recovery...\n";
        
        // Stop motors
        chassis.drive_with_voltage(0, 0);
        wait(200, msec);
        
        // Reverse for 1 second at slow speed
        const float reverseVoltage = -3.0f;  // Slow reverse
        const int reverseDuration = 1000;    // 1 second
        
        cout << "  Reversing for " << reverseDuration << "ms...\n";
        chassis.drive_with_voltage(reverseVoltage, reverseVoltage);
        wait(reverseDuration, msec);
        chassis.drive_with_voltage(0, 0);
        wait(300, msec);
        
        // Get new position after reversing
        double new_x = GPS.xPosition();
        double new_y = GPS.yPosition();
        double new_h = GPS.heading();
        
        if (isnan(new_x) || isnan(new_y) || isnan(new_h)) {
            Controller.Screen.clearScreen();
            Controller.Screen.print("Bad GPS after reverse");
            cout << "  Recovery failed: Bad GPS\n";
            wait(2000, msec);
            return;
        }
        
        cout << "  New position after reverse: (" << new_x << "," << new_y << ") H=" << new_h << "\n";
        
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Re-planning path...");
        
        // Re-plan path from new position to same target
        std::vector<astar::Point> newPath = astar::findPath(
            fieldMap,
            new_x, new_y,
            target_x, target_y,
            grid_resolution_cm,
            robot_radius_cm,
            safety_margin_cm
        );
        
        if (newPath.empty()) {
            Controller.Screen.clearScreen();
            Controller.Screen.print("No path after retry");
            cout << "  Recovery failed: No path found\n";
            wait(2000, msec);
            return;
        }
        
        // Adjust path (skip first waypoint, use actual target as end)
        std::vector<astar::Point> newAdjustedPath;
        if (newPath.size() <= 2) {
            newAdjustedPath.push_back({target_x, target_y});
        } else {
            for (size_t i = 1; i < newPath.size(); i++) {
                newAdjustedPath.push_back(newPath[i]);
            }
            newAdjustedPath.back() = {target_x, target_y};
        }
        
        cout << "  New path has " << newAdjustedPath.size() << " waypoints\n";
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("Retry: %d WPs", (int)newAdjustedPath.size());
        wait(500, msec);
        
        // Retry with new path
        Controller.Screen.clearScreen();
        Controller.Screen.print("Following new path...");
        success = purePursuitFollowPath(newAdjustedPath, 4.0f, 25.0f, 3.0f, -1.0f, true);
        
        cout << "  Recovery attempt " << (success ? "SUCCEEDED" : "FAILED") << "\n";
    }
    // === END TIMEOUT RECOVERY ===
    
    /* === ORIGINAL TIMEOUT HANDLING (COMMENTED OUT) ===
    // Stop and report
    chassis.drive_with_voltage(0, 0);
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);
    
    // Final position from GPS (most accurate)
    double final_x = GPS.xPosition();
    double final_y = GPS.yPosition();
    double final_h = GPS.heading();
    double err = sqrt(pow(final_x - target_x, 2) + pow(final_y - target_y, 2));
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print(success ? "Success!" : "Timeout/Error");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Pos: %.1f,%.1f H:%.0f", final_x, final_y, final_h);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Error: %.1f cm", err);
    
    cout << "Pure Pursuit " << (success ? "SUCCESS" : "FAILED") << "\n";
    cout << "Final GPS: (" << final_x << "," << final_y << ") H=" << final_h << " Error: " << err << " cm\n";
    
    wait(3000, msec);
    */ // END ORIGINAL TIMEOUT HANDLING
    
    // Stop and report final results
    chassis.drive_with_voltage(0, 0);
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);
    
    // Final position from GPS (most accurate)
    double final_x = GPS.xPosition();
    double final_y = GPS.yPosition();
    double final_h = GPS.heading();
    double err = sqrt(pow(final_x - target_x, 2) + pow(final_y - target_y, 2));
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print(success ? "Success!" : "Failed");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Pos: %.1f,%.1f H:%.0f", final_x, final_y, final_h);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Error: %.1f cm", err);
    
    cout << "Pure Pursuit " << (success ? "SUCCESS" : "FAILED") << "\n";
    cout << "Final GPS: (" << final_x << "," << final_y << ") H=" << final_h << " Error: " << err << " cm\n";
    
    wait(3000, msec);
}