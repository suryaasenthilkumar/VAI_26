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
using namespace vex;
using namespace std;


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

// Function to find the target object based on type and return its record
DETECTION_OBJECT findTarget(OBJECT type){
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    // Iterate through detected objects to find the closest target of the specified type
    for(int i = 0; i < local_map.detectionCount; i++) {
        double distance = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
        if (distance < lowestDist && local_map.detections[i].classID == (int) type) {
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
    Intake.spin(dir);
    Belt.spin(dir);
}

void runIntake(vex::directionType dir, int rotations, bool driveForward = false) {
    Intake.spinFor(dir, rotations, vex::rotationUnits::rev, false);
    Belt.spinFor(dir, rotations, vex::rotationUnits::rev, !driveForward);
    if (driveForward)
        Drivetrain.driveFor(directionType::fwd, 70, vex::distanceUnits::cm, 40, velocityUnits::pct);
}

void stopIntake() {
    Intake.stop();
    Belt.stop();
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

void emergencyStop() {
    chassis.drive_with_voltage(0, 0);
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
}

// Test A* path planning and waypoint following
void testPathPlanning() {
    Controller.Screen.clearScreen();
    Controller.Screen.print("A* Path Test");
    wait(200, msec);
    
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