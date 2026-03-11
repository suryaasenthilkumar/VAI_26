/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.h                                              */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Header for AI robot movement functions                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <robot-config.h>
#include <vector>
#include <utility>

enum OBJECT {
    BallUndefined,
    BallBlue,
    BallRed
};

using namespace vex;

// Sets default PID constants for the chassis
void default_constants();

// Calculates the distance to a given target (x, y)
double distanceTo(double target_x, double target_y);

// Calculates the bearing angle from current position to target position
double calculateBearing(double currX, double currY, double targetX, double targetY);

// Moves the robot to a specified position and orientation
void moveToPosition(double target_x, double target_y, double target_theta);

// Finds a target object based on the specified type
DETECTION_OBJECT findTarget(OBJECT type);

// Drives to the closest specified object
void goToObject(OBJECT type);

// Turns the robot to a specific angle with given tolerance and speed
void turnTo(double angle, int tolerance, int speed);

// Drives the robot in a specified heading for a given distance and speed
void driveFor(int heading, double distance, int speed);

// void runIntake(vex::directionType dir);
// void runIntake(vex::directionType dir, int rotations, bool driveForward);

// intake balls until full
void intakeBalls();

// outakes balls on middle bottom goal
void outakeBallsBottom();

// outakes balls, reverse == middle top, forward == long goal top
void outakeBallsTop(directionType dir);

void stopIntake();

void goToGoal();

// Print top 3 easiest balls of requested color on controller
void printBestBalls(OBJECT type);

// Navigate to and pick up the top 3 easiest balls using pure pursuit
void collectBestBalls(OBJECT type);

// Find, announce, path plan, and collect the top 3 easiest balls
void collectBestBallsWithStatus(OBJECT type);

void emergencyStop();

// Tests the path planning algorithm
void testPathPlanning();

// PID performance test: 6 consecutive 20-inch drives with timing
void pidtest();

// Variablized turn PID tuning test
void turnpidtest();

// Variablized drive PID tuning test
void drivepidtest();

// Pure pursuit path following algorithm
bool purePursuitFollowPath(const std::vector<std::pair<double,double>>& path,
                           float baseVelocity,
                           float lookaheadDist,
                           float endTolerance,
                           float endHeading,
                           bool useGPS);

// Test function for pure pursuit algorithm
void testPurePursuit();