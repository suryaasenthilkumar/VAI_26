#include "vex.h"
#include "DualGPS.h"
#include "JAR-Template/odom.h"
#include "JAR-Template/drive.h"

using namespace vex;

extern brain Brain;
extern controller Controller;

extern gps LGPS;
extern gps RGPS;
extern DualGPS GPS;
extern optical IntakeSensor;
extern optical OutSensor;
extern optical MiddleSensor;
// Left Drive
extern motor LeftDriveA;
extern motor LeftDriveB;
extern motor LeftDriveC;
extern motor_group LeftDrive;
// Right Drive
extern motor RightDriveA;
extern motor RightDriveB;
extern motor RightDriveC;
extern motor_group RightDrive;

// Intake and Belt motors
extern motor Intake;
extern motor Belt;

// Inertial sensor for smartdrive
extern inertial Inertial;

// Smartdrive object for built-in VEX drivetrain methods
extern smartdrive Drivetrain;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
