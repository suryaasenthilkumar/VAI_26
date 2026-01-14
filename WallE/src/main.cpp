/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "ai_functions.h"

using namespace vex;

brain Brain;
controller Controller;
// Robot configuration code.
gps LGPS = gps(PORT14, 12.5, -11, distanceUnits::cm, 270);
gps RGPS = gps(PORT19, -12.5, -11, distanceUnits::cm, 90);
// DualGPS GPS = DualGPS(LGPS, RGPS, Inertial, vex::distanceUnits::cm);
// optical IntakeSensor = optical(PORT14);
// optical OutSensor = optical(PORT6);
// optical MiddleSensor = optical(PORT7);
// Left Drive
motor LeftDriveA = motor(PORT1, ratio6_1, true);
motor LeftDriveB = motor(PORT2, ratio6_1, true);
motor LeftDriveC = motor(PORT11, ratio6_1, true);
motor_group LeftDrive = motor_group(LeftDriveA, LeftDriveB, LeftDriveC);
// Right Drive
motor RightDriveA = motor(PORT9, ratio6_1, true);
motor RightDriveB = motor(PORT10, ratio6_1, false);
motor RightDriveC = motor(PORT20, ratio6_1, false);
motor_group RightDrive = motor_group(RightDriveA, RightDriveB, RightDriveC);

// Intake and Belt motors
motor Intake = motor(PORT8, ratio18_1, false);
motor Belt = motor(PORT12, ratio18_1, false);

// Inertial sensor for smartdrive (same port as chassis uses)
inertial Inertial = inertial(PORT6);

// Create smartdrive object for VEX built-in drivetrain methods
// Parameters: leftMotorGroup, rightMotorGroup, inertialSensor, wheelTravel, trackWidth, wheelBase, units, externalGearRatio
// wheelTravel = π * diameter = π * 3.25 ≈ 10.21 inches
// trackWidth = 12.0 inches (distance between left and right wheels)
// wheelBase = 0.0 for tank drive (distance between front and back wheels, not applicable for tank)
smartdrive Drivetrain = smartdrive(LeftDrive, RightDrive, Inertial, 13.5, 13.5, 0.0, distanceUnits::in, 0.75);

// A global instance of competition
competition Competition;

// create instance of jetson class to receive location and other
// data from the Jetson nano
//
ai::jetson  jetson_comms;

Drive chassis(

  //Pick your drive setup from the list below:
  //ZERO_TRACKER_NO_ODOM
  //ZERO_TRACKER_ODOM
  //TANK_ONE_FORWARD_ENCODER
  //TANK_ONE_FORWARD_ROTATION
  //TANK_ONE_SIDEWAYS_ENCODER
  //TANK_ONE_SIDEWAYS_ROTATION
  //TANK_TWO_ENCODER
  //TANK_TWO_ROTATION
  //HOLONOMIC_TWO_ENCODER
  //HOLONOMIC_TWO_ROTATION
  //
  //Write it here:
  //ZERO_TRACKER_NO_ODOM,
  TANK_ONE_FORWARD_ROTATION,
  
  //Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
  //You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".
  
  //Left Motors:
  motor_group(LeftDriveA, LeftDriveB, LeftDriveC),
  
  //Right Motors:
  motor_group(RightDriveA, RightDriveB, RightDriveC),
  
  //Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
  PORT6,
  
  //Input your wheel diameter. (4" omnis are actually closer to 4.125"):
  3.25,
  
  //External ratio, must be in decimal, in the format of input teeth/output teeth.
  //If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
  //If the motor drives the wheel directly, this value is 1:
  0.75,
  
  //Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
  //For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
  360,
  
  /*---------------------------------------------------------------------------*/
  /*                                  PAUSE!                                   */
  /*                                                                           */
  /*  The rest of the drive constructor is for robots using POSITION TRACKING. */
  /*  If you are not using position tracking, leave the rest of the values as  */
  /*  they are.                                                                */
  /*---------------------------------------------------------------------------*/
  
  //If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.
  
  //FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
  //LF:      //RF:    
  PORT1,     -PORT2,
  
  //LB:      //RB: 
  PORT3,     -PORT4,
  
  //If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
  //If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
  //If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
  0,
  
  //Input the Forward Tracker diameter (reverse it to make the direction switch):
  0,
  
  //Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
  //For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
  //This distance is in inches:
  0,
  
  //Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
  0,
  
  //Sideways tracker diameter (reverse to make the direction switch):
  0,
  
  //Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
  0
  
);
DualGPS GPS = DualGPS(LGPS, RGPS, chassis.Gyro, vex::distanceUnits::cm);

// Configure chassis PID/voltage limits with optimized turn tuning
void configureChassis(){
  chassis.set_drive_constants(12, 0.45, 0.0, 0.02, 0);
  chassis.set_heading_constants(8, 0.40, 0.0, 0.02, 0);
  // Optimized turn PID: Kp=0.090, Ki=0.120, Kd=0.260
  chassis.set_turn_constants(8, 0.090, 0.120, 0.260, 0);
  chassis.set_swing_constants(8, 0.090, 0.120, 0.260, 0);

  chassis.set_drive_exit_conditions(1.5, 200, 2500);
  chassis.set_turn_exit_conditions(3.0, 300, 2000);
  chassis.set_swing_exit_conditions(3.0, 300, 2000);
}
  
/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT15, "robot_32456_1", linkType::manager );
#else
#pragma message("building for the worker")
ai::robot_link       link( PORT13, "robot_32456_1", linkType::worker );
#endif

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auto_Isolation(void) {
  // Calibrate GPS Sensor
  GPS.calibrate();
  // Optional wait to allow for calibration
  waitUntil(!(GPS.isCalibrating()));

  goToObject(OBJECT::BallBlue);
  runIntake(directionType::fwd, 3, true);
  goToGoal();
  Drivetrain.driveFor(directionType::rev, 115, distanceUnits::cm);
  Belt.setVelocity(70, pct);
  runIntake(directionType::fwd, 5, false);
  // Back off from the goal
  Drivetrain.driveFor(directionType::fwd, 30, distanceUnits::cm);

}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void auto_Interaction(void) {
  // Add functions for interaction phase
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;

void autonomousMain(void) {
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................

  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();

  firstAutoFlag = false;
}

void driverControl(void) {
  static bool calibrated = false;
  if(!calibrated) {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Calibrating GPS...");
    GPS.calibrate();
    waitUntil(!GPS.isCalibrating());
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Ready!");
    calibrated = true;
  }
  while (true) {
    chassis.control_arcade();
    if(Controller.ButtonDown.pressing()){
      waitUntil(!Controller.ButtonDown.pressing());
      testPathPlanning();
      wait(500, msec);
    }
  }
}

int main() {

  // Ensure PID constants are initialized before any autonomous/path calls
  configureChassis();

  // local storage for latest data from the Jetson Nano
  static AI_RECORD local_map;

  // Run at about 15Hz
  int32_t loop_time = 33;

  // start the status update display
  thread t1(dashboardTask);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomousMain);
  Competition.drivercontrol(driverControl);

  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  //FILE *fp = fopen("/dev/serial2","wb");
  this_thread::sleep_for(loop_time);

  Intake.setVelocity(30, percent);
  Belt.setVelocity(30, percent);

  while(1) {
      // get last map data
      jetson_comms.get_data( &local_map );

      // set our location to be sent to partner robot
      link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );

      // fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az)

      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();

      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}