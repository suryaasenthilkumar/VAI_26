/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Robolabs                                                     */
/*    Created:      Sun Jan 11 2026                                           */
/*    Description:  VEX AI                                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "ai_functions.h"
#include "astar.h"
#include "field_map.h"
#include <cmath>
#include <cstdlib>
#include <vector>

using namespace vex;

brain Brain; 
controller Controller;

// Sensors
gps LGPS = gps(PORT9, -13.5, -5.5, distanceUnits::cm, 270);
gps RGPS = gps(PORT10, 13.5, -5.5, distanceUnits::cm, 90);
// DualGPS GPS = DualGPS(LGPS, RGPS, Inertial, vex::distanceUnits::cm);
//Sensors
optical OpticalTop = optical(PORT4);
optical OpticalBottom = optical(PORT1);
optical OpticalBottom1 = optical(PORT6);
// Inertial sensor for smartdrive (same port as chassis uses)
inertial Inertial = inertial(PORT17);

// Left Drive
motor LeftDriveA = motor(PORT16, ratio6_1, true);
motor LeftDriveB = motor(PORT15, ratio6_1, true);
motor LeftDriveC = motor(PORT14, ratio6_1, true);
motor_group LeftDrive = motor_group(LeftDriveA, LeftDriveB, LeftDriveC);
// Right Drive
motor RightDriveA = motor(PORT20, ratio6_1, false);
motor RightDriveB = motor(PORT19, ratio6_1, false);
motor RightDriveC = motor(PORT18, ratio6_1, false);
motor_group RightDrive = motor_group(RightDriveA, RightDriveB, RightDriveC);

// Intake motors
motor FirstStage = motor(PORT8, ratio6_1, false); // forward intake
motor SecondStage = motor(PORT11, ratio6_1, false); // forward outake / up
motor ThirdStage = motor(PORT12, ratio6_1, false); // forward outake / up
motor ZeroStage = motor(PORT13, ratio6_1, false); // forward up


// Create smartdrive object for VEX built-in drivetrain methods
// Parameters: leftMotorGroup, rightMotorGroup, inertialSensor, wheelTravel, trackWidth, wheelBase, units, externalGearRatio
// wheelTravel = π * diameter = π * 3.25 ≈ 10.21 inches
// trackWidth = 12.0 inches (distance between left and right wheels)
// wheelBase = 0.0 for tank drive (distance between front and back wheels, not applicable for tank)
smartdrive Drivetrain = smartdrive(LeftDrive, RightDrive, Inertial, 13.5, 13.5, 0.0, distanceUnits::in, 0.75);

// Pneumatics
digital_out MatchLoader = digital_out(Brain.ThreeWirePort.A);
digital_out Expansion = digital_out(Brain.ThreeWirePort.B);
digital_out ColorSort = digital_out(Brain.ThreeWirePort.C);
digital_out Stopper = digital_out(Brain.ThreeWirePort.D);



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
  PORT17,
  
  //Input your wheel diameter. (4" omnis are actually closer to 4.125"):
  3.25,
  
  //External ratio, must be in decimal, in the format of input teeth/output teeth.
  //If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
  //If the motor drives the wheel directly, this value is 1:
  0.75,
  
  //Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
  //For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
  356.4,
  
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
  0,     0,
  
  //LB:      //RB: 
  0,     0,
  
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

// Configure chassis PID/voltage limits with optimized tuning
void configureChassis(){
  // Set drive motors to brake mode instead of coast
  // LeftDrive.setStopping(brake);
  // RightDrive.setStopping(brake);
  
  // Optimized drive PID: Kp=0.550, Ki=0.040, Kd=0.570
  chassis.set_drive_constants(12, 0.550, 0.040, 0.570, 0);
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
ai::robot_link       link( PORT21, "robot_32456_1", linkType::manager );
#else
#pragma message("building for the worker")
ai::robot_link       link( PORT10, "robot_32456_1", linkType::worker );
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
  SecondStage.setVelocity(70, pct);
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
// Track autonomous start time for controller timer display
static int g_autonStartMs = -1;
static bool g_phase2Signal = false;
static int remainingAutonMs() {
  if (g_autonStartMs < 0) return 0;
  return 105000 - (Brain.Timer.system() - g_autonStartMs);
}

// Simple bearing helper (math coords -> navigation heading)
static double bearingDegrees(double fromX, double fromY, double toX, double toY) {
  double dx = toX - fromX;
  double dy = toY - fromY;
  double rad = atan2(dy, dx);
  double deg = rad * 180.0 / M_PI;
  // convert to navigation: 0° = north, clockwise positive
  double nav = fmod(90.0 - deg, 360.0);
  if (nav < 0) nav += 360.0;
  return nav;
}

// Path plan and follow using A* + Pure Pursuit (cm coordinates), modeled after testPurePursuit
bool planAndFollowPathCm(double targetX_cm, double targetY_cm, float finalHeading, bool /*useSmooth*/) {
  // Abort Phase 1 immediately at 20s remaining
  if (g_autonStartMs >= 0 && remainingAutonMs() <= 20000 && !g_phase2Signal) {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Going to Park");
    Controller.rumble("---");
    g_phase2Signal = true;
    chassis.drive_with_voltage(0, 0);
    return false;
  }

  double curr_x = GPS.xPosition();
  double curr_y = GPS.yPosition();
  double curr_h = GPS.heading();

  if ((curr_x == 0.0 && curr_y == 0.0) || std::isnan(curr_x) || std::isnan(curr_y) || std::isnan(curr_h)) {
    return false;
  }

  FieldMap fieldMap;
  fieldMap.populateStandardField();

  const double robot_width_in = 13.5;
  const double robot_radius_cm = robot_width_in * 2.54 * 0.5; // ~17.1 cm
  const double safety_margin_cm = 5.0;                        // keep 5cm away from obstacles
  const double grid_resolution_cm = 30.0;                     // smoother waypoints

  std::vector<astar::Point> path = astar::findPath(
      fieldMap, curr_x, curr_y, targetX_cm, targetY_cm,
      grid_resolution_cm, robot_radius_cm, safety_margin_cm);

  if (path.empty()) {
    return false;
  }

  // Prepare path for pure pursuit: skip first waypoint (current cell), use actual target as end
  std::vector<astar::Point> adjustedPath;
  if (path.size() <= 2) {
    adjustedPath.push_back({targetX_cm, targetY_cm});
  } else {
    for (size_t i = 1; i < path.size(); i++) {
      adjustedPath.push_back(path[i]);
    }
    adjustedPath.back() = {targetX_cm, targetY_cm};
  }

  // Follow path using pure pursuit
  const float baseVelocity = 6.0f;   // cruise speed
  const float lookaheadDist = 20.0f; // lookahead distance in cm
  const float endTolerance = 5.0f;   // arrival tolerance in cm
  const bool useGPS = true;          // sensor fusion for accuracy

  return purePursuitFollowPath(adjustedPath, baseVelocity, lookaheadDist,
                               endTolerance, finalHeading, useGPS);
}

// ---------------------------------------------------------------------------
// Phase helpers for 105s autonomous: Phase 1 random roaming, Phase 2 to (-120,0)
// ---------------------------------------------------------------------------

static void showTimeRemainingMs(int remainingMs) {
  int secs = remainingMs / 1000;
  if (secs < 0) secs = 0;
  int mm = secs / 60;
  int ss = secs % 60;
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Time %02d:%02d   ", mm, ss);
}

static bool isSafeRandomTarget(double x, double y) {
  // Keep targets within field bounds and away from the center obstacle
  if (fabs(x) > 170 || fabs(y) > 170) return false;
  if (fabs(x) + fabs(y) < 80) return false;
  return true;
}

static bool pickRandomTarget(double &x, double &y) {
  static bool seeded = false;
  if (!seeded) {
    srand(static_cast<unsigned>(Brain.Timer.system()));
    seeded = true;
  }
  for (int i = 0; i < 20; i++) {
    double rx = -170 + (rand() % 341);  // [-170,170]
    double ry = -170 + (rand() % 341);
    if (isSafeRandomTarget(rx, ry)) {
      x = rx;
      y = ry;
      return true;
    }
  }
  return false;
}

static void runPhase1(int phaseMillis) {
  int phaseStart = Brain.Timer.system();
  
  // Get starting position
  double startX = GPS.xPosition();
  double startY = GPS.yPosition();
  
  // Waypoint loop: start -> (120,120) -> (-120,-120) -> (120,120) -> (-120,-120) -> ...
  std::vector<std::pair<double, double>> waypoints = {
    {startX, startY},
    {120.0, 120.0},
    {-120.0, -120.0}
  };
  
  int wpIdx = 0;
  
  while (Brain.Timer.system() - phaseStart < phaseMillis) {
    // If timer hits 20s, signal and break immediately
    if (remainingAutonMs() <= 20000 && !g_phase2Signal) {
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Killed Phase 1 Task");
      Controller.Screen.setCursor(2, 1);
      Controller.Screen.print("Going to Park");
      Controller.rumble("---");
      g_phase2Signal = true;
      break;
    }
    // Update controller timer for total 105s run
    if (g_autonStartMs >= 0) {
      int remaining = 105000 - (Brain.Timer.system() - g_autonStartMs);
      showTimeRemainingMs(remaining);
    }
    
    // Get current waypoint
    double tx = waypoints[wpIdx].first;
    double ty = waypoints[wpIdx].second;

    int moveStart = Brain.Timer.system();
    bool ok = planAndFollowPathCm(tx, ty, -1, true);

    // Bail if the move fails or runs long to keep phase responsive
    if (!ok || Brain.Timer.system() - moveStart > 8000) {
      chassis.drive_with_voltage(0, 0);
    }
    
    // Move to next waypoint: start -> 150/150, then alternate between 150/150 and -150/-150
    if (wpIdx == 0) {
      wpIdx = 1;  // After start, go to (150,150)
    } else {
      wpIdx = (wpIdx == 1) ? 2 : 1;  // Alternate between (150,150) and (-150,-150)
    }
  }
}

static void runPhase2() {
  planAndFollowPathCm(-120.0, 0.0, -1, true);
}

/*
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
*/

// New autonomous: 105s total. Phase 1 (80s) waypoint loop, Phase 2 (last ~25s) drive to (-120,0).
void autonomousMain(void) {
  const int totalMs = 105000;
  const int phase1Ms = 80000;
  int start = Brain.Timer.system();
  g_autonStartMs = start;
  g_phase2Signal = false;

  runPhase1(phase1Ms);
  // Show timer immediately after Phase 1 completes or aborts
  showTimeRemainingMs(totalMs - (Brain.Timer.system() - g_autonStartMs));

  if (Brain.Timer.system() - start < totalMs - 5000) { // leave buffer for endgame
    runPhase2();
  }
  g_autonStartMs = -1;
}

// Task wrapper to run autonomous from driver control
int runAutonTask() {
  autonomousMain();
  return 0;
}

void driverControl(void) {
  static bool calibrated = false;
  static bool pendingAuton = false;
  if(!calibrated) {
    // Capture early A-press so it runs right after calibration
    if (Controller.ButtonA.pressing()) {
      pendingAuton = true;
    }
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Calibrating GPS...");
    GPS.calibrate();
    waitUntil(!GPS.isCalibrating());
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Ready!");
    calibrated = true;

    // If A was pressed during calibration, run autonomous immediately
    if (pendingAuton) {
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Auton: 105s");
      runAutonTask();
      wait(500, msec);
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Complete");
      wait(500, msec);
      pendingAuton = false;
    }
  }
  while (true) {
    // Button A: start phase-based autonomous
    if(Controller.ButtonA.pressing()){
      waitUntil(!Controller.ButtonA.pressing());
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Auton: 105s");
      runAutonTask();
      wait(500, msec);
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Complete");
      wait(500, msec);
    }

    chassis.control_arcade();
    if(Controller.ButtonDown.pressing()){
      waitUntil(!Controller.ButtonDown.pressing());
      turnpidtest();
      wait(500, msec);
    }
    if(Controller.ButtonUp.pressing()){
      waitUntil(!Controller.ButtonUp.pressing());
      pidtest();
      wait(500, msec);
    }
    if(Controller.ButtonL1.pressing()){
      waitUntil(!Controller.ButtonL1.pressing());
      testPathPlanning();
      wait(500, msec);
    }
    if(Controller.ButtonL2.pressing()){
      waitUntil(!Controller.ButtonL2.pressing());
      drivepidtest();
      wait(500, msec);
    }
    if(Controller.ButtonY.pressing()){
      waitUntil(!Controller.ButtonY.pressing());
      testPurePursuit();
      wait(500, msec);
    }
    if(Controller.ButtonX.pressing()){
      waitUntil(!Controller.ButtonX.pressing());
      collectBestBalls(OBJECT::BallBlue);
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
  // Competition.autonomous(autonomousMain);
  // Competition.drivercontrol(driverControl);

  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  //FILE *fp = fopen("/dev/serial2","wb");
  this_thread::sleep_for(loop_time);

  // FirstStage.setVelocity(30, percent);
  // SecondStage.setVelocity(30, percent);

  float x_mock = 250.0f;
  float y_mock = 250.0f;
  float az_mock = 90.0f;
  int32_t status_mock = 1.0f;

  while(1) {
      // get last map data
      jetson_comms.get_data( &local_map );

      // mock data sent, replace with actual location data from jetson
      link.set_remote_location(x_mock, y_mock, az_mock, status_mock);

      // set our location to be sent to partner robot
      // link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );

      printf("%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az);

      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();

      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}