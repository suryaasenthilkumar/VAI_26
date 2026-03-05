#include "robot-config.h"
// intakes until full
// only bottom sort run
void intakeBalls() {
  Stopper.set(true);
  if (OpticalBottom1.isNearObject() || OpticalBottom.isNearObject()) {
    ZeroStage.spin(forward, 5, percent);
  } else {
    ZeroStage.spin(forward, 100, percent);
  }
  // responsible for intaking balls correctly
  FirstStage.spin(forward, 100, percent);
  if (OpticalTop.isNearObject()) {
    SecondStage.stop(coast);
    ThirdStage.stop(coast);
  } else {
    SecondStage.spin(forward, 100, percent);
    ThirdStage.spin(forward,100,percent);
  }
}

void outakeBallsBottom() {
  ColorSort.set(false);
  ZeroStage.spin(reverse, 100, percent);
  FirstStage.spin(reverse, 100, percent);
  SecondStage.spin(reverse, 100, percent);
  ThirdStage.spin(reverse, 100, percent);
}

// sets piston blocking balls up and outakes until number of balls reached
// top sort and bottom sort run
void outakeBallsTop(vex::directionType direction) {
  Stopper.set(false);
  if (OpticalBottom1.isNearObject() || OpticalBottom.isNearObject()) {
    ZeroStage.spin(forward, 5, percent);
  } else {
    ZeroStage.spin(forward, 100, percent);
  }
  FirstStage.spin(forward, 100, percent);
  SecondStage.spin(forward, 100, percent);
  thirdStageDefaultDir = direction;
  if (!thirdStageOverrideActive) {
    ThirdStage.spin(thirdStageDefaultDir, 12000, voltageUnits::mV);
  }
}

// void stopIntake() {
//   ZeroStage.stop(coast);
//   FirstStage.stop(brake);
//   SecondStage.stop(brake);
//   ThirdStage.stop(brake);
// }