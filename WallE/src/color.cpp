#include "robot-config.h"
#include "ai_functions.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

vex::timer ColorSortTimer;

directionType thirdStageDefaultDir = forward;
bool thirdStageOverrideActive = false;

int detectBallFromHue(int hue) {
  if ((hue >= 300 && hue <= 359) || (hue >= 0 && hue <= 50)) {
    return BallRed;
  }
  if (hue >= 140 && hue <= 278) {
    return BallBlue;
  }
  return BallUndefined;
}

enum ColorState {
  COLOR_IDLE = 0,
  COLOR_EJECTING,
  COLOR_RESET
};

int colorState = COLOR_IDLE;
int colorTimer = 0;

int onBottomDetectedThread() {
  // Optical setup (run once)
  OpticalBottom.setLightPower(100, percent);
  OpticalBottom1.setLightPower(100, percent);
  OpticalBottom.setLight(ledState::on);
  OpticalBottom1.setLight(ledState::on);
  ColorSortTimer.reset();
  while (true) { 
      if ( OpticalBottom.isNearObject() || 
                        OpticalBottom1.isNearObject()) {
        int hue0 = OpticalBottom.hue();
        int hue1 = OpticalBottom1.hue();
        int avgHue = (hue0 + hue1) / 2;
        Brain.Screen.clearScreen();
        int detected = BallUndefined;

        // Classify independently
        int d0 = OpticalBottom.isNearObject() ? detectBallFromHue(hue0) : BallUndefined;
        int d1 = OpticalBottom1.isNearObject() ? detectBallFromHue(hue1) : BallUndefined;

        if (d0 == d1) {
          detected = d0;
        } else if (d0 == BallBlue || d1 == BallBlue) {
          // favors ball blue since default color is red
          detected = BallBlue;
        } else if (d0 != BallUndefined) {
          detected = d0;
        } else if (d1 != BallUndefined) {
          detected = d1;
        } else {
          detected = detectBallFromHue(avgHue);
        }
        Brain.Screen.printAt(10, 60, "Detected Color: %d", detected);
        
        // Wrong color → eject
        if (detected != TEAMCOLOR) {
          colorState = COLOR_EJECTING;
        } else {
          colorState = COLOR_IDLE;
        }
      } 

    switch (colorState) {
      case COLOR_IDLE: {
        Brain.Screen.printAt(10, 80, "State: Color Idle");
        ColorSort.set(false);
        break;
      }

      // ───────────────────────────────
      case COLOR_EJECTING: {
          Brain.Screen.printAt(10, 80, "State: Color Eject");
          ColorSort.set(true);
          colorTimer = ColorSortTimer.time();
          colorState = COLOR_RESET;
          break;
      }

      // ───────────────────────────────
      case COLOR_RESET: {
        Brain.Screen.printAt(10, 80, "State: Color Reset");

        int currentTime = ColorSortTimer.time();
        if (currentTime - colorTimer > 100) {
          ColorSort.set(false);
          colorTimer = 0;
          colorState = COLOR_IDLE;
        }
        break;
      }
    }

    // Let auton & other tasks run
    this_thread::sleep_for(15);
  }       
  return 0;
}

int topState = 0;
int topColorTimer = 0;

int onTopDetectedThread() {
  // Setup once
  OpticalTop.setLightPower(100, percent);
  OpticalTop.setLight(ledState::on);

  while (true) {
    switch (topState) {

      // ───────────────────────────────
      case COLOR_IDLE: {
        thirdStageOverrideActive = false;
        Brain.Screen.printAt(10, 110, "Top State: IDLE");

        if (OpticalTop.isNearObject()) {
          int hue = OpticalTop.hue();
          int detected = detectBallFromHue(hue);

          Brain.Screen.printAt(
            10, 130,
            "Top Hue: %d Color: %d",
            hue, detected
          );

          // Wrong color → eject
          if (detected != TEAMCOLOR) {
            topState = COLOR_EJECTING;
          }
        }
        break;
      }

      // ───────────────────────────────
      case COLOR_EJECTING: {
        Brain.Screen.printAt(10, 110, "Top State: EJECT");
        thirdStageOverrideActive = true;
        // Reverse third stage briefly
        if (thirdStageDefaultDir == forward) {
          ThirdStage.spin(reverse, 12000, voltageUnits::mV);
        } else {
          ThirdStage.spin(forward, 12000, voltageUnits::mV);
        }
        colorTimer = ColorSortTimer.time();
        topState = COLOR_RESET;
        break;
      }

      // ───────────────────────────────
      case COLOR_RESET: {
        Brain.Screen.printAt(10, 110, "Top State: RESET");
        int currentTime = ColorSortTimer.time();
        if (currentTime > colorTimer + 700) {
          // Resume normal direction
          ThirdStage.spin(thirdStageDefaultDir, 12000, voltageUnits::mV);
          topState = COLOR_IDLE;
        }
        break;
      }
    }
    this_thread::sleep_for(20);
  }

  return 0;
}
