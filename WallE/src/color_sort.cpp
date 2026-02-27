/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       color_sort.cpp                                           */
/*    Author:       Robolabs                                                 */
/*    Created:      Jan 25 2026                                              */
/*    Description:  Color sorting functionality                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robot-config.h"
#include "ai_functions.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// Team color - change to BallRed or BallBlue based on your team
const OBJECT TEAMCOLOR = BallBlue;

// Simple ball counter class
class BallCounter {
private:
  int maxBalls;
  int currentBalls;
public:
  BallCounter(int max) : maxBalls(max), currentBalls(0) {}
  bool countBall() {
    if (currentBalls < maxBalls) {
      currentBalls++;
      return true;
    }
    return false;
  }
  bool removeBall() {
    if (currentBalls > 0) {
      currentBalls--;
      return true;
    }
    return false;
  }
  int balls() { return currentBalls; }
};

BallCounter counter(6);
vex::timer ColorSortTimer;

// called when bottom optical *detects* a new object
void onBottomDetected() {
  // attempt to add a ball
  Brain.Screen.print("Bottom detected");
  int hue = OpticalBottom.hue();
  OBJECT detected = BallUndefined;

    // RED: 350–359 and 0–10
  if ((hue >= 300 && hue <= 359) ||
      (hue >= 0   && hue <= 50)) {
    detected = BallRed;
  }

  // BLUE: 210–240
  if (hue >= 160 && hue <= 240) {
    detected = BallBlue;
  }

  if (TEAMCOLOR != detected) {
    ColorSort.set(true);
    wait(150, msec);
  } else {
    if (counter.countBall()) {
      Brain.Screen.printAt(10, 50, "Ball added! Count: %d", counter.balls());
    } else {
      Brain.Screen.printAt(10, 50, "Count full (%d)", counter.balls());
    }
  }

  ColorSort.set(false);
}

// called when top optical *detects* a new object being sorted out
void onTopDetected() {
    // Brain.Screen.print("Top detected");

  int hue = OpticalTop.hue();
  // bool shouldSort = false;
  OBJECT detected = BallUndefined;

  if ((hue >= 300 && hue <= 359) ||
      (hue >= 0   && hue <= 50)) {
    // shouldSort = true;
    detected = BallRed;
  }
  // BLUE: 210–240
  if (hue >= 60 && hue <= 260) {
    detected = BallBlue;
  }
  
  Brain.Screen.printAt(10, 90, "Detected hue: %d Color: %d", hue, detected);

  if (TEAMCOLOR != detected) {
    ColorSortTimer.event([](){
        // stop intake after 1 second if still detecting object
        Score.spin(reverse,12000,voltageUnits::mV);
        wait(500, msec);
        Score.stop(hold);
    }, 200);
      // attempt to remove a ball
    if (counter.removeBall()) {
      Brain.Screen.printAt(10, 70, "Ball removed! Count: %d", counter.balls());
    }
  } else { // if theres ball of same color stop the intake and hold top ball
    Score.stop(hold);
    ColorSortTimer.event([](){
      if (OpticalTop.isNearObject()) {
        // stop intake after 1 second if still detecting object
        Score.spinFor(forward, 1, rev, false);
        Belt.spinFor(reverse, 1, rev, false);
        // Belt.stop(hold);
      } 
    }, 500);
    // Belt.stop(hold);
  }
}