#ifndef ROBOT_NAVIGATION_H
#define ROBOT_NAVIGATION_H

#include <cmath>

#include "vex.h"

using namespace vex;

class DualGPS
{
public:
    DualGPS(gps &Lgps, gps &Rgps, inertial &imu, vex::distanceUnits units);
    ~DualGPS();

    void calibrate();
    bool isCalibrating();

    double xPosition();
    double yPosition();
    double heading();
    double xVelocity();
    double yVelocity();
    double quality();
    uint32_t timestamp();

    gps &Left_GPS;
    gps &Right_GPS;
    inertial &IMU;


private:
    static int updateThread(void* arg);
    vex::thread* update_thread;
    mutable vex::mutex data_mutex;
    unsigned long current_timestamp;
    void updatePosition();
    float normalizeQuality(int quality);
    vex::distanceUnits Units;

    double x;
    double y;
    double abs_heading;
    double Vx;
    double Vy;
    double total_quality;
    uint32_t Timestamp;

    
};

#endif // ROBOT_NAVIGATION_H