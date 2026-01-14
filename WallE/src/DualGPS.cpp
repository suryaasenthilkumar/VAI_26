#include "robot-config.h"
#include "DualGPS.h"

DualGPS::DualGPS(gps &Lgps, gps &Rgps, inertial &imu, vex::distanceUnits units)
: Left_GPS(Lgps),
  Right_GPS(Rgps), 
  IMU(imu),
  current_timestamp(Brain.Timer.system()),
  Units(units),
  x(0.0f), y(0.0f), abs_heading(0.0f),
  Vx(0.0f), Vy(0.0f), total_quality(0.0f),
  Timestamp(0)
{
    //fprintf(fp,"\rIniciando thread\n");
    update_thread = new vex::thread(updateThread, static_cast<void*>(this));
}

DualGPS::~DualGPS()
{
    // Signal thread to terminate
    if (update_thread != nullptr) 
    {
        update_thread->interrupt();
        delete update_thread;
        update_thread = nullptr;
    }
}

int DualGPS::updateThread(void* arg) 
{
    DualGPS* instance = static_cast<DualGPS*>(arg);
    while (true) 
    {
        instance->updatePosition();
        vex::this_thread::sleep_for(16);
    }
}


void DualGPS::updatePosition()
{
    float LeftX = Left_GPS.xPosition(Units), 
    LeftY = Left_GPS.yPosition(Units), 
    LeftH = Left_GPS.heading();
    float RightX = Right_GPS.xPosition(Units), 
    RightY = Right_GPS.yPosition(Units), 
    RightH = Right_GPS.heading();

    int LeftQual = Left_GPS.quality();
    int RightQual = Right_GPS.quality();

    double LeftWeight = normalizeQuality(LeftQual);
    double RightWeight = normalizeQuality(RightQual);
    double totalWeight = LeftWeight + RightWeight;
    uint32_t new_timestamp = Left_GPS.timestamp();
    //fprintf(fp,"\r Variables terminadas \n");
    if (totalWeight > 0)
    {   
       
        
        double new_x = ((LeftX * LeftWeight) + (RightX * RightWeight) )/ totalWeight;
        double new_y = (LeftY * LeftWeight + RightY * RightWeight) / totalWeight;
        double new_heading = (LeftH * LeftWeight + RightH * RightWeight) / totalWeight;
        double new_Vx = 0 ;
        double new_Vy = 0;
        //fprintf(fp,"\r x %.2f new %.2f \n", ((LeftX * LeftWeight) + (RightX * RightWeight) ), new_x);
        // Calculate GPS-based velocity
        float deltaTime = (new_timestamp - current_timestamp)/ 1000.0; // Convert to 
        //fprintf(fp,"\r Calculando tiempo delta \n");
        if (deltaTime > 0)
        {
            new_Vx = (new_x - this->x) / deltaTime;
            new_Vy = (new_y- this->y) / deltaTime;
        }
        //fprintf(fp,"\r Nuevos valores para newVel \n");
        {
            data_mutex.lock();
            x = new_x;
            y = new_y;
            abs_heading = new_heading;
            Vx = new_Vx;
            Vy = new_Vy;
            total_quality = totalWeight;
            Timestamp = new_timestamp;
            
        }
        //fprintf(fp,"\r Obtenido new \n");
        current_timestamp = new_timestamp;

        if (RightQual >= 99 && LeftQual >= 99)
        {
          IMU.setHeading(new_heading, degrees);
        }
        data_mutex.unlock();
        //fprintf(fp,"\r Desbloqueando data \n");
    }
    data_mutex.unlock();
    //fprintf(fp,"\r Total w es menor a 0 o hilo terminado\n");
}

float DualGPS::normalizeQuality(int quality) 
{
    return (quality >= 90) ? (quality - 90.0) / 10.0 : 0.0;
}

double DualGPS::xPosition()
{

    data_mutex.lock();
    double data = x;
    data_mutex.unlock();
    return data;
}

double DualGPS::yPosition()
{
    data_mutex.lock();
    double data = y;
    data_mutex.unlock();
    return data;

}

double DualGPS::heading()
{
    data_mutex.lock();
    double data = abs_heading;
    data_mutex.unlock();
    return data;
}

double DualGPS::xVelocity()
{
    data_mutex.lock();
    double data = Vx;
    data_mutex.unlock();
    return data;
}

double DualGPS::yVelocity()
{
    data_mutex.lock();
    double data = Vy;
    data_mutex.unlock();
    return data;
}

double DualGPS::quality()
{
    data_mutex.lock();
    double data = total_quality;
    data_mutex.unlock();
    return data;

}

uint32_t DualGPS::timestamp()
{
    data_mutex.lock();
    uint32_t data = Timestamp;
    data_mutex.unlock();
    return data;
}

void DualGPS::calibrate() {
    Left_GPS.calibrate();
    Right_GPS.calibrate();
}

bool DualGPS::isCalibrating() {
    return Left_GPS.isCalibrating() || Right_GPS.isCalibrating();
}





