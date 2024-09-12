#pragma once
#include <Wire.h>
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class TurnSensor{
    private:
    uint32_t turnAngle;
    int16_t turnRate;
    int16_t gyroOffset;
    uint16_t gryroLastUpdate;
    IMU imu;

    public:
    TurnSensor();
    void turnSensorReset();
    void turnSensorUpdate();
    void turnSensorSetup(); 
    uint32_t getTurnAngle();
};