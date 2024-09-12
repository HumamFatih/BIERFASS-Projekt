#ifndef DRIVINGSYSTEM_H
#define DRIVINGSYSTEM_H

#pragma once
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>
#include "TurnSensor.h"

using namespace Pololu3piPlus32U4;

#define NUM_SENSORS 5

typedef uint16_t cm;

class drivingSystem
{
public:
    drivingSystem(uint16_t maxSpeed = 200, uint16_t calibrationSpeed = 30, uint16_t turnSpeed = 30, uint16_t proportional = 64, uint16_t derivative = 256, uint16_t minSpeed = 0);
    ~drivingSystem();
    void calibrateSensors();
    uint16_t getBlackLineSensorReadings(uint16_t lineSensorValues[]);
    void followLine();
    bool crossedHorizontalLine();
    void printLineSensorValues();
    void driveXcmOnLine(cm distance);
    void driveXcmStraight(cm distance);
    void turnXdegree(uint16_t degree);
    bool driveToXYSave(uint8_t X, uint8_t Y, cm boxLength);
    bool driveToXYSpeed(uint8_t X, uint8_t Y, cm boxLength);
    bool driveToXYStraightLine(uint8_t X, uint8_t Y, cm boxLength);
    void stop();
    void startSequence();
    void driveNumberOfboxes(uint8_t amount);
    void stopAtNextLine();
    void followLineForever();
    void turnLeft(int degrees);
    void turnRight(int degrees);
    int32_t getAngle();
    void turningSensorReset();
    void turningSensorUpdate();
    void turningSensorSetup(); 
    void turningConstant(uint16_t speed);
    void setSpeeds(uint16_t maxSpeed = 200, uint16_t calibrationSpeed = 30, uint16_t turnSpeed = 30);

private:
    int16_t lastError_;
    uint16_t lineSensorValues_[NUM_SENSORS];
    uint16_t maxSpeed_;
    int16_t minSpeed_;
    uint16_t baseSpeed_;
    uint16_t calibrationSpeed_;
    uint16_t proportional_;
    uint16_t derivative_;
    LineSensors lineSensors_;
    Motors motors_;
    Encoders encoders_;
    uint16_t turnSpeed_;
    TurnSensor turnSensor_;
};

#endif