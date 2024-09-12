#include "TurnSensor.h"

TurnSensor::TurnSensor(){
    turnRate = 0;
    gryroLastUpdate = 0;
}

void TurnSensor::turnSensorReset(){
    gryroLastUpdate = micros();
    turnAngle = 0;
}

void TurnSensor::turnSensorUpdate(){
    imu.readGyro();
    turnRate = imu.g.z - gyroOffset;

    // Figure out how much time has passed since the last update (dt)
    uint16_t m = micros();
    uint16_t dt = m - gryroLastUpdate;
    gryroLastUpdate = m;

    // (angular change = angular velocity * time)
    int32_t d = (int32_t)turnRate * dt;

    // The units of d are gyro digits times microseconds.  We need
    // to convert those to the units of turnAngle, where 2^29 units
    // represents 45 degrees.  The conversion from gyro digits to
    // degrees per second (dps) is determined by the sensitivity of
    // the gyro: 0.07 degrees per second per digit.
    //
    // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
    // = 14680064/17578125 unit/(digit*us)
    turnAngle += (int64_t)d * 14680064 / 17578125;
}

void TurnSensor::turnSensorSetup(){
    Wire.begin();
    imu.init();
    imu.enableDefault();
    imu.configureForTurnSensing();
    ledYellow(1);
    delay(500);

    // Calibrate the gyro.
    int32_t total = 0;
    for (uint16_t i = 0; i < 1024; i++)
        {
        // Wait for new data to be available, then read it.
        while(!imu.gyroDataReady()) {}
        imu.readGyro();
        // Add the Z axis reading to the total.
        total += imu.g.z;
  }
    ledYellow(0);
    gyroOffset = total / 1024;
}

uint32_t TurnSensor::getTurnAngle(){
    return turnAngle;
}