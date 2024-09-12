#include "drivingSystem.h"
#include <Pololu3piPlus32U4IMU.h>

// Allowed deviation (in degrees) relative to target angle that must be achieved before driving straight
#define DEVIATION_THRESHOLD 5

using namespace std;

/**
 * @brief Constructor for the drivingSystem class.
 *
 * @param maxSpeed Maximum speed of the driving system.
 * @param calibrationSpeed Speed used during sensor calibration.
 * @param proportional Proportional term for PID control.
 * @param derivative Derivative term for PID control.
 * @param minSpeed Minimum speed of the driving system.
 */
drivingSystem::drivingSystem(uint16_t maxSpeed, uint16_t calibrationSpeed, uint16_t turnSpeed, uint16_t proportional, uint16_t derivative, uint16_t minSpeed)
{
    maxSpeed_ = maxSpeed;
    minSpeed_ = minSpeed;
    baseSpeed_ = maxSpeed;
    turnSpeed_ = turnSpeed;
    calibrationSpeed_ = calibrationSpeed;
    proportional_ = proportional;
    derivative_ = derivative;
    // buzzer_.play(">g32>>c32");
    motors_.setSpeeds(0, 0);
    // turnAngle = 0;
    // gryroLastUpdate = 0;
}

drivingSystem::~drivingSystem()
{
    // Destructor implementation, if needed
}

/**
 * @brief Calibrates the line sensors by rotating in place.
 *
 * Wait 1.5 seconds and then begin automatic sensor calibration
 * by rotating in place to sweep the sensors over the line.
 */
void drivingSystem::calibrateSensors()
{
    lineSensors_.resetCalibration();
    delay(1500);
    for (uint8_t i = 0; i < 80; i++)
    {
        if (i > 20 && i <= 60)
        {
            motors_.setSpeeds(-(int16_t)calibrationSpeed_, calibrationSpeed_);
        }
        else
        {
            motors_.setSpeeds(calibrationSpeed_, -(int16_t)calibrationSpeed_);
        }

        lineSensors_.calibrate();
    }
    motors_.setSpeeds(0, 0);
    // buzzer_.play("L16 cdegreg4");
}

/// \brief Reads the sensors, provides calibrated values, and returns an
/// estimated black line position.
///
/// \param[out] sensorValues A pointer to an array in which to store the
/// calibrated sensor readings.  There **MUST** be space in the array for
/// five values. It writes the raw values to this pointer.
///
/// \return An estimate of the position of a black line under the sensors.
///
/// The estimate is made using a weighted average of the sensor indices
/// multiplied by 1000, so that a return value of 0 indicates that the line
/// is directly below sensor 0, a return value of 1000 indicates that the
/// line is directly below sensor 1, 2000 indicates that it's below sensor
/// 2000, etc. Intermediate values indicate that the line is between two
/// sensors. The formula is (where \f$v_0\f$ represents the value from the
/// first sensor):
///
/// \f[
/// {(0 \times v_0) + (1000 \times v_1) + (2000 \times v_2) + \cdots
/// \over
/// v_0 + v_1 + v_2 + \cdots}
/// \f]
///
/// As long as your sensors aren’t spaced too far apart relative to the
/// line, this returned value is designed to be monotonic, which makes it
/// great for use in closed-loop PID control. Additionally, this method
/// remembers where it last saw the line, so if you ever lose the line to
/// the left or the right, its line position will continue to indicate the
/// direction you need to go to reacquire the line. For example, if sensor
/// 4 is your rightmost sensor and you end up completely off the line to
/// the left, this function will continue to return 4000.
///
/// This function is intended to detect a black (or dark-colored) line on a
/// white (or light-colored) background. For a white line, see
/// readLineWhite().
uint16_t drivingSystem::getBlackLineSensorReadings(uint16_t lineSensorValues[])
{
    return lineSensors_.readLineBlack(lineSensorValues);
}

/**
 * @brief Follows the black line based on sensor readings.
 *
 * The function adjusts motor speeds using proportional and derivative
 * PID terms to keep the robot following the line.
 */
void drivingSystem::followLine()
{
    // Get the position of the line.  Note that we *must* provide
    // the "lineSensorValues" argument to readLineBlack() here, even
    // though we are not interested in the individual sensor
    // readings.
    int16_t position = getBlackLineSensorReadings(lineSensorValues_);

    // Our "error" is how far we are away from the center of the
    // line, which corresponds to position 2000.
    int16_t error = position - 2000;

    // Get motor speed difference using proportional and derivative
    // PID terms (the integral term is generally not very useful
    // for line following).
    int16_t speedDifference = error * (int32_t)proportional_ / 256 + (error - lastError_) * (int32_t)derivative_ / 256;

    lastError_ = error;

    // Get individual motor speeds.  The sign of speedDifference
    // determines if the robot turns left or right.
    int16_t leftSpeed = (int16_t)baseSpeed_ + speedDifference;
    int16_t rightSpeed = (int16_t)baseSpeed_ - speedDifference;

    // Constrain our motor speeds to be between 0 and maxSpeed.
    // One motor will always be turning at maxSpeed, and the other
    // will be at maxSpeed-|speedDifference| if that is positive,
    // else it will be stationary.  For some applications, you
    // might want to allow the motor speed to go negative so that
    // it can spin in reverse.
    leftSpeed = constrain(leftSpeed, minSpeed_, (int16_t)maxSpeed_);
    rightSpeed = constrain(rightSpeed, minSpeed_, (int16_t)maxSpeed_);

    motors_.setSpeeds(leftSpeed, rightSpeed);
}

/**
 * @brief Checks if the robot has crossed a horizontal line.
 *
 * @return True if the robot has crossed a horizontal line, false otherwise.
 */
bool drivingSystem::crossedHorizontalLine()
{
    getBlackLineSensorReadings(lineSensorValues_);
    // bot wiggles around, that makes it difficult to count lines
    if ((lineSensorValues_[0] > 700) && (lineSensorValues_[4] > 700))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void drivingSystem::printLineSensorValues()
{
    getBlackLineSensorReadings(lineSensorValues_);

    Serial.print(String(lineSensorValues_[0], DEC).substring(0, 4) + " | ");
    Serial.print(String(lineSensorValues_[1], DEC).substring(0, 4) + " | ");
    Serial.print(String(lineSensorValues_[2], DEC).substring(0, 4) + " | ");
    Serial.print(String(lineSensorValues_[3], DEC).substring(0, 4) + " | ");
    Serial.println(String(lineSensorValues_[4], DEC).substring(0, 4));
}

/**
 * @brief Drives the robot a specified distance in centimeters.
 *
 * @param distance Distance to drive in centimeters.
 */
void drivingSystem::driveXcmOnLine(cm distance)
{
    int32_t encoderSteps = distance * 500 / 15;
    encoders_.getCountsAndResetLeft();
    encoders_.getCountsAndResetRight();

    while (encoders_.getCountsLeft() < encoderSteps && encoders_.getCountsRight() < encoderSteps)
    {
        followLine();
    }
    encoders_.getCountsAndResetLeft();
    encoders_.getCountsAndResetRight();
    motors_.setSpeeds(0, 0);
}

/**
 * @brief Drives the robot a specified distance in centimeters.
 *
 * @param distance Distance to drive in centimeters.
 */
void drivingSystem::driveXcmStraight(cm distance)
{
    int16_t encoderSteps = distance * 500 / 15;
    encoders_.getCountsAndResetLeft();
    encoders_.getCountsAndResetRight();

    while (encoders_.getCountsLeft() < encoderSteps && encoders_.getCountsRight() < encoderSteps)
    {
        motors_.setSpeeds(maxSpeed_, maxSpeed_);
    }
    encoders_.getCountsAndResetLeft();
    encoders_.getCountsAndResetRight();
    motors_.setSpeeds(0, 0);
}

void drivingSystem::turnXdegree(uint16_t degree)
{
    // Placeholder implementation for turning a specific degree
}

bool drivingSystem::driveToXYSave(uint8_t X, uint8_t Y, cm boxLength)
{
    driveXcmOnLine(boxLength * Y);
    delay(100);
    turnRight(90);
    delay(100);
    driveXcmOnLine(boxLength * X-1);
    stopAtNextLine();
    return true;
}

bool drivingSystem::driveToXYSpeed(uint8_t X, uint8_t Y, cm boxLength)
{
    if (X==0)
    {
        driveXcmOnLine(boxLength*Y);
    }else{
        driveToXYSave(X, Y, boxLength);
    }
    
    return true;
}

bool drivingSystem::driveToXYStraightLine(uint8_t X, uint8_t Y, cm boxLength)
{
    uint16_t length = sqrt(pow((X * boxLength), 2) + pow((Y * boxLength), 2));
    double angle = atan2(X, Y) * 180 / PI;

    Serial.println(length);
    Serial.println(angle);

    turnRight(angle - (angle * 0.05));
    delay(100);
    driveXcmStraight(length);
    return true;
}

/**
 * @brief Stops the robot by setting motor speeds to zero.
 */
void drivingSystem::stop()
{
    motors_.setSpeeds(0, 0);
}

/**
 * @brief Drives the robot a specified number of boxes.
 *
 * @param amount Number of boxes to drive.
 */
void drivingSystem::driveNumberOfboxes(uint8_t amount)
{
    bool onLine = false;
    uint8_t crossedLines = 0;
    while (crossedLines <= amount)
    {
        onLine = crossedHorizontalLine();
        /*    if(lastCrossedHorizontalLine !=  onLine){
               lastCrossedHorizontalLine = onLine;
               if(onLine == true){
               crossedLines++;
               }
           } */
        if (onLine)
        {
            crossedLines++;
        }
        followLine();
    }
}

/**
 * @brief Stops the robot when it reaches the next horizontal line.
 */
void drivingSystem::stopAtNextLine()
{
    uint16_t prev_max = maxSpeed_;
    uint16_t prev_base = baseSpeed_;
    maxSpeed_ = 75;
    baseSpeed_ = maxSpeed_;
    do
    {
        followLine();
    } while ((lineSensorValues_[0] < 700) && (lineSensorValues_[4] < 700));
    maxSpeed_ = prev_max;
    baseSpeed_ = prev_base;
    stop();
}

/**
 * @brief Follows the line forever.
 *
 * This function continuously follows the line.
 */
void drivingSystem::followLineForever()
{
    while (true)
    {
        followLine();
    }
}

int32_t drivingSystem::getAngle()
{
    //  return (((int32_t)turnAngle >> 16) * 360) >> 16;
    return (((int32_t)turnSensor_.getTurnAngle() >> 16) * 360) >> 16;
}

// Turn left
void drivingSystem::turnLeft(int degrees)
{
    turnSensor_.turnSensorReset();
    turnSensor_.turnSensorUpdate();
    motors_.setSpeeds(-turnSpeed_, turnSpeed_);
    int angle = 0;
    do
    {
        delay(1);
        turnSensor_.turnSensorUpdate();
        angle = (((int32_t)turnSensor_.getTurnAngle() >> 16) * 360) >> 16;
    } while (angle < degrees);
    motors_.setSpeeds(0, 0);
}

// Turn right
void drivingSystem::turnRight(int degrees)
{
    turnSensor_.turnSensorReset();
    turnSensor_.turnSensorUpdate();
    motors_.setSpeeds(turnSpeed_, -turnSpeed_);
    int angle = 0;
    do
    {
        delay(1);
        turnSensor_.turnSensorUpdate();
        angle = (((int32_t)turnSensor_.getTurnAngle() >> 16) * 360) >> 16;
        lineSensors_.calibrate();
    } while (angle > -degrees);
    motors_.setSpeeds(0, 0);
}

void drivingSystem::turningSensorReset()
{
    turnSensor_.turnSensorReset();
}

void drivingSystem::turningSensorSetup()
{
    turnSensor_.turnSensorSetup();
}

void drivingSystem::turningSensorUpdate()
{
    turnSensor_.turnSensorUpdate();
}

void drivingSystem::turningConstant(uint16_t speed)
{
    motors_.setSpeeds(speed, -speed);
    lineSensors_.calibrate();
}

void drivingSystem::setSpeeds(uint16_t maxSpeed, uint16_t calibrationSpeed, uint16_t turnSpeed){
    maxSpeed_ = maxSpeed;
    calibrationSpeed_ = calibrationSpeed;
    turnSpeed_ = turnSpeed;
}