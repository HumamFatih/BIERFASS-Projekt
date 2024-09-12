/* This example uses the line sensors on the 3pi+ 32U4 to follow
a black line on a white background, using a PID-based algorithm.
It works well on courses with smooth, 6" radius curves and can
even work with tighter turns, including sharp 90 degree corners.
This example has been tested with robots using 30:1 MP motors.
Modifications might be required for it to work well on different
courses or with different motors. */

// #include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>
#include "drivingSystem.h"
#include "pololuDisplay.h"
#include "receiveCoordinates.h"

using namespace Pololu3piPlus32U4;

ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;

//Buzzer buzzer;

drivingSystem Bot(110, 30, 30);

pololuDisplay pDisplay;

receiveCoordinates pCoordinates;

void setup()
{
  Bot.turningSensorSetup();
  Bot.turningSensorReset();
  pDisplay.initDisplay();
  pDisplay.startScreen();
}

void loop()
{
  Bot.turningSensorUpdate();
  // rage mode
  if (buttonC.isPressed())
  {
    Bot.setSpeeds(200);
    pDisplay.modeScreen(A);
    delay(250);
    pDisplay.updateTimeSinceProgrammStart();
    Bot.turnRight(120);

    while (!(Serial1.available() > 1))
    {
      Bot.turningConstant(17);
    }
    Bot.stop();
    pCoordinates.receiveNewCoordinates();
    pDisplay.showTargetCoordinates(pCoordinates.getX_char(), pCoordinates.getY_String());
    Bot.driveToXYStraightLine(pCoordinates.getX_int(), pCoordinates.getY_int(), 10);
    pDisplay.updateCompetitionTime();
    pDisplay.stopwatchForPololu();
  }
  // slow mode
  else if (buttonB.isPressed())
  {
    Bot.setSpeeds(80);
    pDisplay.modeScreen(A);
    delay(250);
    pDisplay.updateTimeSinceProgrammStart();
    Bot.turnRight(120);

    while (!(Serial1.available() > 1))
    {
      Bot.turningConstant(17);
    }
    Bot.stop();
    pCoordinates.receiveNewCoordinates();
    pDisplay.showTargetCoordinates(pCoordinates.getX_char(), pCoordinates.getY_String());
    Bot.driveToXYSpeed(pCoordinates.getX_int(), pCoordinates.getY_int()-1, 10);
    pDisplay.updateCompetitionTime();
    pDisplay.stopwatchForPololu();
  }
  // fast mode
  else if (buttonA.isPressed())
  {
    pDisplay.modeScreen(A);
    delay(250);
    pDisplay.updateTimeSinceProgrammStart();
    Bot.turnRight(120);

    while (!(Serial1.available() > 1))
    {
      Bot.turningConstant(17);
    }
    Bot.stop();
    pCoordinates.receiveNewCoordinates();
    pDisplay.showTargetCoordinates(pCoordinates.getX_char(), pCoordinates.getY_String());
    Bot.driveToXYSpeed(pCoordinates.getX_int(), pCoordinates.getY_int()-1, 10);
    pDisplay.updateCompetitionTime();
    pDisplay.stopwatchForPololu();
  }
  else
  {
    // Bot.printLineSensorValues();
  }
}