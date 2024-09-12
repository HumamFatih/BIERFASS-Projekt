
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "string.h"

enum screenModes{A, B, C};

class pololuDisplay
{
private:
Adafruit_SSD1306 m_display;
volatile unsigned long m_timeSinceProgrammStart;
volatile unsigned long m_timeSinceCompetitionStart;
volatile unsigned long m_competitionTime; 

public:
pololuDisplay();
void stopwatchForPololu();
void updateCompetitionTime();
void updateTimeSinceProgrammStart();
void updateTimeSinceCompetitionStart();
void showTargetCoordinates(char x, String y);
void initDisplay();
void printString(String a_string);
void startScreen();
void modeScreen(screenModes a_mode);
};