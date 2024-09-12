
#include "pololuDisplay.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

pololuDisplay::pololuDisplay()
{
    Adafruit_SSD1306 m_display(128, 64, &Wire, -1);
    m_timeSinceProgrammStart = 0;
    m_competitionTime = 0;
    m_timeSinceProgrammStart = millis();
}

void pololuDisplay::initDisplay()
{
    m_display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    m_display.display();
    delay(2000);
    m_display.clearDisplay();
    m_display.setTextColor(SSD1306_WHITE);
    m_display.setCursor(0, 0);
    m_display.display();
}

void pololuDisplay::stopwatchForPololu()
{
    m_display.setTextSize(2);
    unsigned long milliseconds = m_competitionTime % 1000;
    unsigned long seconds = (m_competitionTime / 1000) % 60;
    unsigned long minutes = (m_competitionTime / 60000) % 60;

    m_display.clearDisplay();
    m_display.setCursor(0, 1);

    m_display.println("Time: ");

    if (minutes < 10)
    {
        m_display.print("0");
    }
    m_display.print(minutes);
    m_display.print(":");

    if (seconds < 10)
    {
        m_display.print("0");
    }
    m_display.print(seconds);
    m_display.print(":");

    if (milliseconds < 100)
    {
        m_display.print("0");
    }
    if (milliseconds < 10)
    {
        m_display.print("0");
    }
    m_display.print(milliseconds);

    m_display.display();
}

void pololuDisplay::showTargetCoordinates(char x, String y)
{
    m_display.setTextSize(1);
    m_display.clearDisplay();
    m_display.setCursor(0, 1);
    m_display.print("Target:");
    m_display.print(x);
    m_display.print(y);
    m_display.display();
}

void pololuDisplay::updateCompetitionTime()
{
    m_competitionTime = millis() - m_timeSinceProgrammStart;
}

void pololuDisplay::updateTimeSinceProgrammStart()
{
    m_timeSinceProgrammStart = millis();
}

void pololuDisplay::updateTimeSinceCompetitionStart()
{
    m_timeSinceCompetitionStart = millis();
}

void pololuDisplay::printString(String a_string)
{
    m_display.clearDisplay();
    m_display.setCursor(0, 1);
    m_display.print(a_string);
    m_display.display();
}

void pololuDisplay::startScreen()
{
    m_display.setTextSize(1);
    m_display.setCursor(0, 1);
    m_display.clearDisplay();
    m_display.println("Choose the mode:");
    m_display.println("A for fast-mode");
    m_display.println("B for slow-mode");
    m_display.println("C for rage-mode");
    m_display.display();
}

void pololuDisplay::modeScreen(screenModes a_mode)
{
    m_display.setTextSize(1);
    m_display.setCursor(0, 1);
    m_display.clearDisplay();
    switch (a_mode)
    {
    case A:
        m_display.println("fast-mode:");
        m_display.println("");
        m_display.println("Waiting for");
        m_display.println("coordinates...");
        break;
    case B:
        m_display.println("slow-mode:");
        m_display.println("");
        m_display.println("Waiting for");
        m_display.println("coordinates...");
        break;

    case C:
        m_display.println("rage-mode:");
        m_display.println("");
        m_display.println("Waiting for");
        m_display.println("coordinates...");
        break;
    }
    m_display.display();
}