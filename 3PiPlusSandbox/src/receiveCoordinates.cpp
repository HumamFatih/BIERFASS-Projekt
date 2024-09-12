
#include "receiveCoordinates.h"

receiveCoordinates::receiveCoordinates()
{
    Serial1.begin(9600);
    m_xCoordinateInt = 0;
    m_yCoordinateInt = 0;
    m_xCoordinateChar = 0;
    m_yCoordinateString = "";
}

void receiveCoordinates::receiveNewCoordinates()
{
    parserForCoordinates(getCoordinatesAsString());
}

String receiveCoordinates::getCoordinatesAsString()
{
    char incomingChars[7];
    String incomingString = "";
    //if (Serial1.available() >= 7)
    //{
        Serial1.readBytes(incomingChars, 7);
    //}
    
    for (int i = 0; i < 7; i++)
    {
        if (incomingChars[i] >= 0x40)
        {
            incomingString += incomingChars[i];
            incomingString += incomingChars[i + 1];
            incomingString += incomingChars[i + 2];
            if (incomingChars[i + 3] == '0')
            {
                incomingString += incomingChars[i + 3];
            }
            i = 7;
        }
    }

    return incomingString;
}

void receiveCoordinates::parserForCoordinates(String a_coordinatesAsString)
{
    m_xCoordinateChar = a_coordinatesAsString.charAt(0);
    m_xCoordinateInt = m_xCoordinateChar - 0x41;
    m_yCoordinateString = a_coordinatesAsString.charAt(2);
    m_yCoordinateInt = a_coordinatesAsString.charAt(2) - 0x30;

    if (a_coordinatesAsString.length() >= 4)
    {
        m_yCoordinateString += a_coordinatesAsString.charAt(3) - 0x30;
        m_yCoordinateInt = 10;
    }
}

int receiveCoordinates::getX_int()
{
    return m_xCoordinateInt;
}

int receiveCoordinates::getY_int()
{
    return m_yCoordinateInt;
}

char receiveCoordinates::getX_char()
{
    return m_xCoordinateChar;
}
String receiveCoordinates::getY_String()
{
    return m_yCoordinateString;
}
