
#include <PololuMenu.h>

class receiveCoordinates
{
private:
    int m_xCoordinateInt;
    int m_yCoordinateInt;
    char m_xCoordinateChar;
    String m_yCoordinateString;
    
    void parserForCoordinates(String a_coordinatesAsString);

public:
    receiveCoordinates();
    void receiveNewCoordinates();
    int getX_int();
    int getY_int();
    char getX_char();
    String getY_String();
    String getCoordinatesAsString();
};