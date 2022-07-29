#include <SoftwareSerial.h>

SoftwareSerial gps(0, 1); // RX, TX
byte gpsdata = 0;

void getGpsData()
{
  gps.begin(9600);
  gpsdata = gps.read();
  Serial.write( gpsdata );
}
