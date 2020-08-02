#include <SoftwareSerial.h>
#include <TinyGPS.h>

/* Define SW UART for GPS receiver
*
*  GPSr Pin  | Arduino Pin
* -----------+-------------
*  RX        | D3
*  TX        | D4
*
*/
SoftwareSerial gpsSerial(3, 4);
TinyGPS gps;

// Define some variables for GPS data
float lat, lon;
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long fix_age;

// Setup Routine
void setup() {
    // Setup the serial port to computer
    Serial.begin(9600);
    // Setup the serial port to GPSr
    gpsSerial.begin(9600);
}

// Main loop
void loop() {
    while(gpsSerial.available()) { 
        if(gps.encode(gpsSerial.read())) { 
            gps.crack_datetime(&year, &month, &day, &hour, &minute,
                &second, &hundredths, &fix_age);
            Serial.print(year);
            Serial.print("-");
            Serial.print(month);
            Serial.print("-");
            Serial.print(day);
            Serial.print(", ");
            Serial.print(hour);
            Serial.print(":");
            Serial.print(minute);
            Serial.print(":");
            Serial.print(second);
            Serial.print(", ");
            gps.f_get_position(&lat,&lon);
            Serial.print(lat, 6);
            Serial.print(", ");
            Serial.print(lon, 6);
            Serial.print("\n");
        }
    }
}
