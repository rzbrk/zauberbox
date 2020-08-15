//#include <SoftwareSerial.h>
#include <NeoSWSerial.h>
#include <TinyGPS.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* Define SW UART for GPS receiver
*
*  GPSr Pin  | Arduino Pin
* -----------+-------------
*  RX        | D3
*  TX        | D4
*
*/
//SoftwareSerial gpsSerial(3, 4);
NeoSWSerial gpsSerial(3, 4);
TinyGPS gps;

// Define servo
Servo servo1;

/* Define LCD
*  I2C address 0x27
*  16 columns
*  2 rows
*/
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define some variables for GPS data
float lat, lon;
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long fix_age;

// Setup Routine
void setup() {
    // Setup the serial port to computer
    Serial.begin(9600);
    Serial.print("Starting . . .\n");
    Serial.print("  UART to computer [ok]\n");

    // Setup the serial port to GPSr
    gpsSerial.begin(9600);
    Serial.print("  UART to GPSr [ok]\n");

    // Setup the servo
    //delay(1000);
    servo1.attach(9);
    servo1.write(90);
    Serial.print("  Servo [ok]\n");

    // Setup LCD
    lcd.init();
    lcd.backlight();
    //lcd.noBacklight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("It's Working!");
    Serial.print("  LCD [ok]\n");

    // Setup output pin for piezo
    pinMode(6, OUTPUT);
}

// Main loop
void loop() {
    while(gpsSerial.available()) { 
        if(gps.encode(gpsSerial.read())) { 
            gps.crack_datetime(&year, &month, &day, &hour, &minute,
                &second, &hundredths, &fix_age);
            Serial.print("  ");
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
            Serial.print(", ");
            Serial.print(fix_age);
            Serial.print("\n");

            servo1.write(3*second);

            lcd.setCursor(0, 1);
            lcd.print(hour);
            lcd.print(":");
            lcd.print(minute);
            lcd.print(":");
            lcd.print(second);
            lcd.print("   ");
            
            digitalWrite(6, HIGH);
            delay(50);
            digitalWrite(6, LOW);

        }
    }
}
