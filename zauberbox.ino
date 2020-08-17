#include <NeoSWSerial.h>
#include <TinyGPS++.h>
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
NeoSWSerial gpsSerial(3, 4);
TinyGPSPlus gps;

// Because gps.satellites.value() from TinyGPS++ does not work as
// expected we extract the number of satellites in view from the GPGSV
// sentence manually
TinyGPSCustom nmea_nsat(gps, "GPGSV", 3);

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
int year, nsat;
byte month, day, hour, minute, second;

void lcd_print_time(byte hour, byte minute, byte second) {
    lcd.setCursor(0, 0);
    if (hour < 10) { lcd.print("0"); }
    lcd.print(hour);
    lcd.print(":");
    if (minute < 10) { lcd.print("0"); }
    lcd.print(minute);
    lcd.print(":");
    if (second < 10) { lcd.print("0"); }
    lcd.print(second);
}

void lcd_print_nsat(int nsat) {
    lcd.setCursor(12, 0);
    lcd.print("[");
    if (nsat < 10) { lcd.print("0"); }
    lcd.print(nsat);
    lcd.print("]");
}

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
    servo1.attach(9);
    servo1.write(90);
    Serial.print("  Servo [ok]\n");

    // Setup LCD
    lcd.init();
    lcd.backlight();
    //lcd.noBacklight();
    lcd.clear();
    lcd_print_time(hour, minute, second);
    lcd_print_nsat(nsat);
    Serial.print("  LCD [ok]\n");

    // Setup output pin for piezo
    pinMode(6, OUTPUT);
}

// Main loop
void loop() {
    while(gpsSerial.available() > 0) { 
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated()) {
            year = gps.date.year();
            month = gps.date.month();
            day = gps.date.day();
            hour = gps.time.hour();
            minute = gps.time.minute();
            second = gps.time.second();
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
            lat = gps.location.lat();
            lon = gps.location.lng();

            Serial.print(lat, 6);
            Serial.print(", ");
            Serial.print(lon, 6);
            if (nmea_nsat.isUpdated()) {
                nsat = atol(nmea_nsat.value());
            } else {
                nsat = 0;
            }
            Serial.print(", ");
            Serial.print(nsat);
            Serial.print("\n");

            servo1.write(3*second);

            // Update LCD
            lcd_print_time(hour, minute, second);
            lcd_print_nsat(nsat);

            digitalWrite(6, HIGH);
            delay(50);
            digitalWrite(6, LOW);
        }
    }
}
