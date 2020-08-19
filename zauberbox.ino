#include <NeoSWSerial.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <CmdLine.h>

// Speed of serial port
#define SERIAL_BAUDRATE 9600

/* Define SW UART for GPS receiver
*
*  GPSr Pin  | Arduino Pin
* -----------+-------------
*  RX        | D3
*  TX        | D4
*
*/
#define GPS_RX_PIN 3
#define GPS_TX_PIN 4
#define GPS_BAUDRATE 9600
NeoSWSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

// Because gps.satellites.value() from TinyGPS++ does not work as
// expected we extract the number of satellites in view from the GPGSV
// sentence manually
TinyGPSCustom nmea_nsat(gps, "GPGSV", 3);

// Define servo
// Servo attached to pin D9 of microcontroller
#define SERVO_PIN 9
#define SERVO_INIT_POS 90
#define SERVO_UNLOCK_POS 0
#define SERVO_LOCK_POS 180
Servo servo1;

// Define piezo
// Piezo attached to pin D6 of microcontroller
#define PIEZO_PIN 6

/* Define LCD
*  I2C address 0x27
*  16 columns
*  2 rows
*/
#define LCD_I2C_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// Create the command line interface (CLI)and use the Serial port
// to introduce commands
// See: http://blog.industrialshields.com/en/command-line-arduino-library/
CmdLine cmdline(Serial);

// Define the Arduino CLI commands and associate them a function
void cli_help(const char* arg);
void cli_time(const char* arg);
void cli_date(const char* arg);
void cli_servo_unlock(const char* arg);
void cli_servo_lock(const char* arg);
void cli_servo_getpos(const char* arg);
void cli_beep (const char* arg);
void cli_reset (const char* arg);

const cmd_t commands[] = {
    {"help", cli_help},
    {"time", cli_time},
    {"date", cli_date},
    {"servo_unlock", cli_servo_unlock},
    {"servo_lock", cli_servo_lock},
    {"servo_getpos", cli_servo_getpos},
    {"beep", cli_beep},
    {"reset", cli_reset},
};

// Define some variables for GPS data
float lat, lon;
int year, nsat;
byte month, day, hour, minute, second;

// Setup Routine
void setup() {
    // Setup the serial port to computer
    Serial.begin(SERIAL_BAUDRATE);
    while (!Serial);
    delay(1000);
    Serial.print(F("\r\n-=# Arduino Zauberbox #=-\r\n\r\n"));
    Serial.print(F("  Starting . . .\r\n"));
    Serial.print(F("  UART to computer [ok]\r\n"));

    // Setup the serial port to GPSr
    gpsSerial.begin(GPS_BAUDRATE);
    Serial.print(F("  UART to GPSr [ok]\r\n"));

    // Setup the servo
    servo1.attach(SERVO_PIN);
    servo1.write(SERVO_INIT_POS);
    Serial.print(F("  Servo [ok]\r\n"));

    // Setup LCD
    lcd.init();
    lcd.backlight();
    //lcd.noBacklight();
    lcd.clear();
    lcd_print_time(hour, minute, second);
    lcd_print_nsat(nsat);
    Serial.print(F("  LCD [ok]\r\n"));

    // Setup CLI
    cmdline.begin(commands, sizeof(commands));

    // Setup output pin for piezo
    pinMode(PIEZO_PIN, OUTPUT);
}

// Main loop
void loop() {
    while(gpsSerial.available() > 0) { 
        // Update cmdline often to process commands written in the Serial port
        cmdline.update();
        
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated()) {
            year = gps.date.year();
            month = gps.date.month();
            day = gps.date.day();
            hour = gps.time.hour();
            minute = gps.time.minute();
            second = gps.time.second();

            lat = gps.location.lat();
            lon = gps.location.lng();

            if (nmea_nsat.isUpdated()) {
                nsat = atol(nmea_nsat.value());
            } else {
                nsat = 0;
            }

            servo1.write(3*second);

            // Update LCD
            lcd_print_time(hour, minute, second);
            lcd_print_nsat(nsat);

            // Let piezo produce beep
            beep(50);
        }
    }
}

// Define reset function. When called, go to address zero.
void (* reset_func) (void) = 0;

// Let the piezo beep for a given time
void beep(long millisec) {
    digitalWrite(PIEZO_PIN, HIGH);
    delay(millisec);
    digitalWrite(PIEZO_PIN, LOW);
}

// Function to display/update time on LCD
// HH:MM:SS
void lcd_print_time(byte hour, byte minute, byte second) {
    // Print time in upper-left corner
    lcd.setCursor(0, 0);
    if (hour < 10) { lcd.print(F("0")); }
    lcd.print(hour);
    lcd.print(F(":"));
    if (minute < 10) { lcd.print(F("0")); }
    lcd.print(minute);
    lcd.print(F(":"));
    if (second < 10) { lcd.print(F("0")); }
    lcd.print(second);
}

// Function to display/update number of satellites on LCD
// [NN]
void lcd_print_nsat(int nsat) {
    // Print nsat to upper-right corner
    lcd.setCursor(12, 0);
    lcd.print(F("["));
    if (nsat < 10) { lcd.print(F("0")); }
    lcd.print(nsat);
    lcd.print(F("]"));
}

void cli_help(const char* arg) {
    Serial.print(F("\r\n-=# Arduino Zauberbox #=-\r\n\r\n"));
    Serial.print(F("Available commands:\r\n"));
    Serial.print(F("  help            print this help message\r\n"));
    Serial.print(F("  time            print GPS time\r\n"));
    Serial.print(F("  date            print GPS date\r\n"));
    Serial.print(F("  servo_unlock    unlock box\r\n"));
    Serial.print(F("  servo_lock      lock box\r\n"));
    Serial.print(F("  servo_getpos    print current servo position\r\n"));
    Serial.print(F("  beep            produces beep\r\n"));
    Serial.print(F("  reset           perform software reset\r\n"));
    Serial.print(F("\r\n"));
}

void cli_time(const char* arg) { 
    Serial.print(F("  Current time: "));
    if (hour < 10) { Serial.print(F("0")); }
    Serial.print(hour);
    Serial.print(F(":"));
    if (minute < 10) { Serial.print(F("0")); }
    Serial.print(minute);
    Serial.print(F(":"));
    if (second < 10) { Serial.print(F("0")); }
    Serial.print(second);
    Serial.print(F("\r\n\r\n"));
}

void cli_date(const char* arg) { 
    Serial.print(F("  Current date: "));
    if (year = 0) { Serial.print(F("000")); }
    Serial.print(year);
    Serial.print(F("-"));
    if (month < 10) { Serial.print(F("0")); }
    Serial.print(month);
    Serial.print(F("-"));
    if (day < 10) { Serial.print(F("0")); }
    Serial.print(day);
    Serial.print(F("\r\n\r\n"));
}
void cli_servo_unlock(const char* arg) {
    Serial.print(F("  Unlocking . . . "));
    servo1.write(SERVO_UNLOCK_POS);
    Serial.print(F("[ok]\r\n\r\n"));
}

void cli_servo_lock(const char* arg) {
    Serial.print(F("  Locking . . . "));
    servo1.write(SERVO_LOCK_POS);
    Serial.print(F("[ok]\r\n\r\n"));
}

void cli_servo_getpos(const char* arg) {
    Serial.print(F("  Servo position: "));
    Serial.print(servo1.read());
    Serial.print(F("\r\n\r\n"));
}

void cli_beep(const char* arg) {
    beep(50);
}

void cli_reset(const char* arg) {
    reset_func();
}
