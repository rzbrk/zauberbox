#include <NeoSWSerial.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <CmdLine.h>
#include <EEPROM.h>

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
void cli_position(const char* arg);
void cli_servo_unlock(const char* arg);
void cli_servo_lock(const char* arg);
void cli_servo_getpos(const char* arg);
void cli_wpts_show(const char* arg);
void cli_wpts_next(const char* arg);
void cli_wpts_prev(const char* arg);
void cli_beep (const char* arg);
void cli_reset (const char* arg);

const cmd_t commands[] = {
    {"help", cli_help},
    {"time", cli_time},
    {"date", cli_date},
    {"position", cli_position},
    {"servo_unlock", cli_servo_unlock},
    {"servo_lock", cli_servo_lock},
    {"servo_getpos", cli_servo_getpos},
    {"wpts_show", cli_wpts_show},
    {"wpts_next", cli_wpts_next},
    {"wpts_prev", cli_wpts_prev},
    {"beep", cli_beep},
    {"reset", cli_reset},
};

// Define some variables for GPS data
// lat          current latitude from GPS [degrees]
// lon          current longitude from GPS [degrees]
// dist         distance to active waypoint [meters]
double lat, lon, dist;
// year         year, e.g. 2020
// nsat         GPS satellites in view [1]
int year, nsat;
// month        month, 1...12
// day          day, 1...31
// hour         hour, 0...23
// minute       minute, 0...59
// second       second, 0...59
// wpt          index of active waypoint (row in array waypoint),
//              range: 0...WPTS_NUMBER. If wpt = WPTS_NUMBER all
//              waypoints are passed
byte month, day, hour, minute, second, wpt;

/* Define a multidimensional array to store waypoints
*   Column | Value
*  ---------+---------------------------
*        0 | latitude
*        1 | longitude
*/
#define WPTS_NUMBER 3
// Define the address in EEPROM to store the index of current waypoint
#define EE_ADDR 0
double waypoint[WPTS_NUMBER][2] = {
    {48.85826, 2.294516}, // Paris Eiffel Tower
    {50.638983, 7.235957}, // Selhof Kapelle
    {3.0, 3.0},
};

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

    // Setup the servo. Set the servo to SERVO_INIT_POS (usually locked)
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

    // Setup output pin for piezo
    pinMode(PIEZO_PIN, OUTPUT);
    Serial.print(F("  Piezo [ok]\r\n"));

    // After reboot, initialize number of current waypoint from EEPROM
    wpt = EEPROM.read(EE_ADDR);
    if (wpt > WPTS_NUMBER) {
        wpt = WPTS_NUMBER;
    }
    Serial.print(F("  Initialized active waypoint from EEPROM [ok]\r\n"));
    Serial.print(F("\r\n"));

    // Setup CLI
    cmdline.begin(commands, sizeof(commands));
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

            nsat = gps.satellites.value();

            // Calculate the distance between the current GPS position and
            // the current waypoint in meters.
            // If wpt = WPTS_NUMBER, that last waypoint in the list (array
            // waypoint) are passed. In this case, set dist to zero and
            // unlock the box.
            if (wpt < WPTS_NUMBER) {
                dist = TinyGPSPlus::distanceBetween(
                    lat,
                    lon,
                    waypoint[wpt][0],
                    waypoint[wpt][1]);
            } else {
                dist = 0.0;
                servo1.write(SERVO_UNLOCK_POS);
            }

            // Update LCD
            lcd.clear();
            lcd_print_time(hour, minute, second);
            lcd_print_nsat(nsat);
            lcd_print_distance(dist);

            // Let piezo produce beep
            //beep(50);
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

// Function to display/update the distance to the next waypoint on LCD
void lcd_print_distance(double dist) {
    String unit = "m";
    
    if (dist > 1000) {
        dist = dist / 1000.0;
        unit = "km";
    }
    lcd.setCursor(0,1);
    lcd.print(dist);
    lcd.print(F(" "));
    lcd.print(unit);
}

void cli_help(const char* arg) {
    Serial.print(F("\r\n-=# Arduino Zauberbox #=-\r\n\r\n"));
    Serial.print(F("Available commands:\r\n"));
    Serial.print(F("  help            print this help message\r\n"));
    Serial.print(F("  time            print GPS time\r\n"));
    Serial.print(F("  date            print GPS date\r\n"));
    Serial.print(F("  position        print GPS position (lat/lon)\r\n"));
    Serial.print(F("  servo_unlock    unlock box\r\n"));
    Serial.print(F("  servo_lock      lock box\r\n"));
    Serial.print(F("  servo_getpos    print current servo position\r\n"));
    Serial.print(F("  wpts_show       print list of waypoints\r\n"));
    Serial.print(F("  wpts_next       skip to next waypoints\r\n"));
    Serial.print(F("  wpts_prev       skip to prev waypoints\r\n"));
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
    if (year == 0) { Serial.print(F("000")); }
    Serial.print(year);
    Serial.print(F("-"));
    if (month < 10) { Serial.print(F("0")); }
    Serial.print(month);
    Serial.print(F("-"));
    if (day < 10) { Serial.print(F("0")); }
    Serial.print(day);
    Serial.print(F("\r\n\r\n"));
}

void cli_position(const char* arg) {
    static char outstr[15];
    Serial.print(F("  Current position:\r\n"));
    Serial.print(F("  Latitude:  "));
    dtostrf(lat, 10, 4, outstr);
    Serial.print(outstr);
    Serial.print(F("\r\n"));
    Serial.print(F("  Longitude: "));
    dtostrf(lon, 10, 4, outstr);
    Serial.print(outstr);
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

void cli_wpts_show(const char* arg) {
    static char outstr[15];
    Serial.print(F("  Waypoints:\r\n\r\n"));
    Serial.print(F("   no | lat          | lon          | passed?\r\n"));
    Serial.print(F("  ----+--------------+--------------+---------\r\n"));
    for (int i = 0; i < WPTS_NUMBER; i++) {
        Serial.print(F("   "));
        if (i < 10) { Serial.print(F(" ")); }
        Serial.print(i);
        Serial.print(F(" |"));
        for (int j = 0; j < 2; j++) {
            dtostrf(waypoint[i][j], 13, 6, outstr);
            Serial.print(outstr);
            Serial.print(F(" |"));
        }
        if (i >= wpt) {
            Serial.print(F("      no"));
        } else {
            Serial.print(F("     yes"));
        }
        Serial.print(F("\r\n"));
    }
    Serial.print(F("\r\n"));
}

void cli_wpts_next(const char* arg) {
    Serial.print(F("  Skip to next waypoint "));
    if (wpt < WPTS_NUMBER) {
        wpt = wpt + 1;
    }
    EEPROM.write(EE_ADDR, wpt);
    Serial.print(F("[ok]\r\n\r\n"));
}

void cli_wpts_prev(const char* arg) {
    Serial.print(F("  Skip to prev waypoint "));
    if (wpt > 0) {
        wpt = wpt - 1;
    }
    EEPROM.write(EE_ADDR, wpt);
    Serial.print(F("[ok]\r\n\r\n"));
}

void cli_beep(const char* arg) {
    beep(50);
}

void cli_reset(const char* arg) {
    reset_func();
}
