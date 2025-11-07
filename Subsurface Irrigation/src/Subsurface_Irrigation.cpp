/**
 * @file main.cpp
 * @brief Subsurface Irrigation System using Arduino and OLED Display
 * 
 * This program reads moisture levels from sensors, displays the values on an OLED screen,
 * and controls water relays and a pump based on the moisture levels. It also includes a button
 * to toggle the display between sensor values and a default message.
 * 
 * @author Yuhon
 * @date 2023
 */

 /**
 * @brief Reads moisture sensor values and maps them to a percentage (0-100).
 */
void read_value();

/**
 * @brief Draws the moisture sensor values on the OLED display.
 */
void drawValues();

/**
 * @brief Controls the watering process for the plants.
 */
void water_plant();

/**
 * @brief Puts the system to sleep for a specified number of minutes.
 */
void sleepMinutes(int minutes);

/**
 * @brief Dumps all logged data in CSV format to the serial monitor.
 */
void dumpAll();

#include <Arduino.h>
#include <Wire.h>
#include "U8glib.h"
#include <LowPower.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);

// array to store mapped moisture values
int moisture_values[4];

// set water relays
int relay_pins[4] = {6, 8, 9, 10};

// Moisture sensor ranges: {dry, wet} (subsurface)
// int moisture_ranges[4][2] = {
//   {578, 466}, // Sensor 0
//   {570, 430}, // Sensor 1
//   {575, 445}, // Sensor 2
//   {577, 447}  // Sensor 3
// };

// // Moisture sensor ranges: {dry, wet} (above surface)
int moisture_ranges[4][2] = {
  {660, 454}, // Sensor 0
  {630, 435}, // Sensor 1
  {628, 410}, // Sensor 2
  {622, 395}  // Sensor 3
};

// set watering thresholds as percentage
int thresholds[4] = {20, 20, 20, 20};

// set water pump
int pump = 4;

// set button
int button = 12;

// logging
const int MAX_LOG_LENGTH = 48;
unsigned long timestamps[MAX_LOG_LENGTH];
uint8_t log_s0[MAX_LOG_LENGTH];
uint8_t log_s1[MAX_LOG_LENGTH];
uint8_t log_s2[MAX_LOG_LENGTH];
uint8_t log_s3[MAX_LOG_LENGTH];
int log_count = 0;

int water_count = 0;

// User define functions

// Dump all logged data in CSV format
void dumpAll() {
  Serial.println(F("time_ms,sensor0,sensor1,sensor2,sensor3"));
  for (int i = 0; i < log_count; i++) {
    Serial.print(timestamps[i]);
    Serial.print(',');

    Serial.print(log_s0[i]);
    Serial.print(',');

    Serial.print(log_s1[i]);
    Serial.print(',');

    Serial.print(log_s2[i]);
    Serial.print(',');

    Serial.println(log_s3[i]);

    log_s0[i] = 0;
    log_s1[i] = 0;
    log_s2[i] = 0;
    log_s3[i] = 0;
  }
  Serial.println(water_count);
  water_count = 0;
}

// Read the moisture sensor values and map them to a percentage (0-100)
void read_value()
{
  for (int i = 0; i < 4; i++) {
    moisture_values[i] = map(
      analogRead(A0 + i),
      moisture_ranges[i][0], // dry
      moisture_ranges[i][1], // wet
      0, 100
    );
    // moisture_values[i] = analogRead(A0 + i);
  }
}

// Display sensor readings on the screen
void drawValues(){
  u8g.firstPage();
  do
  {
    const int baseX[4] = {0, 32, 64, 96};
    const int labelY   = 60;
    const int valueY   = 40;

    read_value();
    u8g.setFont(u8g_font_7x14);

    for (int i = 0; i < 4; i++) {
      // Draw port number
      u8g.setPrintPos(baseX[i] + 9, labelY);
      u8g.print(i);

      // Draw moisture value above the port number
      int shown = moisture_values[i];
      // if (shown < 0)   shown = 0;
      // if (shown > 100) shown = 100;
      char buf[5];
      itoa(shown, buf, 10);
      u8g.setPrintPos(baseX[i] + 9, valueY);
      u8g.print(buf);
    }
  } while ( u8g.nextPage() );
}

//Water plants to certain moisture level
void water_plant()
{
  int startVals[4];
  for (int i = 0; i < 4; i++) {
    int sensorValue = analogRead(A0 + i);
    startVals[i] = sensorValue;

    // Convert threshold percentage to raw sensor value for this sensor
    int threshold_raw = map(
      thresholds[i],
      0, 100,
      moisture_ranges[i][0], // dry
      moisture_ranges[i][1]  // wet
    );

    if (sensorValue > threshold_raw) { // dry
      water_count++;
      digitalWrite(relay_pins[i], HIGH); //activate relay
      digitalWrite(pump, HIGH); //activate pump
      delay(500);
      digitalWrite(relay_pins[i], LOW); //deactivate relay
      digitalWrite(pump, LOW); //deactivate pump
    }
  }
  // Log the data
  if (log_count < MAX_LOG_LENGTH) {
    timestamps[log_count] = millis();
    log_s0[log_count] = map(
      startVals[0],
      moisture_ranges[0][0], // dry
      moisture_ranges[0][1], // wet
      0, 100
    );
    log_s1[log_count] = map(
      startVals[1],
      moisture_ranges[1][0], // dry
      moisture_ranges[1][1], // wet
      0, 100
    );
    log_s2[log_count] = map(
      startVals[2],
      moisture_ranges[2][0], // dry
      moisture_ranges[2][1], // wet
      0, 100
    );
    log_s3[log_count] = map(
      startVals[3],
      moisture_ranges[3][0], // dry
      moisture_ranges[3][1], // wet
      0, 100
    );
    log_count++;
  }
}

//Sleep for a certain number of minutes
void sleepMinutes(int minutes) {
  uint32_t cycles = ((uint32_t)minutes * 60UL) / 8UL;
  for (int i = 0; i < cycles; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

void setup()
{
  delay(1500);
  Wire.begin();
  Serial.begin(9600);
  delay(5000);
  // declare relay as output
  for (int i = 0; i < 4; i++) {
    pinMode(relay_pins[i], OUTPUT);
    digitalWrite(relay_pins[i], LOW);
  }
  // declare pump as output
  pinMode(pump, OUTPUT);
  // declare switch as input
  pinMode(button, INPUT);
  digitalWrite(pump, HIGH);
}

void loop()
{
  u8g.sleepOff();
  unsigned long startTime = millis();
  water_plant();
  while (millis() - startTime < (10 * 60UL * 1000UL)) { // 10 minutes
    drawValues();
    delay(500);
    if (Serial){
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 'D' || c == 'd') {
          dumpAll();
        }
      }
      delay(100);
      if (!Serial) break;
    }
  }
  drawValues();
  //Prepare for sleep
  for (int i = 0; i < 4; i++) digitalWrite(relay_pins[i], LOW);
  digitalWrite(pump, LOW);
  u8g.sleepOn();
  delay(50);
  sleepMinutes(20);
}