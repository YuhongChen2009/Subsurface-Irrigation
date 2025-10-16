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

#include <Arduino.h>
#include <Wire.h>
#include "U8glib.h"
#include <LowPower.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);

// array to store mapped moisture values
int moisture_values[4];

// set water relays
int relay_pins[4] = {6, 8, 9, 10};

// Moisture sensor ranges: {dry, wet}
int moisture_ranges[4][2] = {
  {600, 466}, // Sensor 0
  {612, 450}, // Sensor 1
  {608, 480}, // Sensor 2
  {620, 469}  // Sensor 3
};

// set watering thresholds as percentage
int thresholds[4] = {20, 20, 20, 20};

int sub_buffer = 50;
int above_buffer = 75;

// set water pump
int pump = 4;

// set button
int button = 12;

// User define functions

// Read the moisture sensor values and map them to a percentage (0-100)
//TEST
void read_value()
{
  for (int i = 0; i < 4; i++) {
    moisture_values[i] = map(
      analogRead(A0 + i),
      moisture_ranges[i][0], // dry
      moisture_ranges[i][1], // wet
      0, 100
    );
    Serial.print(analogRead(A0 + i));
    Serial.print(" ");
  }
  Serial.println();
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
      if (shown < 0)   shown = 0;
      if (shown > 100) shown = 100;
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
  for (int i = 0; i < 4; i++) {
    int sensorValue = analogRead(A0 + i);

    // Convert threshold percentage to raw sensor value for this sensor
    int threshold_raw = map(
      thresholds[i],
      0, 100,
      moisture_ranges[i][0], // dry
      moisture_ranges[i][1]  // wet
    );

    Serial.println(sensorValue);

    if (sensorValue > threshold_raw) { // dry
      digitalWrite(relay_pins[i], HIGH); //activate relay
      digitalWrite(pump, HIGH); //activate pump
      while (true) {
        drawValues();
        if (sensorValue - above_buffer < threshold_raw) { // wet enough
          digitalWrite(relay_pins[i], LOW); //deactivate relay
          digitalWrite(pump, LOW); //deactivate pump
          break;
        }
        sensorValue = analogRead(A0 + i);
        delay(500);
      }
    }
    else {
      digitalWrite(relay_pins[i], LOW); //deactivate relay
      digitalWrite(pump, LOW); //deactivate pump
    }
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
  drawValues();
  water_plant();
  //Prepare for sleep
  for (int i = 0; i < 4; i++) digitalWrite(relay_pins[i], LOW);
  digitalWrite(pump, LOW);
  u8g.sleepOn();
  delay(50);
  sleepMinutes(30);
}