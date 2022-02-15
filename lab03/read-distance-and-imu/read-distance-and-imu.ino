#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distanceSensorTwo(Wire, 4, INTERRUPT_PIN);

void setup(void)
{
  // Activate 1 ToF sensor
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  // Explicilty deactivate the other ToF Sensor
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  
  Wire.begin();
  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  // Activate the first sensor
  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("ToF Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("ToF Sensor 1 online!");

  // NEW: Change first sensor address
  Serial.println("Changing sensor address to 0x30");
  distanceSensor.setI2CAddress(0x30);
  // ...then reactivate the other
  digitalWrite(4, HIGH);
  delay(500);
  if (distanceSensorTwo.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("ToF Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("ToF Sensor 2 online!");
  
}

void print_distance(SFEVL53L1X sensor, String sensorName) {
  sensor.startRanging();
  while (!sensor.checkForDataReady()) {
    delay(10);
  }
  int distance = sensor.getDistance();
  sensor.clearInterrupt();
  sensor.stopRanging();

  Serial.print(sensorName);
  Serial.print(" | ");
  Serial.print("Distance(mm): ");
  Serial.print(distance);
  
  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distanceFeet, 2);

  Serial.println();
}

void loop(void)
{
  print_distance(distanceSensor, "Sensor 1");
  print_distance(distanceSensorTwo, "Sensor 2");
  delay(1000);
}
