// LSM6DS3_Arduino_Rotation - LSM6DS3_Arduino_Rotation.ino
//
// Description:
// Retrieves motion data from the on-board LSM6DS3 IMU of the Arduino Uno WiFi
// Rev2 using the Arduino_LSM6DS3 library and displays rotation angles (roll,
// pitch, and yaw) in the Serial Monitor.
//
// Created by John Woolsey on 01/28/2020.
// Copyright (c) 2019 Woolsey Workshop.  All rights reserved.
// Includes
#include <Arduino_LSM6DS3.h>
#include <MadgwickAHRS.h>
// Defines
#define SAMPLE_RATE 10  // in Hz
// Constructors
Madgwick filter;  // Madgwick algorithm for roll, pitch, and yaw calculations

float angle = 0;
float dist_forward = 0;
float ax, ay, az;  // accelerometer values
float gx, gy, gz;  // gyroscope values


ISR(TCB1_INT_vect){
  // Take reading:
  getAccelerations();
  // Clear interrupt flag
  TCB1.INTFLAGS = TCB_CAPT_bm;
}





void setup() {
  Serial.begin(9600);  // initialize serial bus (Serial Monitor)
  while (!Serial);  // wait for serial initialization
  Serial.print("LSM6DS3 IMU initialization ");
  if (IMU.begin()) {  // initialize IMU
    Serial.println("completed successfully.");
  } else {
    Serial.println("FAILED.");
    IMU.end();
    while (1);
  }
  Serial.println();
  filter.begin(SAMPLE_RATE);  // initialize Madgwick filter

  TCB1.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode
  TCB1.CCMP = 125; // Value to compare with. 125 gives 1KHz
  TCB1.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
  TCB1.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer
}
void loop() {
   //static unsigned long previousTime = millis();
   //unsigned long currentTime = millis();
   //if (currentTime - previousTime >= 1000/SAMPLE_RATE) {
      // printValues();
      //printRotationAngles();
      //printValues();
      //previousTime = millis();
   //}
   char buffer[5];    // string buffer for use with dtostrf() function
   Serial.print("Yaw = ");  Serial.print(dtostrf(angle, 4, 0, buffer)); Serial.println(" °");
   delay(100);
}
// Prints IMU values.
void printValues() {
   char buffer[8];    // string buffer for use with dtostrf() function
   float ax, ay, az;  // accelerometer values
   float gx, gy, gz;  // gyroscope values
   // Retrieve and print IMU values
   if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()
      && IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
      gx += 1.2;
      gy += 3.4;
      gz -= 2.0;
      ay -= 0.1;
      Serial.print("ax = ");  Serial.print(dtostrf(ax, 4, 1, buffer));  Serial.print(" g, ");
      Serial.print("ay = ");  Serial.print(dtostrf(ay, 4, 1, buffer));  Serial.print(" g, ");
      Serial.print("az = ");  Serial.print(dtostrf(az, 4, 1, buffer));  Serial.print(" g, ");
      Serial.print("gx = ");  Serial.print(dtostrf(gx, 7, 1, buffer));  Serial.print(" °/s, ");
      Serial.print("gy = ");  Serial.print(dtostrf(gy, 7, 1, buffer));  Serial.print(" °/s, ");
      Serial.print("gz = ");  Serial.print(dtostrf(gz, 7, 1, buffer));  Serial.println(" °/s");
   }
}
// Prints rotation angles (roll, pitch, and yaw) calculated using the
// Madgwick algorithm.
// Note: Yaw is relative, not absolute, based on initial starting position.
// Calculating a true yaw (heading) angle requires an additional data source,
// such as a magnometer.
void getAccelerations() {
   if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()
      && IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
      gx += 1.2;
      gy += 3.4;
      gz -= 2.0;
      ay -= 0.1;
      filter.updateIMU(gx, gy, gz, ax, ay, az);  // update roll, pitch, and yaw values
      angle = filter.getYaw();
      // may want to calculate distance, although will not be accurate. 
      

      
      //Serial.print("X_dist = "); 
   }
}