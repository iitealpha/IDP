#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Developed from sample code of motors
// Simple algorithm, wobbling expected

// Initialize the Adafruit Motor Shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Define motors: left and right
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1); // Left Motor
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); // Right Motor

// Define sensor pins
const int sensorLeft = 4;
const int sensorCenter = 3;
const int sensorRight = 2;
//const int sensorBack = 5;

void setup() {
  Serial.begin(9600); // Start serial communication
  Serial.println("Starting...");

  // Set sensor pins as input
  pinMode(sensorLeft, INPUT);
  pinMode(sensorCenter, INPUT);
  pinMode(sensorRight, INPUT);
  //pinMode(sensorBack, INPUT);

  // Initialize the Motor Shield
  if (!AFMS.begin()) {
    Serial.println("Motor Shield not found.");
    while (1); // Halt if shield not found
  }
  Serial.println("Motor Shield initialized.");
}

void loop() {
  // Read sensors
  // Don't know if boolean will work but we'll see
  bool left = digitalRead(sensorLeft);
  bool center = digitalRead(sensorCenter);
  bool right = digitalRead(sensorRight);
  //bool back = digitalRead(sensorBack);

  // Simpple Line following logic, probably will wobble
  if(center) {
    // If center sensor detects line: move straight
    myMotor1->setSpeed(150);
    myMotor2->setSpeed(150);
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
  } else if(left) {
    // If left sensor detects line: turn right
    myMotor1->setSpeed(100); // Slow down left motor
    myMotor2->setSpeed(150); // Keep right motor at higher speed
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
  } else if(right) {
    // If right sensor detects line: turn left
    myMotor1->setSpeed(150); // Keep left motor at higher speed
    myMotor2->setSpeed(100); // Slow down right motor
    myMotor1->run(FORWARD`);
    myMotor2->run(FORWARD);
  //} else if(back) {
    // If back sensor detects line: robot has veered off, so stop
    //myMotor1->setSpeed(150);
    //myMotor2->setSpeed(150);
    //myMotor1->run(FORWARD); // Stop left motor
    //myMotor2->run(FORWARD); // Stop right motor
  } else {
    // If no sensor detects the line: stop
    myMotor1->setSpeed(150);
    myMotor2->setSpeed(150);
    myMotor1->run(RELEASE);
    myMotor2->run(RELEASE);
  }

  delay(100); // Short delay to stabilize sensor readings
}