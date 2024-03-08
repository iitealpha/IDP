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
// bool left = digitalRead(sensorLeft);
// bool center = digitalRead(sensorCenter);
// bool right = digitalRead(sensorRight);
//bool back = digitalRead(sensorBack);

  // Simpple Line following logic, probably will wobble
    // If center sensor detects line: move straight
myMotor1->setSpeed(150);
myMotor2->setSpeed(150);
myMotor1->run(FORWARD);
myMotor2->run(FORWARD);
}