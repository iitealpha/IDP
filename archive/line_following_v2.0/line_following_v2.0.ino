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
const int sensorFarLeft = 2;
const int sensorLeft = 3;
const int sensorRight = 4;
const int sensorFarRight = 5;
const int button = 6;
const int LED = 7;

const int main_speed = 150;

uint8_t mode = 0;   // Mode state: 1=off, 2=forward, 3=backward
int first_press_time = millis();



void button_press_ISR(){
  // Debounce button
  int new_time = millis();
  if (millis() > (first_press_time + 300)){
    mode = (mode + 1) % 3;
    first_press_time = new_time;
  }
  if(mode == 1){
    digitalWrite(LED, 1);
  } else{
    digitalWrite(LED, 0);
  }
}

void setup() {
  Serial.begin(9600); // Start serial communication
  Serial.println("Starting...");

  // Set sensor pins as input
  pinMode(sensorFarLeft, INPUT);
  pinMode(sensorLeft, INPUT);
  pinMode(sensorRight, INPUT);
  pinMode(sensorFarRight, INPUT);
  pinMode(button, INPUT);
  pinMode(LED, OUTPUT);

  // Initialize the Motor Shield
  if (!AFMS.begin()) {
    Serial.println("Motor Shield not found.");
    while (1); // Halt if shield not found
  }
  Serial.println("Motor Shield initialized.");

  // Button Interrupt:
  attachInterrupt(digitalPinToInterrupt(button), button_press_ISR, FALLING);
}

void forward(uint8_t speed){
  myMotor1->setSpeed(speed); 
  myMotor2->setSpeed(speed); 
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
}

void turn_left(uint8_t speed, int8_t diff_percent){
  myMotor1->setSpeed(speed); 
  myMotor2->setSpeed(speed*diff_percent/100); 
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
}

void turn_right(uint8_t speed, int8_t diff_percent){
  myMotor1->setSpeed(speed*diff_percent/100); 
  myMotor2->setSpeed(speed); 
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
}

void backwards(uint8_t speed){
  myMotor1->setSpeed(speed); 
  myMotor2->setSpeed(speed); 
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
}

void stop(){
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
}

void loop() {
  // Read sensors
  // Don't know if boolean will work but we'll see
  bool farLeft = digitalRead(sensorFarLeft);
  bool left = digitalRead(sensorLeft);
  bool right = digitalRead(sensorRight);
  bool farRight = digitalRead(sensorFarRight);

  // Simpple Line following logic, probably will wobble
  /*
  if(center) {
    // If center sensor detects line: move straight
    myMotor1->setSpeed(150);
    myMotor2->setSpeed(150);
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
  } else if(left) {
  */
  if(mode == 1){
    if(left){
      // If left sensor detects line: turn right
      turn_left(main_speed, 50);

    } else if(right) {
      // If right sensor detects line: turn left
      turn_right(main_speed, 50);

    } else {
      // Probably central, just go forward:
      forward(main_speed);
    }
    if (farLeft){
      turn_left(main_speed, 0);
    } else if(farRight){
      turn_right(main_speed, 0);
    }
  } else{
    stop();
  }

  delay(200); // Short delay to stabilize sensor readings
  // Change if increasing speed.
}