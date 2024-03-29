#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Graph.h"
#include "Graph.cpp"

#define MAX_RANG (520)  //the max measurement value of the module is 520cm(a little bit longer than effective max range) 
#define ADC_SOLUTION (1023.0)  //ADC accuracy of Arduino UNO is 10bit 

int sensityPin = A0;  // ultrasonic input

// Developed from sample code of motors
// Simple algorithm, wobbling expected

// Initialize the Adafruit Motor Shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Define motors: left and right
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1); // Left front Motor
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); // Right front Motor

// Define sensor pins
const int sensorFarLeft = 2;  // Now at the back
const int sensorLeft = 3;
const int sensorRight = 4;
const int sensorFarRight = 5;   // Now at the back
const int button = 6;
const int LED1 = 7;
const int LED2 = 8;

const int main_speed = 150; // Is this the maximal speed?

uint8_t mode = 0;   // Mode state: 1=off, 2=forward, 3=backward
unsigned long first_press_time = millis();

uint8_t junction_state = 0;   // Juncion state: 0=none, 1=left, 2=right
uint8_t junction_state_new = 0;

float dist_t, sensity_t; 

Graph g_map[] = { // Never ever touch this or you will break logic of robot. 
    Graph(1, 1, new int[3]{8, 0, 0}, new int[3]{1, 0, 0}),
    Graph(2, 1, new int[3]{10, 0, 0}, new int[3]{1, 0, 0}),
    Graph(3, 1, new int[3]{11, 0, 0}, new int[3]{1, 0, 0}),
    Graph(4, 1, new int[3]{9, 0, 0}, new int[3]{3, 0, 0}),
    Graph(5, 1, new int[3]{14, 0, 0}, new int[3]{1, 0, 0}),
    Graph(6, 1, new int[3]{18, 0, 0}, new int[3]{2, 0, 0}),
    Graph(7, 1, new int[3]{17, 0, 0}, new int[3]{1, 0, 0}),
    Graph(8, 3, new int[3]{9, 1, 12}, new int[3]{2, 3, 1}),
    Graph(9, 3, new int[3]{4, 10, 8}, new int[3]{1, 2, 4}),
    Graph(10, 3, new int[3]{2, 9, 11}, new int[3]{3, 4, 2}),
    Graph(11, 3, new int[3]{10, 15, 3}, new int[3]{4, 1, 3}),
    Graph(12, 3, new int[3]{13, 8, 19}, new int[3]{2, 3, 1}),
    Graph(13, 3, new int[3]{18, 14, 12}, new int[3]{1, 2, 4}),
    Graph(14, 3, new int[3]{5, 13, 15}, new int[3]{3, 4, 2}),
    Graph(15, 3, new int[3]{14, 20, 11}, new int[3]{4, 1, 3}),
    Graph(16, 3, new int[3]{18, 19, 17}, new int[3]{3, 4, 2}),
    Graph(17, 3, new int[3]{7, 16, 20}, new int[3]{3, 4, 2}),
    Graph(18, 3, new int[3]{6, 16, 13}, new int[3]{4, 1, 3}),
    Graph(19, 2, new int[3]{12, 16, 0}, new int[3]{3, 2, 0}),
    Graph(20, 2, new int[3]{15, 17, 0}, new int[3]{3, 4, 0})
};
//1 - North, 2 - East, 3 - South, 4 - West

void move(int16_t speed, int16_t rotation_fraction) {
  // This function describes every possible motion configuration in the most convinient (I think) way.
  // Speed that you give is the maximal speed of two wheels. 
  // Rotation fraction shows to which extent you rotate. 
  // rotation fraction = 0 => you go forward or backwards. 
  // Rotation fraction 1 means you move clockwise, -1 means you move anticklokwise.

  int16_t other_speed = 0;
  int16_t v_left = 0;
  int16_t v_right = 0;

  if (rotation_fraction > 0) {
    other_speed = speed - 2*rotation_fraction*speed;
  } else {
    other_speed = speed + 2*rotation_fraction*speed;
  }

  if (((rotation_fraction>0) && (speed<0)) || ((rotation_fraction<0) && (speed>0))){
    v_left = speed;
    v_right = other_speed;
  } else {
    v_right = speed;
    v_left = other_speed;
  }

  myMotor1->setSpeed(abs(v_left)); 
  myMotor2->setSpeed(abs(v_right));

  if (v_left > 0) {
    myMotor1->run(FORWARD);
  } else {
    myMotor1->run(BACKWARD);
  }
  if (v_right > 0) {
    myMotor2->run(FORWARD);
  } else {
    myMotor2->run(BACKWARD);
  }
}

void forward(int16_t speed){ //forward(speed) = move(speed, 0)
  // use negative speed for reversing
  myMotor1->setSpeed(abs(speed)); 
  myMotor2->setSpeed(abs(speed));
  if (speed > 0){
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
  } else{
    myMotor1->run(BACKWARD);
    myMotor2->run(BACKWARD);
  }
}

void turn_left(int16_t speed, int8_t diff_percent){
  myMotor1->setSpeed(abs(speed)); 
  myMotor2->setSpeed(abs(speed*diff_percent/100)); 
  if (speed > 0){
    myMotor1->run(FORWARD);
    if (diff_percent > 0){
      myMotor2->run(FORWARD);
    } else{
      myMotor2->run(BACKWARD);
    }
  } else{
    myMotor1->run(BACKWARD);
    if (diff_percent > 0){
      myMotor2->run(BACKWARD);
    } else{
      myMotor2->run(FORWARD);
    }
  }
}

void turn_right(int16_t speed, int8_t diff_percent){
  myMotor1->setSpeed(abs(speed*diff_percent/100)); 
  myMotor2->setSpeed(abs(speed)); 
  if (speed > 0){
    myMotor2->run(FORWARD);
    if (diff_percent > 0){
      myMotor1->run(FORWARD);
    } else{
      myMotor1->run(BACKWARD);
    }
  } else{
    myMotor2->run(BACKWARD);
    if (diff_percent > 0){
      myMotor1->run(BACKWARD);
    } else{
      myMotor1->run(FORWARD);
    }
  }
}


void stop(){
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
}



void button_press_ISR(){
  // Debounce button
  unsigned long new_time = millis();
  if (millis() > (first_press_time + 300)){
    mode = (mode + 1) % 3;
    first_press_time = new_time;
    junction_state = 0;
    junction_state_new = 0;
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
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, 0);
  digitalWrite(LED2, 0);

  // Initialize the Motor Shield
  if (!AFMS.begin()) {
    Serial.println("Motor Shield not found.");
    while (1); // Halt if shield not found
  }
  Serial.println("Motor Shield initialized.");

  // Button Interrupt:
  attachInterrupt(digitalPinToInterrupt(button), button_press_ISR, RISING);
}



void loop() {
  // Read sensors
  // Don't know if boolean will work but we'll see
  bool farLeft = digitalRead(sensorFarLeft);
  bool left = digitalRead(sensorLeft);
  bool right = digitalRead(sensorRight);
  bool farRight = digitalRead(sensorFarRight);

  sensity_t = analogRead(sensityPin); 
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;

  if(mode == 1){
    // Forwards motion
    if (junction_state == 0){
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
      if (farLeft){   // Junction, probably want to turn until front sensors go white, then back to regular line following.
        junction_state_new = 1;
        turn_left(main_speed, -100);
      } else if (farRight){
        junction_state_new = 2;
        turn_right(main_speed, -100);
      }
    } else if (junction_state == 1){
      // Need to turn left until front left sensor goes white
      if (right){
        junction_state_new = 0;
        // This is a problem if left sensor is already white before turn.
        // May be better to use an interrupt after a certain delay.
      }
    } else if (junction_state == 2){
      // Need to turn right until front right sensor goes white
      if (left){
        junction_state_new = 0;
      }
    }
    if (dist_t < 10.0){
      // close to wall
      mode = 2; // reverse
      junction_state = 0;
      junction_state_new = 0;
    }

  } else if(mode == 2){
    // Reverse motion
    if (junction_state == 0){
      // Assume straight for now
      if(left){
        // If left sensor detects line: turn right
        //turn_left(-main_speed, 50);

      } else if(right) {
        // If right sensor detects line: turn left
        //turn_right(-main_speed, 50);

      } else {
        // Probably central, just go forward:
        forward(-main_speed);
      }
      if (farLeft and farRight){   // T-junction
        junction_state_new = 1;   // Turn right?
        turn_left(-main_speed, -100);
      }
    } else if (junction_state == 1){
      // Turn until front sensor lights:
        if (right){
          junction_state_new = 0;
          mode = 1; // go forward again.
        }
    }
  } else{
    stop();
  }

  junction_state = junction_state_new;
  if(mode == 1){
    digitalWrite(LED1, 1);
    digitalWrite(LED2, 0);
  } else if(mode == 2){
    digitalWrite(LED1, 0);
    digitalWrite(LED2, 1);
  } else{
    digitalWrite(LED1, 0);
    digitalWrite(LED2, 0);
  }
  delay(50); // Short delay to stabilize sensor readings
  // Change if increasing speed.

}
