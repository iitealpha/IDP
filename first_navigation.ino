#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
// Mechanism setup
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define MAX_RANG (520)  //the max measurement value of the module is 520cm(a little bit longer than effective max range) 
#define ADC_SOLUTION (1023.0)  //ADC accuracy of Arduino UNO is 10bit 

// Mechanism initialisation
Servo myservo; // create servo object to control a servo
int pos = 0; // variable to store the servo position
int pos_max = 30; //set desired maximum rotation
Adafruit_TCS34725 tcs = Adafruit_TCS34725();
// End mechanism initialisation

int sensityPin = A0;  // ultrasonic input

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Define motors: left and right
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1); // Left Motor
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); // Right Motor

// Define sensor pins
// changed all int to uint8_t
const uint8_t sensorFarLeft = 2;  // Now at the back (near the wheels)
const uint8_t sensorLeft = 3;
const uint8_t sensorRight = 4;
const uint8_t sensorFarRight = 5;   // Now at the back (near the wheels)
const uint8_t button = 6;
const uint8_t LED_Red = 7;
const uint8_t LED_Green = 8;
const uint8_t LED_Blue = 10;

const uint8_t main_speed = 200;
const int delay_time = 25; // Time that will be delayed every single time

uint8_t mode = 0;   // Mode state: 0=off, 1=forward, 2=backward
unsigned long first_press_time = millis();

float dist_t, sensity_t; 

// map_names = [1,2,3,4,...20]
const int number_of_connections[20] = {1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3,2,2}; // This one keeps number of connections each graph has. 
// This graph shows numbers of connected graphs in a clockwise +1 way
// ORDER IS CRUCIAL
const int map_of_connections[20][3] = {{8,0,0},{10,0,0},{11,0,0},{9,0,0},{14,0,0},{18,0,0},{17,0,0},{12,9,1},{8,4,10},{11,2,9},{3,10,15},{19,13,8},{12,18,14},{15,5,13},{11,14,20},{17,18,19},{20,7,16},{13,6,16},{16,12,0},{15,17,0}};
const int first_direction[20] = {1,1,1,3,1,2,1,1,4,2,3,1,4,2,3,2,2,3,2,3}; //Direction (in compass) of the first listed connection. 
// Compass directions: 1 - North, 2 - East, 3 - South, 4 - West.

const int better_map_of_directions[20][20] = { // First coordinate is current graph, second is next graph. Result is the compass direction
{5,0,0,0,0,  0,0,1,0,0,     0,0,0,0,0,  0,0,0,0,0},
{0,5,0,0,0,  0,0,0,0,1,     0,0,0,0,0,  0,0,0,0,0},
{0,0,5,0,0,  0,0,0,0,0,     1,0,0,0,0,  0,0,0,0,0},
{0,0,0,5,0,  0,0,0,3,0,     0,0,0,0,0,  0,0,0,0,0},
{0,0,0,0,5,  0,0,0,0,0,     0,0,0,1,0,  0,0,0,0,0},
{0,0,0,0,0,  5,0,0,0,0,     0,0,0,0,0,  0,0,2,0,0},
{0,0,0,0,0,  0,5,0,0,0,     0,0,0,0,0,  0,1,0,0,0},

{3,0,0,0,0,  0,0,0,2,0,     0,1,0,0,0,  0,0,0,0,0}, //8
{0,0,0,1,0,  0,0,4,0,2,     0,0,0,0,0,  0,0,0,0,0},
{0,3,0,0,0,  0,0,0,4,0,     2,0,0,0,0,  0,0,0,0,0},
{0,0,3,0,0,  0,0,0,0,4,     0,0,0,0,1,  0,0,0,0,0}, //11
{0,0,0,0,0,  0,0,3,0,0,     0,0,2,0,0,  0,0,0,1,0},
{0,0,0,0,0,  0,0,0,0,0,     0,4,0,2,0,  0,0,1,0,0},
{0,0,0,0,3,  0,0,0,0,0,     0,0,4,0,2,  0,0,0,0,0}, //14
{0,0,0,0,0,  0,0,0,0,0,     3,0,0,4,0,  0,0,0,0,1},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,0,  0,2,3,4,0},
{0,0,0,0,0,  0,3,0,0,0,     0,0,0,0,0,  4,0,0,0,2}, //17
{0,0,0,0,0,  4,0,0,0,0,     0,0,3,0,0,  1,0,0,0,0},

{0,0,0,0,0,  0,0,0,0,0,     0,3,0,0,0,  2,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,3,  0,4,0,0,0}};

const int map_of_sizes[20][20] = { // First coordinate is current graph, second is goal graph, and result is distance between dots
{0,0,0,0,0,  0,0,350,0,0,     0,0,0,0,0,  0,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,350,     0,0,0,0,0,  0,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,0,     350,0,0,0,0,  0,0,0,0,0},
{0,0,0,0,0,  0,0,0,330,0,     0,0,0,0,0,  0,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,305,0,  0,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,0,  0,0,450,0,0},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,0,  0,360,0,0,0},

{350,0,0,0,0,  0,0,0,710,0,     0,850,0,0,0,  0,0,0,0,0}, //8
{0,0,0,330,0,  0,0,710,0,325,     0,0,0,0,0,  0,0,0,0,0},
{0,350,0,0,0,  0,0,0,325,0,     1035,0,0,0,0,  0,0,0,0,0},
{0,0,350,0,0,  0,0,0,0,1035,     0,0,0,0,850,  0,0,0,0,0}, //11
{0,0,0,0,0,  0,0,850,0,0,     0,0,1020,0,0,  0,0,0,640,0},
{0,0,0,0,0,  0,0,0,0,0,     0,1020,0,335,0,  0,0,370,0,0},
{0,0,0,0,305,  0,0,0,0,0,     0,0,335,0,720,  0,0,0,0,0}, //14
{0,0,0,0,0,  0,0,0,0,0,     850,0,0,720,0,  0,0,0,0,640},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,0,  0,420,380,900,0},
{0,0,0,0,0,  0,360,0,0,0,     0,0,0,0,0,  420,0,0,0,510}, //17
{0,0,0,0,0,  450,0,0,0,0,     0,0,370,0,0,  380,0,0,0,0},

{0,0,0,0,0,  0,0,0,0,0,     0,640,0,0,0,  900,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,640,  0,510,0,0,0}};

int current_graph = 10; //In default situation starts from graph 2
int current_graph_number = 0; // we always start from second element of array. 
int current_compass = 1; // In defolt situation starts from going to the North
int current_scenario = 1; // Starts from straight line
bool this_is_the_end = false; // Becomes true when we reach final destination and need to reverse or go backwards
unsigned long time_of_last_junction_detected;

bool moving;  // True if moving, for flashing LED.

//int random_path[] = {2,10,11,15,17,16,12,8,10}; // This one is just random, can have any length, just connect the graphs
int random_path[] = {2,10,9,4,4,9,8,12,13,14,15,11,10,2}; 

void reset(){
  // Reboots the arduino
  CPU_CCP = 0xD8;
  WDT.CTRLA = 0x4;
  while (true){}
}

ISR(TCB0_INT_vect){
  // Timer ISR for flashing blue LED when moving
  flash_led();
  // Clear interrupt flag
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

void flash_led(){
  if (moving){
    if (digitalRead(LED_Blue)){
      digitalWrite(LED_Blue, 0);
    }
    else{
      digitalWrite(LED_Blue, 1);
    }
  }
  else{
    digitalWrite(LED_Blue, 0);
  }
}

int new_compass(int current_graph_g, int destination_graph_g) {
  return better_map_of_directions[current_graph_g-1][destination_graph_g-1]; //Pretty useles, I usually assign directly. Change assignment to this function in the future
}

int mode_t_junction(int start, int finish) {
  int x = 200;
  //Serial.println(start);
  //Serial.println(finish);
  if ((start == 2) && (finish == 0)) {x = 2;}
  if ((start == 0) && (finish == 2)) {x = 3;}
  if ((start == 1) && (finish == 2)) {x = 4;}
  if ((start == 1) && (finish == 0)) {x = 5;}
  if ((start == 2) && (finish == 1)) {x = 6;}
  if ((start == 0) && (finish == 1)) {x = 7;}
  return x;
}

void move(int16_t speed, float rotation_fraction) {
  // This function describes every possible motion configuration in the most convinient (I think) way.
  // Speed that you give is the maximal speed of two wheels. 
  // Rotation fraction shows to which extent you rotate. 
  // rotation fraction = 0 => you go forward or backwards. 
  // Rotation fraction -1 means you move clockwise, 1 means you move anticklokwise.
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

  if (mode != 0) {
    if (v_left > 0) {
      myMotor1->run(FORWARD);
    } else {
      myMotor1->run(BACKWARD);
    }
    if (v_right > 0) {
      myMotor2->run(FORWARD);
    } else {
      myMotor2->run(BACKWARD);
    }} else {
      stop();
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
    if (mode == 0) {
      mode = 1; 
    } else {
      //mode = 0;
      reset();  // To stop the robot, reset arduino.
    }
    first_press_time = new_time;
  }
}

void setup() {
  Serial.begin(9600); // Start serial communication
  Serial.println("Starting...");

  // Mechanism setup
  myservo.attach(9); // use pin 9
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  } // Mechanism done

  // Set sensor pins as input
  pinMode(sensorFarLeft, INPUT);
  pinMode(sensorLeft, INPUT);
  pinMode(sensorRight, INPUT);
  pinMode(sensorFarRight, INPUT);
  pinMode(button, INPUT);
  pinMode(LED_Red, OUTPUT);
  pinMode(LED_Green, OUTPUT);
  pinMode(LED_Blue, OUTPUT);
  digitalWrite(LED_Red, 0);
  digitalWrite(LED_Green, 0);
  digitalWrite(LED_Blue, 0);

  // Initialize the Motor Shield
  if (!AFMS.begin()) {
    Serial.println("Motor Shield not found.");
    while (1); // Halt if shield not found
  }
  Serial.println("Motor Shield initialized.");

  // Button Interrupt:
  attachInterrupt(digitalPinToInterrupt(button), button_press_ISR, RISING);

  // Timer Interrupt:
  // Timer A (used as clock for B) clocked at 250kHz.
  TCB0.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode
  TCB0.CCMP = 62500; // Value to compare with. 62500 gives 2Hz.
  TCB0.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer

  current_compass = 1;
  current_graph = 2;

}

void straight_junction(){ // This function must go on as long as you are in the junction
  Serial.println("Go straight in junction");
  while ((digitalRead(sensorFarRight) == 1) || (digitalRead(sensorFarLeft) == 1)) {
    straight(); 
  } 
  //Serial.println("Straight junction is done");
  delay(delay_time);
}

void simple_mode_of_motion(){
  int y = better_map_of_directions[random_path[current_graph_number]-1][random_path[current_graph_number+1]-1];
  //Serial.println("Current graph");
  //Serial.println(random_path[current_graph_number]);
  //Serial.println("Next graph");
  //Serial.println(random_path[current_graph_number + 1]);
  //Serial.println("Goal compass");
  //Serial.println(y);
  //Serial.println("Current compass");
  //Serial.println(current_compass);
  if (y == 5) {
    // Include code for grabbing here
    this_is_the_end = true;
    Serial.print("NOW WE GO BACKWARDS");
    Serial.println(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION);
    stop_and_grab();
    Serial.println(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION);

  } else if (current_compass == 5){
    Serial.println("We have grabbed something");
  } else if (random_path[current_graph_number] == 19 || random_path[current_graph_number] == 20){
    Serial.println("Skip this junction");
  } else if ((4 + y - current_compass) % 4 == 3) { // Turn left
    left_junction();
    Serial.println("Left junction is done!!!");
  } else if ((4 + y - current_compass) % 4 == 1) {// Turn right
    right_junction();
  } else if ((4 + y - current_compass ) % 4 == 0) { // Go straight
    straight_junction();
  } else { // Go backwards
    backwards();
  }

}

void left_junction(){ // This function must go on as long as you are in the junction
  if (this_is_the_end == false) {
    move(main_speed, -1);
    Serial.print("You have entered left junction");
    while (digitalRead(sensorRight) == 1) {}
    while (digitalRead(sensorLeft) == 0) { // While right sensor is outside of its first line, move it to the line
    }
    Serial.print("You have entered the line after the left junction");
    while (digitalRead(sensorLeft) == 1) {
  } } else {
    backwards_left_junction();
  }
}

void right_junction(){ // This function must go on as long as you are in the junction
  if (this_is_the_end == false) {
    Serial.println("You have entered the right junction");      
    move(main_speed, 1);
    while (digitalRead(sensorRight) == 1) {}
    while (digitalRead(sensorRight) == 0) {}
    Serial.println("You have entered the line after the right junction");
    while (digitalRead(sensorRight) == 1) {
  } } else {
    backwards_right_junction();
  }
}

void straight(){ // Regular function for going straightforward
  bool right = digitalRead(sensorRight);
  bool left = digitalRead(sensorLeft);
    if (this_is_the_end == false) {
    if (right && !left) { //Move right
      move(main_speed, 0.5);
    } else if (!right && left) { //Move to the left
      move(main_speed, -0.5);
    } else if (!right && !left) { // Includes both going 
      move(main_speed, 0);
    } else { // Both are white, so we need time delay and going straightforward for short period of time ignoring all sensors. 
      for (int i = 0; i < 5; ++i) {
        move(main_speed, 0);
        delay(delay_time);
      }}
  } else {
    backwards();
  }
}

void backwards(){
  bool right = digitalRead(sensorRight);
  bool left = digitalRead(sensorLeft);  
  if (right && !left) { //M fgf ove left
  // rotate fraction it set to a small value to ensure robot corrects iteself instead of purely rotating
    move(-main_speed, -0.1); // Anti-Clockwise
  } else if (!right && left) { //Move to the right
    move(-main_speed, 0.1); // Clockwise
  } else if (!right && !left) { // Includes both going 
    move(-main_speed, 0);
  } else { // Both are white, so we need time delay and going straightforward for short period of time ignoring all sensors. 
    for (int i = 0; i < 5; ++i) {
      move(-main_speed, 0);
      delay(delay_time);
    }
  }
}

void backwards_left_junction(){ // Rotate clokwise until certain results
  Serial.print("You have entered the left junction");
  while (digitalRead(sensorRight) == 1) { 
    move(main_speed, 1);
    delay(delay_time);
  }
  while (digitalRead(sensorRight) == 0) {
    move(main_speed, 1);
    delay(delay_time);
  }
  while (digitalRead(sensorRight) == 1) {
    move(main_speed, 1);
    delay(delay_time);
  }
  this_is_the_end = false;
}

void backwards_right_junction(){ // Rotate anticlockwise until certain reusult. 
  
  Serial.print("You have entered the right junction");
  while (digitalRead(sensorLeft) == 1) { 
    move(main_speed, -1);
    delay(delay_time);
  }
  while (digitalRead(sensorLeft) == 0) {
    move(main_speed, -1);
    delay(delay_time);
  }
  while (digitalRead(sensorLeft) == 1) {
    move(main_speed, -1);
    delay(delay_time);
  }
  this_is_the_end = false;
}

// Mechanism start
void contract() {
  for (pos = 0; pos <= pos_max; pos += 1) { // goes from 0 degrees to max position
    myservo.write(pos); 
    delay(40);
  }
}

void release() {
  for (pos = pos_max; pos >= 0; pos -= 1) { // goes from max position to 0 degrees
    myservo.write(pos);
    delay(40);
  }
}

String block_detect() {
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);

  if (lux > 4000) {
    return "red"; }  
  else if (lux < 4000 && colorTemp > 10000) {
    return "black"; } 
  else {
    return "no block"; }
}

String mechanism() {
  contract();
  String a = block_detect();
  if (a == "no block") {
    release(); }
  return a; 
}

void stop_and_grab(){
  stop();
  // Put here code for grabbing
  mechanism();
  for (int i = 0; i < 50; ++i) {
    delay(delay_time);
  }
} 
// Mechanism done

void reverse(){
  move(main_speed, 1);
  while (digitalRead(sensorRight) == 1) {}
  while (digitalRead(sensorRight) == 0) {}
  Serial.println("You have entered the line after the reverse junction");
  while (digitalRead(sensorRight) == 1) {}
}

bool junction_detected(){
  int goal_compass = better_map_of_directions[random_path[current_graph_number]-1][random_path[current_graph_number+1]-1];
  if (digitalRead(sensorFarRight) || digitalRead(sensorFarLeft) || (goal_compass == 5 && analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION < 10.0) || (current_compass == 5) ) { 
    if (millis() - time_of_last_junction_detected > 1000){
      if (digitalRead(sensorFarRight)){Serial.println("FAR RIGHT");} else if (digitalRead(sensorFarLeft)){Serial.println("FAR LEFT");} else if (goal_compass == 5 && analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION < 10.0){Serial.println("TOO CLOSE");} else if (current_compass == 5){Serial.println("WAITING IS OVER");}
      return true;
    } else {
      Serial.println("TOO EARLY");
      return false;
    }
  } else {
    return false; 
  }
}

void loop() {
  //random_path = {2,10,11,15,20,17,7,7,17,16,18,13,12,8,1};
  
  bool farLeft = digitalRead(sensorFarLeft);
  bool left = digitalRead(sensorLeft);
  bool right = digitalRead(sensorRight);
  bool farRight = digitalRead(sensorFarRight);

  if (mode != 0) {
    moving = true;
    if (junction_detected()){ // When junction is detected, we need to 1) Do the junction to certain side, 2) Change the compass and 3) Change certain graph and 
      time_of_last_junction_detected = millis();
      simple_mode_of_motion(); // This function does corresponding turn and ends when the junction is done
      current_compass = better_map_of_directions[random_path[current_graph_number]-1][random_path[current_graph_number + 1]-1];
      //Serial.print("Compass was switched to " );
      //Serial.println(current_compass);
      current_graph_number = current_graph_number + 1; // Because finished simple_mode_of_motion means that we have gone through the graph and we need to get to the new graph
    } else {
      straight(); 
    }
    delay(delay_time); 
  } else {
    stop();
    moving = false;
  }


}
