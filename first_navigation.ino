#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define MAX_RANG (520)  //the max measurement value of the module is 520cm(a little bit longer than effective max range) 
#define ADC_SOLUTION (1023.0)  //ADC accuracy of Arduino UNO is 10bit 

int sensityPin = A0;  // ultrasonic input

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Define motors: left and right
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1); // Left Motor
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); // Right Motor

// Define sensor pins
// changed all int to uint8_t
const uint8_t sensorFarLeft = 2;  // Now at the back
const uint8_t sensorLeft = 3;
const uint8_t sensorRight = 4;
const uint8_t sensorFarRight = 5;   // Now at the back
const uint8_t button = 6;
const uint8_t LED1 = 7;
const uint8_t LED2 = 8;

const uint8_t main_speed = 150;
const int delay_time = 50; // Time that will be delayed every single time

uint8_t mode = 0;   // Mode state: 1=off, 2=forward, 3=backward
unsigned long first_press_time = millis();

uint8_t junction_state = 0;   // Juncion state: 0=none, 1=left, 2=right
uint8_t junction_state_new = 0;

float dist_t, sensity_t; 

// map_names = [1,2,3,4,...20]
const int map_with_names[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20}; // Graph that has names of the graphs, actually pretty useles.
const int number_of_connections[20] = {1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3,2,2}; // This one keeps number of connections each graph has. 
// This graph shows numbers of connected graphs in a clockwise +1 way
// ORDER IS CRUCIAL
const int map_of_connections[20][3] = {{8,0,0},{10,0,0},{11,0,0},{9,0,0},{14,0,0},{18,0,0},{17,0,0},{12,9,1},{8,4,10},{11,2,9},{3,10,15},{19,13,8},{12,18,14},{15,5,13},{11,14,20},{17,18,19},{20,7,16},{13,6,16},{16,12,0},{15,17,0}};
const int first_direction[20] = {1,1,1,3,1,2,1,1,4,2,3,1,4,2,3,2,2,3,2,3}; //Direction (in compass) of the first listed connection. 
// Compass directions: 1 - North, 2 - East, 3 - South, 4 - West.

const int better_map_of_directions[20][20] = {
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

const int map_of_sizes[20][20] = {
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

int current_graph = 2; //In defolt situatino starts from graph 2
int current_graph_number = 0; // we always start from 0 element of array. 
int current_compass = 1; // In defolt situation starts from going to the North
int current_scenario = 1; // Starts from straight line
bool this_is_the_end = false; // Becomes true when we reach final destination and need to reverse or go backwards
int random_path[] = {2,10,11,15,20,17,7,7,17,16,18,13,12,8,1}; // This one is just random, can have any length, just connect the graphs

int coordinate_from_compas(int current_graph_g,int current_compass_g){
  int direction_that_we_will_be_looking_for = 1 + (current_compass_g +1 )%4; // Shift current_compass by 2
  //Serial.println(direction_that_we_will_be_looking_for);
  //Serial.println(first_direction[current_graph_g-1]);
  return(direction_that_we_will_be_looking_for - first_direction[current_graph_g-1] + 4)%4;
}

int coordinate_from_final_destination(int current_graph_g, int destination_graph_g) {
  int x = 100;
  for (int i = 0; i < 3; ++i) {
    if (map_of_connections[current_graph_g-1][i] == destination_graph_g){x = i;}
  }
  return x;
}

int new_compass(int current_graph_g, int destination_graph_g) {
  return better_map_of_directions[current_graph_g][destination_graph_g];
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

int mode_of_movement(int current_graph_g, int next_graph_g, int current_compass_g) {
  int x = 300;
  if (number_of_connections[current_graph_g-1] == 1){
    x = 13; // Because it goes to the final destination
  } else if(number_of_connections[current_graph_g-1] == 2) {
    x = 1;
  } else {
    x = mode_t_junction(coordinate_from_compas(current_graph_g, current_compass_g), coordinate_from_final_destination(current_graph_g, next_graph_g));
  }
  return x;
}

void move(int16_t speed, int16_t rotation_fraction) {
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

void stop(){
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
}

void button_press_ISR(){
  // Debounce button
  unsigned long new_time = millis();
  if (millis() > (first_press_time + 300)){
    mode = (mode + 1) % 4;
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

  current_compass = 2;
  current_graph = 2;

  /**int answers[sizeof(random_path) / sizeof(random_path[0]) - 1];
  for (int i = 1; i < (sizeof(random_path) / sizeof(random_path[0])); ++i) {
    Serial.println(random_path[i]);
    answers[i-1] = mode_of_movement(random_path[i], random_path[i+1], current_compass);
    current_compass = 1 + (first_direction[random_path[i]-1] + coordinate_from_final_destination(random_path[i], random_path[i+1])+3) % 4;
    
    Serial.println(answers[i-1]);
  }
  answers[sizeof(random_path) / sizeof(random_path[0]) - 2] = 13; **/
}

void straight_junction(){ // This function must go on as long as you are in the junction
  while ((farRight == 1) || (farLeft == 1)) {
    straight(); 
  } 
  delay(delay_time);
}

void left_junction(){ // This function must go on as long as you are in the junction
  while (left == 0) {
    move(main_speed, 0.2);
    delay(delay_time);
  }
  while (left == 1) {
    move(main_speed, 0.2)
    delay(delay_time);
  }
}

void right_junction(){ // This function must go on as long as you are in the junction

}

void straight(){ // Regular function for going straightforward
  if (right && !left) { //Move left
    move(main_speed, 0.2);
  } else if (!right && left) { //Move to the right
    move(main_speed, -0.2);
  } else if (!right && !left) { // Includes both going 
    move(main_speed, 0);
  } else { // Both are white, so we need time delay and going straightforward for short period of time ignoring all sensors. 
    for (int i = 0; i < 5; ++i) {
      move(main_speed, 0);
      delay(delay_time);
    }
  }
}

void backwards(){
if (right && !left) { //Move left
    move(-main_speed, -1); // Counter clockwise
  } else if (!right && left) { //Move to the right
    move(-main_speed, 1); // Clockwise
  } else { // Includes both going 
    move(-main_speed, 0);
  } 
}

bool junction_detected(){
  if (farRight || farLeft || random_path[current_graph_number] == 19 || random_path[current_graph_number] == 20) { 
  //We are also checking for the current mode being some graph to smooth turning 
    return true;
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

  sensity_t = analogRead(sensityPin); 
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;

  if junction_detected(){ // When junction is detected, we need to 1) Do the junction to certain side, 2) Change the compass and 3) Change certain graph and 

  }
}
