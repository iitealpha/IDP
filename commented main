#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include "Adafruit_TCS34725.h"
#include <Wire.h>
Servo mech_servo;


#define MAX_RANG (520)  //the max measurement value of the module is 520cm(a little bit longer than effective max range) 
#define ADC_SOLUTION (1023.0)  //ADC accuracy of Arduino UNO is 10bit 
#define ULTRASONIC_SAMPLE_PERIOD (10) // sample period in ms

#define DEBUG false	// enables serial monitor output, SLOWS DOWN the car SIGNIFICANTLY
#define DEBUG_SERIAL if(DEBUG)SerialNina

int sensityPin = A0;  // ultrasonic input

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Define motors: left and right
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1); // Left Motor
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); // Right Motor

// Define sensor pins
// changed all int to uint8_t
const uint8_t sensorFarLeft = 2;  // Left sensor in the middle of length the car, used for junctions detection, far from FarRightSensor
const uint8_t sensorLeft = 3; // Left sensor in the front of the car, used for understanding how to correct car to make it follow the line. 0 in normal case
const uint8_t sensorRight = 4; // Right sensor in the front of the car, used for understanding how to correct car to make it follow the line. 0 in normal case
const uint8_t sensorFarRight = 5;   // Right sensor in the middle of length the car, used for junctions detection, far from FarLeftSensor
const uint8_t button = 6; // 2
const uint8_t LED_Red = 7;
const uint8_t LED_Green = 8;
const uint8_t LED_Blue = 9;
const uint8_t servo_pin = 10;
const uint8_t loop_speed_test_pin = 11;

//cube color, 1 for red, 2 for black, 3 for nothing, 0 for error
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
const uint8_t cube_color = 0;

const uint8_t main_speed = 255;
const uint8_t slow_speed = 130;
const int delay_time = 1; // Time that will be delayed every single time

uint8_t mode = 0;   // Mode state: 0=off, 1=forward, 2=backward
unsigned long first_press_time = millis();
const float signal_distance = 10.0;

float dist_t, sensity_t; 

// This one keeps number of connections each graph has. Example: number_of_connections[4-1] = 1, therefore graph four has one connection (it is a bay)
const uint8_t number_of_connections[20] = {1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3,2,2}; 

// Put as a coordinate graph number - 1. Example: distances_from_bay[4-1] - distance from the fourth graph and the wall that you need to have to stop
const float distances_from_bays[20] = {0, 12, 0, 10, 8, 8, 10, 10.0 ,0,0,10 ,10 ,0,0,10 , 15.0 ,0,0,0,0}; 

// Compass directions: 1 - North, 2 - East, 3 - South, 4 - West.
// First coordinate is current graph, second is next graph. Result is the compass direction. 
//Example: better_map_of_directions[1-1][8-1] = 1 - direction you need to get from 1 to 8 is 1 (North)
const uint8_t better_map_of_directions[20][20] = { 
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

int paths_matrix[24][8]={ //List of paths from two pairs of nodes
  {0,0,0,0,0,0,0,0},
  {1,8,9,4,9,0,0,0},
  {1,8,12,13,14,5,14,0},
  {1,8,12,13,18,6,18,0},
  {1,8,12,19,16,17,7,17},
  {3,11,10,9,4,9,0,0},
  {3,11,15,14,5,14,0,0},
  {3,11,15,14,13,18,6,18},
  {3,11,15,20,17,7,17,0},
  {4,9,8,1,8,0,0,0}, //9
  {4,9,10,11,3,11,0,0},
  {5,14,13,12,8,1,8,0},
  {5,14,15,11,3,11,0,0},
  {6,18,13,12,8,1,8,0},
  {6,18,13,14,15,11,3,11},
  {7,17,16,19,12,8,1,8},
  {7,17,20,15,11,3,11,0},
  {1,8,9,10,2,10,0,0},
  {3,11,10,2,10,0,0,0},
  {4,9,10,11,15,14,5,14}, //19
  {5,14,13,18,6,18,0,0},
  {6,18,16,17,7,17,0,0},
  {7,17,20,15,11,10,9,4}, // Might cause problems as [current_graph_number+1] undefined for final node
  {2,10,9,4,9,0,0,0}
};

uint8_t current_graph_number = 0; // WIll be updated every time juuction is passed
uint8_t current_compass = 1; // In default situation starts from going to the North
bool this_is_the_end = false; // Becomes true when we reach final destination and need to reverse or go backwards
bool part_two = false; // Becomes true when robot has completed first run
unsigned long time_of_last_junction_detected;

bool moving;  // True if moving, for flashing LED.
uint8_t current_path[] = {2,10,9,4,9,0,0,0}; // Initial path
uint8_t bay_array[] = {4,5,6,7,2}; // Bays that we need to visit
uint8_t current_bay_number = 0; 

const uint8_t distance_history_length = 10;
float distance_history[distance_history_length];  // Stores the last few readings of the distance sensor, for averaging. (queue structure)
uint8_t distance_history_pointer = 0; // points to head of the queue.
uint8_t distance_history_datapoints = 0; // increases until full

// Comparing latest value to the average:
// robot max speed around 20cm/s, but will be less when going into a bay.
// max deviation = fudge_factor * ((num_samples/2) * max_distance_per_sample (assuming constant speed))
const float max_acceptable_deviation = 3 * (0.5 * distance_history_length * 20 * delay_time / 1000);
float current_wall_distance = 0; // Use this for all distance measurements, updated only when value is acceptable.

int16_t v_left_prev = 0; // Needed for faster movement (we do not update speed if it is the same)
int16_t v_right_prev = 0; // Needed for faster movement (we do not update speed if it is the same)
int8_t direction_left_prev = 0; // 1 is forwards, 2 is backwards (never used, we use other variable "this_is_the_end"), 0 is not yet defined.
int8_t direction_right_prev = 0;

void new_path_define(int x);
void new_path(int big_goal_graph);
void move(int16_t speed, float rotation_fraction);
void stop();
void button_press_ISR();
void straight_junction();
void simple_mode_of_motion();
void left_junction();
void right_junction();
void backwards();
void straight();
void backwards_left_junction();
void backwards_right_junction();
void stop_and_grab();
void stop_and_release();
void color_detection();
int spike_in_distance();
bool junction_detected();

// We could get rid of it and describe logic of speeds switching with v_left_prev && v_right_prev, 
//but this check (using current_speed and current_rot_frac) is supposed to speed up the process of checking change of speed
int16_t current_speed = 0; 
float current_rot_frac = 0; 


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

void measure_distance(){
  // Take distaance reading with ultrasonic sensor.
  distance_history_pointer = (distance_history_pointer + 1) % distance_history_length;
  distance_history[distance_history_pointer] = (analogRead(sensityPin) / ADC_SOLUTION) * MAX_RANG ;
  if (distance_history_datapoints < distance_history_length){
    distance_history_datapoints ++; // increase unless full
  }
}

void flash_led(){
  // Flash LED if moving
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

void stop(){
  // Stop the car by releasing both motors
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  moving = false;
}


void button_press_ISR(){
  // Debounce button
  unsigned long new_time = millis();
  if (millis() > (first_press_time + 300)){
    if (mode == 0) {
      mode = 1; 
    } else {
      reset();  // To stop the robot, reset arduino.
    }
    first_press_time = new_time;
  }
}




void new_path_define(int x){
  // Update to a new path; Used when current path is completed by car.
	for (int i = 0; i < 8; ++i) {
	  current_path[i]=paths_matrix[x][i];
	}
}

void new_path(int big_goal_graph){
  int x = current_path[current_graph_number];
	//Hard coding paths between nodes
  // Update the path according to:
  // 1. Next goal graph big_goal_graph
  // 2. Current position x 
  // bays in the first run, or previous goal graph in the second run, where cubes are not found
	if (x==1){
		if (big_goal_graph==4){
			new_path_define(1);
		}else if (big_goal_graph==5){
			new_path_define(2);
		}else if (big_goal_graph==6){
			new_path_define(3);
		}else if (big_goal_graph==7){
			new_path_define(4);
    }else if (big_goal_graph==2){
      new_path_define(17);
		}else{
			new_path_define(0);
		}}
	if (x==3){
		if (big_goal_graph==4){
			new_path_define(5);
		}else if (big_goal_graph==5){
			new_path_define(6);
		}else if (big_goal_graph==6){
			new_path_define(7);
		}else if (big_goal_graph==7){
			new_path_define(8);
    }else if (big_goal_graph==2){
      new_path_define(18);
		}else{
			new_path_define(0);
		}}
	if (x==4){
		if (big_goal_graph==1){
			new_path_define(9);
		}else if (big_goal_graph==3){
			new_path_define(10);
		}else if (big_goal_graph==5){
			new_path_define(19);
		}else{
			new_path_define(0);
		}}
	if (x==5){
		if (big_goal_graph==1){
			new_path_define(11);
		}else if (big_goal_graph==3){
			new_path_define(12);
		}else if (big_goal_graph==6){
			new_path_define(20);
		}else{
			new_path_define(0);
		}}
	if (x==6){
		if (big_goal_graph==1){
			new_path_define(13);
		}else if (big_goal_graph==3){
			new_path_define(14);
		}else if (big_goal_graph==7){
			new_path_define(21);
		}else{
			new_path_define(0);
		}}
	if (x==7){
		if (big_goal_graph==1){
			new_path_define(15);
		}else if (big_goal_graph==3){
			new_path_define(16);
		}else if (big_goal_graph==4){
			new_path_define(22);
		}else{
			new_path_define(0);
		}
	}
}

void move(int16_t speed, float rotation_fraction) {
  // This function describes every possible motion configuration in the most convinient (I think) way.
  // Speed that you give is the maximal speed of two wheels. 
  // Rotation fraction shows to which extent you rotate. 
  // rotation fraction = 0 => you go forward or backwards. 
  // Rotation fraction 1 means you move clockwise, -1 means you move anticklokwise.

  // It is convinient as it is rotation_fraction only that is responsible for geometry of movement 
  // This is the only function (except stop function) that sets motors speeds
  // This function renews motors speeds only if they were actually changed (it speeds the car)

  int16_t other_speed = 0; // Not maximal speed (in absolute value) calculated locally
  int16_t v_left = 0;
  int16_t v_right = 0;
  int8_t direction_left = 0;  
  int8_t direction_right = 0;
  moving = true;

  if (speed != current_speed || current_rot_frac != rotation_fraction) {

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
    if (v_left != v_left_prev){
      myMotor1->setSpeed(abs(v_left)); 
      v_left_prev = v_left;
    }
    if (v_right != v_right_prev){
      myMotor2->setSpeed(abs(v_right));
      v_right_prev = v_right;
    }
    
    if (v_left > 0){
      direction_left = 1;
    } else{
      direction_left = 2;
    }
    if (v_right > 0){
      direction_right = 1;
    } else{
      direction_right = 2;
    }

    if (mode != 0) {
      if (direction_left != direction_left_prev){
        //DEBUG_SERIAL.print(".");
        if (v_left > 0) {
          myMotor1->run(FORWARD);
        } else {
          myMotor1->run(BACKWARD);
        }
        direction_left_prev = direction_left;
      }
      if (direction_right != direction_right_prev){
        //DEBUG_SERIAL.print(".");
        if (v_right > 0) {
          myMotor2->run(FORWARD);
        } else {
          myMotor2->run(BACKWARD);
        }
        direction_right_prev = direction_right;
      }
      //DEBUG_SERIAL.println("");
    } else {
        stop();
    }
    current_rot_frac = rotation_fraction;
    current_speed = speed;
  }
}

void last_bay(){ // Function that is responsible for "junction" between graph 2 and graph 10 when car is supposed to stop in the initial bay and wait
  straight_junction(); // Ends when white line is over (line near graph 2)
  measure_distance();
  while (distance_history[distance_history_pointer] > distances_from_bays[1]){ // Distance is located in distance_from_bays[2-1] as it is distance near bay 2
    delay(1);
    measure_distance();
  }
  stop();
  delay(5000);	//Stops for 5s
  this_is_the_end = true; 
  current_compass = 1; //change compass by 180 degrees or from 3 to 1
  move(-main_speed, 0.0); // Needed to ignore sensors
  delay(1500); // Needed to get out of the line near graph 2 and not have problems with unnecessary junctions being detected
  new_path_define(23);
  current_graph_number = 0; // Because technically we are doing a "junction" at the graph 2, and graph two is the first element of new path array
  current_bay_number=0; // As we are starting a new circle of blocks colection
  part_two = true; // We have collected all blocks and now bays may have no blocks
  
}

void straight_junction(){ // This function must go on as long as you are in the junction
  bool sensor_sequence[] = {1,0,1}; // Needed for reverse 
  if (this_is_the_end == false){ // Classical straight juunction
    DEBUG_SERIAL.println("Go straight in junction");
    while ((digitalRead(sensorFarRight) == 1) || (digitalRead(sensorFarLeft) == 1)) {
      straight(); 
    }
    // Previous part was the case of simple straight junction when we are supposed to just skip the turning and go straight
    // Code below is initial version of turning in the junction after dropping the block
    // However, junctions are very close to buildings, so we decided to rotate by 180 degrees closer to the dropping point 
    // Therefore code below turned to be unnecessary (such situation just never happends) 

  } else if(current_path[current_graph_number] == 8 || current_path[current_graph_number] == 1) { // Turn 180 degrees anticlockwise at a T-junction, i.e. stop turning after crossing second white line.
    move(-main_speed, -1);
    for (int i = 0; i < 3; i++){
      while (digitalRead(sensorLeft) == sensor_sequence[i]) {
        delay(10); // delay in case of "bounce" in line sensor readings.
      }
    }
    this_is_the_end = false;
    for (int i = 0; i < 500/delay_time; i ++) {
        straight();
        delay(delay_time);
      }
    
  } else {  // Turn 180 degrees clockwise at a T-junction, i.e. stop turning after crossing second white line.
    DEBUG_SERIAL.println("Turning 180");
    if (current_path[current_graph_number] == 2) { // Means initial bay. You must never enter this part of code
      delay(500);
    } else {
      move(-main_speed, 1);
      for (int i = 0; i < 3; i++){
        while (digitalRead(sensorRight) == sensor_sequence[i]) {
          delay(10); // delay in case of "bounce" in line sensor readings.
        }
      }
      this_is_the_end = false;
      for (int i = 0; i < 500/delay_time; i ++) {
          straight();
          delay(delay_time);
        }
    }

  }
  delay(delay_time);
}

void simple_mode_of_motion(){
  // Determine the mode of motion to take using:
  // 1. Current graph (junction that was just reached by car)
  // 2. Next graph (next junction to go once the current graph is reached)
  // 3. Current compass and next compass is used to determine where to turn

  // Navigation debug system
  int y = better_map_of_directions[current_path[current_graph_number]-1][current_path[current_graph_number+1]-1];
  DEBUG_SERIAL.print("Current graph: ");
  DEBUG_SERIAL.println(current_path[current_graph_number]);
  DEBUG_SERIAL.print("Next graph: ");
  DEBUG_SERIAL.println(current_path[current_graph_number + 1]);
  DEBUG_SERIAL.print("Goal compass: ");
  DEBUG_SERIAL.println(y);
  DEBUG_SERIAL.print("Current compass: ");
  DEBUG_SERIAL.println(current_compass);

  // Junction actions to take according to different conditions

  // Id current junction is 19 or 20, it means that you need to skip two graphs instead of one, so for one of these supposed "junctions" you need to do nothing and go to next one immediately
  if (current_path[current_graph_number] == 19 || current_path[current_graph_number] == 20 || ((current_path[current_graph_number] == 8 && current_path[current_graph_number-1] == 1) || (current_path[current_graph_number] == 11 && current_path[current_graph_number - 1] == 3)) && (y == 1)){
    DEBUG_SERIAL.println("Skip this junction");
    if ((current_path[current_graph_number] == 8 || current_path[current_graph_number] == 11) && (y == 1)) { 
      // If current graph numbers correspond to dropping points and you have already dropped the blocks, 
      // do nothing as you have already done corresponding junction action
      // Theoretically could be replaced with straight_junction() or be not even checked (this junction would fall to ctiteria of straight_junction)
      for (int i = 0; i < 500/delay_time; i++) {
        straight();
        delay(delay_time);
      }
    }
  } else if (current_path[current_graph_number] == 2 && current_compass == 3) { // We need to stop for five seconds and start new round
    last_bay();
  } else if ((4 + y - current_compass) % 4 == 3) { // Turn left in junction
    DEBUG_SERIAL.print("Left junction is started... ");
    left_junction();
    DEBUG_SERIAL.println("Left junction is done!!!");
  } else if ((4 + y - current_compass) % 4 == 1) {// Turn right in junction
    DEBUG_SERIAL.print("Right junction is started... ");
    right_junction();
    DEBUG_SERIAL.println("Right junction is done!!!");
  } else if ((4 + y - current_compass ) % 4 == 0) { // Go straight in junction
    straight_junction();
  } else { // Go backwards at the point of getting the block or dropping the block 
    this_is_the_end = true;
    if ((current_path[current_graph_number] != 1 && current_path[current_graph_number] != 3)){ // Get the block
      DEBUG_SERIAL.print("NOW WE GO BACKWARDS");
      DEBUG_SERIAL.println(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION);
      stop_and_grab();
      DEBUG_SERIAL.println("Picking up a block...");
      color_detection(); // At the end of this function new path is defined
      DEBUG_SERIAL.println(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION);
      DEBUG_SERIAL.print("NOW WE GO BACKWARDS");
    } else { 
      stop_and_release(); // At the end of this function new path is defined
      // stop and release the block
      DEBUG_SERIAL.println("At the end, releasing block...");
      } 
    }

}

void left_junction(){ // This function must go on as long as you are in the junction
  if (this_is_the_end == false) { // Simply go left
    
    if (current_path[current_graph_number] == 8 || current_path[current_graph_number] == 11 || current_path[current_graph_number] == 12 || current_path[current_graph_number] == 15) {
      // These graphs are problematic (you are close to hitting the wall) and we must not do the smooth turning
      move(main_speed, -1.0); 
    }
    else {
      // Smooth turning
      move(main_speed, -0.7); // Rotation fraction can be changed according to actual performance
    }

    // process of turning determination; involves Left sensor detects whether it has been crossing the destination line of turning left
    while (digitalRead(sensorLeft) == 1) {}
    while (digitalRead(sensorLeft) == 0) {} // While right sensor is outside of its first line, move it to the line
    while (digitalRead(sensorLeft) == 1) {}

    //Extra Delay added for the car to deal with overshoot problem
    move(main_speed, 0.7); // Rotation fraction can be changed according to actual performance
    delay(30); // delay time can also be changed

    
    if (true){ // After certain time do not detect new junctions and follow straight line
      for (int i = 0; i < 500/delay_time; i++) {
        straight();
        delay(delay_time);
      }
    }
  } else {
    // You need to go left, but you are moving in a backwards direction
    backwards_left_junction(); 
  }
}

void right_junction(){ // This function must go on as long as you are in the junction
  if (this_is_the_end == false) {
    // These graphs are problematic (you are close to hitting the wall) and we must not do the smooth turning
    if (current_path[current_graph_number] == 8 || current_path[current_graph_number] == 11 || current_path[current_graph_number] == 12 || current_path[current_graph_number] == 15) {
      move(main_speed, 1.0);
    }
    else {
      // Go smooth
      move(main_speed, 0.7);
    }
    // process of turning determination; involves Right sensor detects whether it has been crossing the destination line when turning right
    while (digitalRead(sensorRight) == 1) {}
    while (digitalRead(sensorRight) == 0) {}
    while (digitalRead(sensorRight) == 1) {} 

    //Extra Delay added for the car to deal with overshoot problem
    move(main_speed, 0.7);
    delay(30);
    if (true){ // After certain time do not detect new junctions and follow straight line
      for (int i = 0; i < 500/delay_time; i++) {
        straight();
        delay(delay_time);
      }
    }

  } else {
    backwards_right_junction();
  }
}


void backwards(){ // It is not a junction function!!! 
// Code to follow the line when going backwards
// Version of straight()
  bool right = digitalRead(sensorRight);
  bool left = digitalRead(sensorLeft);  
  if (right && !left) { //M fgf ove left
  // rotate fraction it set to a small value to ensure robot corrects iteself instead of purely rotating
    move(-main_speed, 0.1); // Anti-Clockwise
  } else if (!right && left) { //Move to the right
    move(-main_speed, -0.1); // Clockwise
  } else if (!right && !left) { // Includes both going 
    move(-main_speed, 0);
  } else { // Both are white, so we need time delay and going straightforward for short period of time ignoring all sensors. 
    for (int i = 0; i < 125/delay_time; ++i) {
      move(-main_speed, 0);
      delay(delay_time);
    }
  }
}

void straight(){ // Regular function for going straightforward
// Not a junction function! Needed to get from one junction to another
  uint8_t sp = main_speed; 
  if (number_of_connections[current_path[current_graph_number]-1] == 1 && (current_path[current_graph_number] != 2) && (current_path[current_graph_number] != 1) && (current_path[current_graph_number] != 3)) {
    sp = 200; // Slow down for better distance control
  }
  bool right = digitalRead(sensorRight);
  bool left = digitalRead(sensorLeft);
  if (this_is_the_end == false) {
    // Rotation fraction here is 0.35 for smooth line following
    // Do not drop it lower than 0.3 as you will not git into corner at graphs 19 and 20
    // For some reasons if this rotation fraction is bigger than 0.7, it also does not fit into corner
    // So, keep this value between 0.3 and 0.7
    if (right && !left) { //Move right
      move(sp, 0.35); // No need to be rapid, so turning is quite smooth
    } else if (!right && left) { //Move to the left
      move(sp, -0.35); // No need to be rapid, so turning is quite smooth
    } else if (!right && !left) { // Includes both going 
      move(sp, 0.0);
    } else { // Both are white, so we need time delay and going straightforward for short period of time ignoring all sensors. 
    // Used when your front sensors have detected a junction and backwards will do it after certain time 
    // This time delay does not bring the robot to the moment of junction detection
    // But it is needed to ignore going to the sides when you do not need to go to the sides
      for (int i = 0; i < 125/delay_time; ++i) {
        move(sp, 0);
        delay(delay_time);
      }
    }
  } else {
    backwards();
  }
}

void backwards_left_junction(){ // Rotate clokwise until certain results
  DEBUG_SERIAL.print("You have entered the left junction");

  float rot = 0.75; // Smooth turning

  // another way of implementing turning while detecting if the turning process is completed.
  while (digitalRead(sensorRight) == 1) { 
    move(main_speed, rot);
    delay(delay_time);
  }
  delay(5);
  while (digitalRead(sensorRight) == 0) {
    move(main_speed, rot);
    delay(delay_time);
  }
  delay(5);
  while (digitalRead(sensorRight) == 1) {
    move(main_speed, rot);
    delay(delay_time);
  }

  // You don't need to take backward action ubtil you go away from current junction
  this_is_the_end = false;
  time_of_last_junction_detected = millis();
  while (millis() - time_of_last_junction_detected < 500) {
    straight();
  }
}

void backwards_right_junction(){ // Rotate anticlockwise until certain reusult. 
  
  float rot = -0.75;

  DEBUG_SERIAL.print("You have entered the right junction");
  // another way of implementing turning while detecting if the turning process is completed.
  while (digitalRead(sensorLeft) == 1) { 
    move(main_speed, rot);
    delay(delay_time);
  }
  delay(5);
  while (digitalRead(sensorLeft) == 0) {
    move(main_speed, rot);
    delay(delay_time);
  }
  delay(5);
  while (digitalRead(sensorLeft) == 1) {
    move(main_speed, rot);
    delay(delay_time);
  }
  // You don't need to take backward action ubtil you go away from current junction
  this_is_the_end = false;
  time_of_last_junction_detected = millis();
  while (millis() - time_of_last_junction_detected < 500) {
    straight();
  }
}

void stop_and_grab(){
  // actions to take:
  // 1. stop the car from moving
  // 2. tell servo to grab the block
  stop();
  DEBUG_SERIAL.println("grabbing block");

  // Fast movement of grabbing hand at first before actual grabbing, which saves time
  int pos = 270;
  
  // slow the servo down to grab the block to make grabbing process not screwed up by hurrying
  for (pos; pos >=5; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    // DEBUG_SERIAL.println("current pos: " + String(pos));
    mech_servo.write(pos);              // tell servo to go to position in variable 'pos'
    if (pos > 45){
      delay(5);
    }else{
      delay(15);
    }
  }

  DEBUG_SERIAL.println("Block was grabbed");
}


void stop_and_release(){
  // actions to take:
  // 1. stop the car from moving
  // 2. tell servo to release the block
  stop();

  // fast release. The position value is renewed fastly, but the servo isn't that fast in real testing
  // the servo then tried the match the position AFTER the pos value is renewed to 270
  // this suprisingly works well when releasing as it saves time by taking most of hand's position reset motion to backwarding process; no need to extra coding in backward motion
  for (int pos = 5; pos <= 270; pos += 44) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    mech_servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }

  // The LED color indicator is dimmed
  digitalWrite(LED_Red, 0);
  digitalWrite(LED_Green, 0);

  // Taking another new path by updating a new current bay number to take
  if (current_path[current_graph_number]==7){
	  current_bay_number=0;
  }else{
  	current_bay_number = current_bay_number + 1;
  }
  new_path(bay_array[current_bay_number]); // Set new path
  current_graph_number = 0; // You are doing the first element's junction in this new path

  int y = better_map_of_directions[current_path[current_graph_number]-1][current_path[current_graph_number+1]-1];
  if (y == 1 && current_path[current_graph_number] != 5 && current_path[current_graph_number] != 7) { 
    move(-main_speed, 0);
    delay(400);
    straight_junction();
  }
}


void color_detection() {
  delay(500);
  uint16_t r, g, b, c, colorTemp, lux;
  unsigned long start_time = millis();

  while (millis() - start_time < 3000){

    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    lux = tcs.calculateLux(r, g, b);

    if (lux > 4000) {
      DEBUG_SERIAL.println("red");
      new_path(3);
      current_graph_number = 0;
      digitalWrite(LED_Red, 1);
      delay(5000);
      return;
    } else if (lux < 4000 && colorTemp > 10000) { // change to less than 4000?
      DEBUG_SERIAL.println("black");
      new_path(1);
      current_graph_number = 0;
      digitalWrite(LED_Green, 1);
      delay(5000);
      return;
    } else {
      //DEBUG_SERIAL.println("no block");
      // add wait 10 seconds then decide no block
      delay(100);
    }
  }
  if (part_two){
	stop_and_release();
  }
  else{
  	DEBUG_SERIAL.println("can't tell colour, guess black");
  	new_path(1);
  	current_graph_number = 0;
  	digitalWrite(LED_Green, 1);
  	//digitalWrite(LED_Red, 1);
  	delay(5000);
  }
  
}

int spike_in_distance(){
  // Calculates if the ultrasonic sensor reading has spiked due to it seeing the cube instead of the wall.
  // Returns +1 is distance has increased significantly, -1 if decreased, and 0 otherwise.
  // length of distance history and sample period must be set to optimal value:
  //  If either of these are too short, by the time the code checks, may have been changed for a while.
  //  If too long, when correcting, may not see two changes occuring within a short time period.
  float average = 0;
  float recent_average = 0;
  for (int i = 0; i < distance_history_datapoints; i++){
    average += distance_history[i];
  }
  average /= distance_history_datapoints; // calculate mean
  current_wall_distance = distance_history[distance_history_pointer];
  if ((current_wall_distance - average) > max_acceptable_deviation){
    // Distance has suddenly increased, need to reset the history so that further changes can be detected
    distance_history[0] = current_wall_distance; // use last value as first in the reset history
    distance_history_pointer = 1;
    distance_history_datapoints = 1;
    return 1;
  } else if ((current_wall_distance - average) < -max_acceptable_deviation){
    // Distance has decreased significantly, same process as above.
    distance_history[0] = current_wall_distance; // use last value as first in the reset history
    distance_history_pointer = 1;
    distance_history_datapoints = 1;
    return -1;
  } else{
    return 0;
  }
  
}

bool junction_detected(){
  //is_a_bay is function that checks if you need to check distance from the wall 
  // is_a_bay is always true when you need to stop before building
  // And also true when you are about to hit the wall and need to detect a junction to avoid it
  bool is_a_bay = ((number_of_connections[current_path[current_graph_number]-1] == 1) && (current_path[current_graph_number] != 2) && (current_path[current_graph_number] != 1) && (current_path[current_graph_number] != 3) 
  || (current_path[current_graph_number] == 16 && current_compass == 1) 
  || (current_path[current_graph_number] == 8 && current_compass == 4) 
  || (current_path[current_graph_number] == 12 && current_compass == 4) 
  || (current_path[current_graph_number] == 16 && current_compass == 1) 
  || (current_path[current_graph_number] == 15 && current_compass == 2) 
  || (current_path[current_graph_number] == 11 && current_compass == 2));  // (and not the starting point)
  // Junction is detected in several cases: 
  // 1) You have detected something using far sensors (far right or far left)
  // 2) You can hit the wall and wall is close ("close" is a value from distances_from_bays[current_path[current_graph_number]-1])
  // 3) You are about to drop the block and both your front sensors light (you have hit red or green panel))
  if (((digitalRead(sensorFarRight) || digitalRead(sensorFarLeft))) || (is_a_bay && current_wall_distance < distances_from_bays[current_path[current_graph_number]-1]) || ((current_path[current_graph_number] == 1 || current_path[current_graph_number] == 3) && digitalRead(sensorLeft) && digitalRead(sensorRight))) {
    return true;
  } else {
    return false; 
  }
}


void setup() {
  
  pinMode(NINA_RESETN, OUTPUT);
  digitalWrite(NINA_RESETN, LOW);
  DEBUG_SERIAL.begin(115200); // Start DEBUG_SERIAL communication
  DEBUG_SERIAL.println("Starting...");

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
  mech_servo.attach(servo_pin);
  mech_servo.write(270); // start vertically, angle is anticlockwise from just past the closed position.
  pinMode(loop_speed_test_pin, OUTPUT);
  digitalWrite(loop_speed_test_pin, 0);
  

  // Initialize the Motor Shield
  if (!AFMS.begin()) {
    DEBUG_SERIAL.println("Motor Shield not found.");
    while (1); // Halt if shield not found
  }
  DEBUG_SERIAL.println("Motor Shield initialized.");

  // Button Interrupt:
  attachInterrupt(digitalPinToInterrupt(button), button_press_ISR, RISING);

  // Timer Interrupt for LED flashing:
  // Timer A (used as clock for B) clocked at 250kHz.
  TCB0.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode
  TCB0.CCMP = 62500; // Value to compare with. 62500 gives 2Hz.
  TCB0.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer

  // Timer interrupt for taking distance readings:
  //TCB1.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode
  //TCB1.CCMP = 125 * ULTRASONIC_SAMPLE_PERIOD; // Value to compare with. 125 gives 1ms
  //TCB1.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
  //TCB1.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer

  current_compass = 1;
}

void loop() {
  bool farLeft = digitalRead(sensorFarLeft);
  bool left = digitalRead(sensorLeft);
  bool right = digitalRead(sensorRight);
  bool farRight = digitalRead(sensorFarRight);

  if (mode != 0) {  
    measure_distance(); 
    current_wall_distance = distance_history[distance_history_pointer];
    straight();
    if (junction_detected()){ // When junction is detected, we need to 1) Do the junction to certain side, 2) Change the compass and 3) Change certain graph and 
      Serial.println(current_path[current_graph_number]);
      time_of_last_junction_detected = millis();
      simple_mode_of_motion(); // This function does corresponding turn and ends when the junction is done
      current_compass = better_map_of_directions[current_path[current_graph_number]-1][current_path[current_graph_number + 1]-1]; // Update compass
      current_graph_number = current_graph_number + 1; // Because finished simple_mode_of_motion means that we have gone through the graph and we need to get to the new graph
    }
    delay(delay_time);
    digitalWrite(loop_speed_test_pin, 1);
    digitalWrite(loop_speed_test_pin, 0);
  } else {
    stop();
  }


}
