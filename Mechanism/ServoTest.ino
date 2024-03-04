// attach brown to ground, red to 5V, orange to pin 9

#include <Servo.h>
#include "Colour_Detection_Test.h"
Servo myservo; // create servo object to control a servo
int pos = 0; // variable to store the servo position
int grab = 0; // when grab = 1, contract the mechanism; when grab = 0, release
int pos_max = 30; //set desired maximum rotation

void setup() {
  myservo.attach(9); // use pin 9
}

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
/*
void loop() { // test code, integrate with rest of code
  if (grab == 1) {
    contract();
    while(grab == 1) {
      delay(2000);
      grab = 0;
    } // Wait 2s then grab is set to 0
  } else {
    release();
    while(grab == 0) {
      delay(4000);
      grab = 1;
    } // Wait 4s then grab is set to 1
  }
} */


String mechanism() {
  contract();
  String a = block_detect();
  if (a == "no block") {
    release(); }
  return a; 
}
