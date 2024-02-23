// attach brown to ground, red to 5V, orange to pin 9

#include <Servo.h>
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

void loop() {
  if (grab == 1) {
    contract();
    while(grab == 1) {
      delay(2000);
      grab = 0;
    } // Wait until grab is set to 0
  } else {
    release();
    while(grab == 0) {
      delay(4000);
      grab = 1;
    } // Wait until grab is set to 1
  }
}



/*
//May run 2 servo’s from ports 9, 10. Not tested yet

#include <Servo.h>
Servo myservo; // create servo object to control a servo
//Servo myservo1;
// twelve servo objects can be created on most boards
int pos = 0; // variable to store the servo position
void setup() {
 myservo.attach(9);
 //myservo1.attach(10); // attaches the servo on pin 9 to the servo object
}
void loop() {
 for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
 // in steps of 1 degree
 myservo.write(pos); // tell servo to go to position in variable 'pos'
 //myservo1.write(pos);
 delay(40); // waits 15 ms for the servo to reach the position
 }
 for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
 myservo.write(pos); // tell servo to go to position in variable 'pos'
 //myservo1.write(pos);
 delay(40); // waits 15 ms for the servo to reach the position
 }
}
*/
/*
#include <Servo.h>
Servo myservo; // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0; // variable to store the servo position
void setup() {
  myservo.attach(9);
}

void contract() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos); // tell servo to go to position in variable 'pos'

    delay(40); // waits 40 ms for the servo to reach the position
  }
}

void release() {
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos); // tell servo to go to position in variable 'pos'

    delay(40); // waits 40 ms for the servo to reach the position
  }
}

void loop() {
  contract();
  delay(1000);
  release();
  delay(1000);
}
*/
