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
