//Brown connector motors are lower speed - use with large wheels,
//Yellow connectors are higher speed - use with small wheels

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3, or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);
// You can also make another motor on port M2
// Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor1->setSpeed(150);
  myMotor2->setSpeed(150);
  myMotor3->setSpeed(150);
  myMotor4->setSpeed(150);
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(FORWARD);
  // turn on motor
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);
}

void loop() {
  uint8_t i;

  Serial.print("tick");

  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor1->setSpeed(255);
    myMotor2->setSpeed(255);
    myMotor3->setSpeed(255);
    myMotor4->setSpeed(255);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor1->setSpeed(255);
    myMotor2->setSpeed(255);
    myMotor3->setSpeed(255);
    myMotor4->setSpeed(255);
    delay(10);
  }
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(BACKWARD);
  myMotor4->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor1->setSpeed(255);
    myMotor2->setSpeed(255);
    myMotor3->setSpeed(255);
    myMotor4->setSpeed(255);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor1->setSpeed(255);
    myMotor2->setSpeed(255);
    myMotor3->setSpeed(255);
    myMotor4->setSpeed(255);
    delay(10);
  }
  Serial.print("tock");

  Serial.print("tech");
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);
  delay(1000);
}
