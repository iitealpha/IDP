#include <SparkFunLSM6DS3.h>

//#include <SPI.h>
//#include <Wire.h>  
//reference: https://arduino.stackexchange.com/questions/78986/arduino-nano-33-iot-lsm6ds3-get-gyro-angle-in-degrees

//#include <SPI.h>
//#include <Wire.h>  
#include <Arduino_LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <Math.h>

float pitchFilteredOld;
float rollFilteredOld;
float yawFilteredOld;
float yaw = 0;
//Magwick filter;
const float sensorRate = 104.00;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  if(!IMU.begin())  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  //filter.begin(sensorRate);
  //Serial.println("Setup complete!");
}  
void loop() {
  float xAcc, yAcc, zAcc;
  float xGyro, yGyro, zGyro;
  //float roll, pitch, yaw;
  
  if(IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){
    IMU.readAcceleration(xAcc, yAcc, zAcc);
    IMU.readGyroscope(xGyro, yGyro, zGyro); 
    
    
    //filter.updateIMU(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);
    


    yaw = 0.98 *(yaw+zGyro) + 0.02*zAcc;




    // pitch = filter.getPitch();
    // float pitchFiltered = 0.1 * pitch + 0.9 * pitchFilteredOld; // low pass filter
    // Serial.println("pitch: " + String(pitchFiltered));
    // pitchFilteredOld = pitchFiltered;

    //roll = filter.getRoll();
    //pitch = filter.getPitch();
    //yaw = filter.getYaw();
    //float rollFiltered = 0.1 * roll + 0.9 * rollFilteredOld; // low pass filter
    //float pitchFiltered = 0.1 * pitch + 0.9 * pitchFilteredOld; // low pass filter
    //float yawFiltered = 0.1 * yaw + 0.9 * yawFilteredOld; // low pass filter
    
    
    // if the difference between the new and old value is less than 0.3, then the new value is the old value
    //if(abs(rollFiltered-rollFilteredOld) <= 0.3){
    //  rollFiltered = rollFilteredOld;
    //}
    //if(abs(pitchFiltered-pitchFilteredOld) <= 0.3){
    //  pitchFiltered = pitchFilteredOld;
    //}
    //if(abs(yawFiltered-yawFilteredOld) <= 0.3){
    //  yawFiltered = yawFilteredOld;
    //}
//
    //pitchFilteredOld = pitchFiltered;
    //rollFilteredOld = rollFiltered;
    //yawFilteredOld = yawFiltered;
    
    //Serial.println("roll: " + String(rollFiltered) + " pitch: " + String(pitchFiltered) + " heading: " + String(yawFiltered));
    Serial.println("Yaw: " + String(yaw));
  }
}