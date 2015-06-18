//Serial 1 for communication between Flora's
#define mySerial Serial1

//Include necessary libraries
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
//Include Modified Library
#include <Adafruit_LSM9DS0_V2.h>
#include <Adafruit_Simple_AHRS.h>


//IMU SECTION
// Create LSM9DS0 board instance of modified version.
Adafruit_LSM9DS0_V2 lsm(1000);  // Use I2C, ID #1000
// Attitude and Heading Reference System
// Create AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

// Function to configure the sensors on the LSM9DS0 board.
void configureLSM9DS0(void)
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

//Custom Variables
int incomingByte;
int flexSensorPin = 10; //analog pin D10
int sensorValue;
int rollS2;



void setup() {
  mySerial.begin(9600);
  
     //Initialize IMU board - LSM9DS0
    if(!lsm.begin())
    {
    // Check for errors
    //Serial.print(F("Error"));
    while(1);
    }
        //If all went well, 
    //Setup the sensor gain and integration time.
    configureLSM9DS0();


}

void loop() {
   sensorValue = analogRead(flexSensorPin);
   
    // Refresh IMU data
  sensors_vec_t   orientation;
   //IMU SECTION
  if (ahrs.getOrientation(&orientation))
  {
     
  //The way the sensor on the neck is oriented, it only needs the heading angle
  //Set orientation heading as the heading value for sensor S1 (Neck sensor)
    rollS2 = int(orientation.roll);
   
   //Serial.println(rollS2);
  }
    // print the results (cvt to string):
    mySerial.print(sensorValue);
    mySerial.print(",");
    mySerial.println(rollS2);   
    
    delay(100);
   
 }
