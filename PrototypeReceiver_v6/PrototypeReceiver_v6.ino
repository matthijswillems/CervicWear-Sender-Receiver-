#include <SoftwareSerial.h>

//Serial 1 for communication between Flora's
//#define mySerial Serial1


SoftwareSerial mySerial(10, 9);



//Include all necessary libraries
#include <SPI.h>
#include <Adafruit_BLE_UART.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>

//BLUETOOTH SECTION
#define ADAFRUITBLE_REQ 6
#define ADAFRUITBLE_RDY 1     // This should be an interrupt pin
#define ADAFRUITBLE_RST 12

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

//IMU SECTION
// Create LSM9DS0 board instance.
Adafruit_LSM9DS0 lsm(1000);  // Use I2C, ID #1000
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

//CUSTOM VARIABLES
const int ledPin = 7; // the pin that the LED is attached to
int incomingByte;      // a variable to read incoming serial data into
String rollS1;
long startTime;
String firstValue;
String secondValue;


String content = "";
char character;
boolean firstContact = false;


//SETUP
void setup() {
    //Serial.begin(9600);

     //Initialize BLE board - NRF8001
    BTLEserial.setDeviceName("CERVAPP"); /* 7 characters max! */
    //the order in which the below statements are declared doesn't matter
    BTLEserial.begin();
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


aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop() {
  //mySerial.begin(9600); 
  //IF I UNCOMMENT THIS, I WILL GET SERIAL DATA, BUT CANNOT CONNECT TO BTLE
  
  while(mySerial.available()){
        //Serial.println("available");
    // read the serial buffer:
  String myString = mySerial.readStringUntil('\n');

  // check length of string
  if (myString.length()>0) {

    myString.trim(); //Strim any whitespace characters
    //just to be sure
    
      int commaIndex = myString.indexOf(',');
      //not sending a second comma, maybe in later stage if I need
      //roll,pitch and heading data in the project.
      // int secondCommaIndex = myString.indexOf(',', commaIndex+1);
      
      //store data
     firstValue = myString.substring(0, commaIndex);
     secondValue = myString.substring(commaIndex+1, myString.length());
      
    Serial.println(firstValue+" "+secondValue);
    //Serial.println(firstValue);
    //Serial.println(secondValue);
    }
    // Send an A to ask for more:

    //delay(20);
//    mySerial.end();


  }
       
  //Need to end mySerial in order for NRF8001 to work correctly
  //As I'm sharing the Interrupt TX Pin (INT1);
  //Refresh nRF8001 data
  BTLEserial.pollACI();
  // Refresh IMU data
  sensors_vec_t   orientation;
   //IMU SECTION
  if (ahrs.getOrientation(&orientation))
  {
     
  //The way the sensor on the neck is oriented, it only needs the roll angle
  //Set orientation roll as the roll value for sensor S1 (Neck sensor)
    rollS1 = String(int(orientation.roll));

   
  // Serial.println(headingS1);
  }
  
   //BLUETOOTH SECTION 
  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  //Serial.println(status); // voor debuggen
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
        startTime = millis();
        
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }
  
  //if connected, send data
  if (status == ACI_EVT_CONNECTED) {
    
        //Build in delay, without actually delaying program
        //Needed time for BTLE to send to Cervicapp witout Pipe Errors
        
          if((millis()-startTime)>500){
        //check if all data received
     
        sendDataOverBluetooth();
        startTime+=500;
          }
        
  }
  
}
  
void sendDataOverBluetooth(){

  //create data packages to send
  
  uint8_t sendPackage1[20];
  uint8_t sendPackage2[20];
  uint8_t sendPackage3[20];

  
  //parse data to stringpackage
  //check if length of each string is 3 characters
  //sometimes extra characters are found (despite trimming)
  //causing the serial data to not arrive properly
  //in which case I don't want to update/send anything
  //if(rollS1.length()==3 && firstValue.length()==3 && secondValue.length()==3){
  String s1package = "S1="+rollS1;
  String s2package = "S2="+secondValue;
  String flxpackage = "FLX"+firstValue;
  //Serial.println(s1package+s2package+flxpackage);
  
    //max bytes is 20
    //characters won't be over 20, 
    //so won't need to break m up
  s1package.getBytes(sendPackage1, 20);
  BTLEserial.write(sendPackage1, char(s1package.length()));
  delay(20);
  s2package.getBytes(sendPackage2, 20);
  BTLEserial.write(sendPackage2, char(s2package.length()));
  delay(20);
  flxpackage.getBytes(sendPackage3, 20);  
  BTLEserial.write(sendPackage3, char(flxpackage.length()));
  delay(20);
  //}

}

