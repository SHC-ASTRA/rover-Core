// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <utility/imumaths.h>
#include <FastLED.h>
// Our own resources
#include "project/CORE.h"
#include "AstraMisc.h"
#include "AstraMotors.h"
#include "AstraCAN2.h"
#include "AstraSensors.h"
#include "TeensyThreads.h"

using namespace std;


#define NUM_LEDS 166
//strip 1: 1-40
//strip 2: 41-82
//strip 3: 83-124
//strip 4: 125-166
//CCW: 1,2,3,4

int led_rbg[3] = {0, 300, 0}; //When using multiple colors, use 255 max, when doing R/B/G use 800-900 for best brightness
int led_counter = 0;

CRGB leds[NUM_LEDS];


#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

SFE_UBLOX_GNSS myGNSS;

//Sensor declarations
Adafruit_BNO055 bno = Adafruit_BNO055();

//Setting up for CAN0 line
AstraFCAN Can0;

//AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Motor1(2, 1, false, 50, 1.00F);//Front Left
AstraMotors Motor2(4, 1, false, 50, 1.00F);//Back Left
AstraMotors Motor3(1, 1, true, 50, 1.00F);//Front Right
AstraMotors Motor4(3, 1, true, 50, 1.00F);//Back Right

AstraMotors motorList[4] = {Motor1, Motor2, Motor3, Motor4};//Left motors first, Right motors Second


//Prototypes
int findRotationDirection(float current_direction, float target_direction);
bool autoTurn(int time,float target_direction);
void turnCW();
void turnCCW();
void Stop();
void goForwards(float speed);
void goBackwards(float speed);
void loopHeartbeats();
String outputBno();
String outputBmp();
void outputGPS();
void setLED(int r_val, int b_val, int g_val);
void parseInput(const String input, std::vector<String>& args, char delim); // parse command to args[]
void safety_timeout();




unsigned long lastAccel;
unsigned long lastDuty;
unsigned long lastHB;
unsigned long lastFeedback;
unsigned long lastCtrlCmd;

String feedback;

unsigned long clockTimer = millis();


void setup() {

  //-----------------//
  // Initialize Pins //
  //-----------------//
  
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(SERIAL_BAUD);
    digitalWrite(LED_BUILTIN, HIGH);

    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);

    // Initalization for using CAN with the sparkmax
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();


    Can0.setPins(CAN_TX, CAN_RX);
	
    // You can set custom size for the queues - those are default
    Can0.setRxQueueSize(5);
	  Can0.setTxQueueSize(5);

    // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
    // but you can easily convert it from numerical value using .convertSpeed()
    Can0.setSpeed(Can0.convertSpeed(500));

    // You can also just use .begin()..
    if(Can0.begin()) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }











    pinMode(20, INPUT_PULLUP); //Needed for IMU to work on PCB


    FastLED.addLeds<WS2812B, PIN_LED_STRIP, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(255);
    for(int i = 0; i < NUM_LEDS; ++i)
    {
      leds[i] = CRGB(led_rbg[0], led_rbg[1], led_rbg[2]);
      FastLED.show();
      delay(10);
    }




  //--------------------//
  // Initialize Sensors //
  //--------------------//
    if(!bno.begin()) {
      Serial.println("!BNO failed to start...");
    } else {
      Serial.println("BNO055 Started Successfully");
    }
    if(!bmp.begin_I2C()) {
      Serial.println("bmp not working");
    } else {
      Serial.println("bmp is working");
    }
    if(!myGNSS.begin()) {
      Serial.println("GPS not working");
    } else {
      Serial.println("GPS is working");
    }

  initializeBMP(bmp);



// Setup for GPS
myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  // Create storage for the time pulse parameters
  UBX_CFG_TP5_data_t timePulseParameters;

  // Get the time pulse parameters
  if (myGNSS.getTimePulseParameters(&timePulseParameters) == false)
  {
    Serial.println(F("getTimePulseParameters failed! not Freezing..."));
  }

  // Print the CFG TP5 version
  Serial.print(F("UBX_CFG_TP5 version: "));
  Serial.println(timePulseParameters.version);

  timePulseParameters.tpIdx = 0; // Select the TIMEPULSE pin
  //timePulseParameters.tpIdx = 1; // Or we could select the TIMEPULSE2 pin instead, if the module has one

  // We can configure the time pulse pin to produce a defined frequency or period
  // Here is how to set the frequency:

  // While the module is _locking_ to GNSS time, make it generate 2kHz
  timePulseParameters.freqPeriod = 2000; // Set the frequency/period to 2000Hz
  timePulseParameters.pulseLenRatio = 0x55555555; // Set the pulse ratio to 1/3 * 2^32 to produce 33:67 mark:space

  // When the module is _locked_ to GNSS time, make it generate 1kHz
  timePulseParameters.freqPeriodLock = 1000; // Set the frequency/period to 1000Hz
  timePulseParameters.pulseLenRatioLock = 0x80000000; // Set the pulse ratio to 1/2 * 2^32 to produce 50:50 mark:space

  timePulseParameters.flags.bits.active = 1; // Make sure the active flag is set to enable the time pulse. (Set to 0 to disable.)
  timePulseParameters.flags.bits.lockedOtherSet = 1; // Tell the module to use freqPeriod while locking and freqPeriodLock when locked to GNSS time
  timePulseParameters.flags.bits.isFreq = 1; // Tell the module that we want to set the frequency (not the period)
  timePulseParameters.flags.bits.isLength = 0; // Tell the module that pulseLenRatio is a ratio / duty cycle (* 2^-32) - not a length (in us)
  timePulseParameters.flags.bits.polarity = 1; // Tell the module that we want the rising edge at the top of second. (Set to 0 for falling edge.)

  // Now set the time pulse parameters
  if (myGNSS.setTimePulseParameters(&timePulseParameters) == false)
  {
    Serial.println(F("setTimePulseParameters failed!"));
  }
  else
  {
    Serial.println(F("Success!"));
  }


  //Start heartbeat thread
  //TEMPORARY FIX, until we get a dedicated microcontroller for heartbeat propogation
  threads.addThread(loopHeartbeats);

  
  
}



//------------//
// Begin Loop //
//------------//
//
//
//-------------------------------------------------//
//                                                 //
//    /////////      //            //////////      //
//    //      //     //            //        //    //
//    //      //     //            //        //    //
//    ////////       //            //////////      //
//    //      //     //            //              //
//    //       //    //            //              //
//    /////////      //////////    //              //
//                                                 //
//-------------------------------------------------//


void loop() {
  
  // Accelerate the motors
  // Every 50 milliseconds, update the speed for all motors
  safety_timeout();

  if(millis()-lastAccel >= 50){
    lastAccel = millis();
    for(int i = 0; i < 4; i++)
    {
      motorList[i].UpdateForAcceleration();
    }

    if(motorList[0].getControlMode() == 1)//send the correct duty cycle to the motors
    {
        for(int i = 0; i < 4; i++)
        {
          sendDutyCycle(Can0, motorList[i].getID(), motorList[i].getDuty());
        }
        
    }else{
        //pass for RPM control mode
    }
 
  }



  //----------------------------------//
  // Runs something at a set interval //
  // Useful for testing               //
  //----------------------------------//

  if((millis()-lastFeedback)>=2000){
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    
    //feedback = outputGPS() + "," + outputBno() + "," + outputBmp();
    double gpsData[3];
    float bnoData2[7];

    getPosition(myGNSS, gpsData);
    pullBNOData(bno, bnoData2);

    Serial.print("core,telemetry,");
    Serial.print(gpsData[0],7);
    Serial.print(",");
    Serial.print(gpsData[1],7);
    Serial.print(",");
    Serial.print((int)gpsData[2]);
    Serial.print(",");
    Serial.print(angVelocityData.gyro.x);
    Serial.print(",");
    Serial.print(angVelocityData.gyro.y);
    Serial.print(",");
    Serial.print(angVelocityData.gyro.z);
    Serial.print(",");
    Serial.print(accelerometerData.acceleration.x);
    Serial.print(",");
    Serial.print(accelerometerData.acceleration.y);
    Serial.print(",");
    Serial.print(accelerometerData.acceleration.z);
    Serial.print(",");
    Serial.print(orientationData.orientation.x);
    Serial.print(",");
    Serial.print(bmp.temperature);
    Serial.print(",");
    Serial.print(bmp.pressure);
    Serial.print(",");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println();
    

    //Serial.printf(",%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", , , , , , , , , , , );
    //gps: lat, long, sats bno: gyro_x,y,z, acc_x,y,z, heading bmp: temp, pressure, altitude
    //Serial.println(feedback);
    
    lastFeedback = millis();

  }


  //------------------//
  // Command Receiving //
  //------------------//
  //
  //-------------------------------------------------------//
  //                                                       //
  //      /////////    //\\        ////    //////////      //
  //    //             //  \\    //  //    //        //    //
  //    //             //    \\//    //    //        //    //
  //    //             //            //    //        //    //
  //    //             //            //    //        //    //
  //    //             //            //    //        //    //
  //      /////////    //            //    //////////      //
  //                                                       //
  //-------------------------------------------------------//
  //
  // The giant CMD helps with finding this place
  //
  // Commands will be received as a comma separated value string
  // Ex: "ctrl,1,1,1,1" or "speedMultiplier,0.5" or "sendHealthPacket"
  // The program parses the string so that each piece of data can be used individually
  // For examples of parsing data you can use the link below
  // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // Command is equal to a line in the serial
    command.trim();                                 // I don't know why this is here, but it is important
    // string delimiter = ",";                         // The key that decides where the command should be split
    // size_t pos = 0;                                 // Standard parse variable
    // string token;                                   // The current piece of the string being used.
    // string token2;                                  // A secondary piece of the string saved.
    // string scommand = command.c_str();              // Converts the Arduino String into a C++ string since they are different things
    // pos = scommand.find(delimiter);
    // token = scommand.substr(0, pos);
    String prevCommand;

    //

    std::vector<String> args = {};
    parseInput(command, args, ',');

    //

    if (args[0] == "ctrl") {                          // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        //Serial.println("ctrl cmd received");
        lastCtrlCmd = millis();
        if(command != prevCommand)
        {
          //Serial.println("NEW COMMAND RECEIVED");

          prevCommand = command;

          motorList[0].setDuty(args[1].toFloat());
          motorList[1].setDuty(args[1].toFloat());

          motorList[2].setDuty(args[2].toFloat());
          motorList[3].setDuty(args[2].toFloat());

          
        }else{
          //pass if command if control command is same as previous
        }
    }else if (args[0] == "speedMultiplier") {         // Is looking for a command that looks like "ctrl,x" where 0<x<1
      // scommand.erase(0, pos + delimiter.length());
      // token = scommand.substr(0, pos);
      // pos = scommand.find(delimiter);
      // //Motor1.setMotorMultiplier(stof(token));
    }else if (args[0] == "brake") {
        if(args[1] == "on") {
            setParameter(Can0, 1, 6, 1);
            setParameter(Can0, 2, 6, 1);
            setParameter(Can0, 3, 6, 1);
            setParameter(Can0, 4, 6, 1);
        } else if(args[1] == "off") {
            setParameter(Can0, 1, 6, 0);
            setParameter(Can0, 2, 6, 0);
            setParameter(Can0, 3, 6, 0);
            setParameter(Can0, 4, 6, 0);
        }
    }else if (args[0] == "auto") {  // Commands for autonomy
        lastCtrlCmd = millis();
        if(command != prevCommand)
        {
          if(args[1] == "turningTo"){ // auto,turningTo
            bool success = false;

            success = autoTurn(args[2].toFloat(),args[3].toFloat());
            if(success)
            {
              Serial.println("turningTo,success");
            }else{
              Serial.println("turningTo,fail");
            }
          }else if(args[1] == "forwards"){  // auto,forwards
           

            goForwards(args[2].toFloat());
          }else if(args[1] == "backwards"){ // auto,backwards
            // scommand.erase(0, pos + delimiter.length());
            // pos = scommand.find(delimiter); 
            // token = scommand.substr(0, pos);

            goBackwards(args[2].toFloat());
          }else if(args[1] == "stop"){  // auto,stop
            Stop();
          }
        }else{
          //pass if command if control command is same as previous
        }
    }else if (args[0] == "data") {  // Send data out
        
          // scommand.erase(0, pos + delimiter.length());
          // prevCommand = command;
          // pos = scommand.find(delimiter);
          // token = scommand.substr(0, pos);


          if(args[1] == "sendGPS"){ // data,sendGPS
            outputGPS();
            //Serial.println(outputGPS());
            //To ensure precision output this will print null
            //but the outputGPS() function will Serial.print values directly in-function

          }else if(args[1] == "sendIMU"){ // data,sendIMU

            Serial.println(outputBno());

          }else if(args[1] == "sendBMP"){ // data,sendBMP

            Serial.println(outputBmp());

          }else if(args[1] == "everything"){  // data,everything

            //Serial.println(outputGPS());
            Serial.println(outputBno());
            Serial.println(outputBmp());

          }else if(args[1] == "getOrientation"){  // data,getOrientation
            Serial.printf("orientation,%f\n", getBNOOrient(bno));
          }
        
    } else if (args[0] == "led_set") {    //set LED strip color format: led_set,r,b,g
      
      for(int i = 0; i < 3; i++)
      {
        led_rbg[i] = args[i+1].toInt();
      }
      
      setLED(led_rbg[0], led_rbg[1], led_rbg[2]);

    } else if (args[0] == "ping") {
      Serial.println("pong");
    } else if (args[0] == "time") {
      Serial.println(millis());
    }


  }

}




//-------------------------------------------------------//
//                                                       //
//    ///////////    //\\          //      //////////    //
//    //             //  \\        //    //              //
//    //             //    \\      //    //              //
//    //////         //      \\    //    //              //
//    //             //        \\  //    //              //
//    //             //          \\//    //              //
//    //             //           \//      //////////    //
//                                                       //
//-------------------------------------------------------//

void safety_timeout(){
  if(millis() - lastCtrlCmd > 2000)//if no control commands are received for 2 seconds
  {
    lastCtrlCmd = millis();//just update the var so this only runs every 2 seconds.
    Stop();
    Serial.println("No Control / Safety Timeout");
  }
}

// Prints the output of the BNO in one line
String outputBno()
{
  float bnoData2[7];
  pullBNOData(bno,bnoData2);
  String output;
  //sprintf(output,"%f,%f,%f,%f,%f,%f,%f",bnoData2[0],bnoData2[1],bnoData2[2],bnoData2[3],bnoData2[4],bnoData2[5],bnoData2[6]);
  
  return output;
}

// Prints the output of the GPS in one line
void outputGPS()
{
  String output = "null";
  double gpsData[3];
  getPosition(myGNSS, gpsData);
  Serial.print("gps,");
  Serial.print(gpsData[0],7);
  Serial.print(",");
  Serial.print(gpsData[1],7);
  Serial.println();
  
  //return output;
}

// Prints the output of the BMP in one line
String outputBmp()
{
  float bmpData[3];
  pullBMPData(bmp, bmpData);
  String output;
  //sprintf(output, "%f,%f,%f",bmpData[0],bmpData[1],bmpData[2]);

  return output;
}

// Bypasses the acceleration to make the rover turn clockwise
// Should only be used for autonomy
void turnCW(){
  sendDutyCycle(Can0, 2, 0.6);
  sendDutyCycle(Can0, 4, 0.6);
  sendDutyCycle(Can0, 1, 0.6);
  sendDutyCycle(Can0, 3, 0.6);
}

// Bypasses the acceleration to make the rover turn counterclockwise
// Should only be used for autonomy
void turnCCW(){
  sendDutyCycle(Can0, 2, -0.6);
  sendDutyCycle(Can0, 4, -0.6);
  sendDutyCycle(Can0, 1, -0.6);
  sendDutyCycle(Can0, 3, -0.6);
}

// Bypasses the acceleration to make the rover stop
// Should only be used for autonomy, but it could probably be used elsewhere
void Stop(){
  sendDutyCycle(Can0, motorList[0].getID(), 0);
  sendDutyCycle(Can0, motorList[1].getID(), 0);
  sendDutyCycle(Can0, motorList[2].getID(), 0);
  sendDutyCycle(Can0, motorList[3].getID(), 0);

  motorList[0].setDuty(0.0);
  motorList[1].setDuty(0.0);
  motorList[2].setDuty(0.0);
  motorList[3].setDuty(0.0);
}

// Tells the rover to go forwards
// Does not bypass acceleration
// Autonomy
void goForwards(float speed){
  motorList[0].setDuty(speed);
  motorList[1].setDuty(speed);
  motorList[2].setDuty(speed);
  motorList[3].setDuty(speed);
}

// Tells the rover to go backwards
// Does not bypass acceleration
// Autonomy
void goBackwards(float speed){
  float temp = (-1) * speed;
  motorList[0].setDuty(temp);
  motorList[1].setDuty(temp);
  motorList[2].setDuty(temp);
  motorList[3].setDuty(temp);
}

// Specify where the rover should turn
// and how long it should take before
// the rover decides that it has failed
// to turn
bool autoTurn(int time, float target_direction){
  int startTime = millis(); 
  unsigned long expectedTime;
  expectedTime = time;
  
  float current_direction = getBNOOrient(bno);
  bool turningRight = findRotationDirection(current_direction, target_direction);

  Serial.printf("StartTime: %d, Expected: %d",(int)startTime, (int)expectedTime);


  while(millis() - startTime < expectedTime){
    current_direction = getBNOOrient(bno);
    if(!((current_direction < target_direction + 2) && (current_direction > target_direction - 2))){
      
      turningRight = findRotationDirection(current_direction, target_direction);

      Serial.print("Turning to: ");
      Serial.println(target_direction);
      Serial.print("Currently at: ");
      Serial.println(getBNOOrient(bno));

      if(turningRight){
        turnCW();
      }else{
        turnCCW();
      }
    }else{
      Stop();
      return true;
    }
  }
  Stop();
  return false;
}


// Finds out which direction the rover should turn
int findRotationDirection(float current_direction, float target_direction){
  int cw_dist = target_direction - current_direction + 360;
  cw_dist %= 360;
  int ccw_dist = current_direction - target_direction + 360; 
  ccw_dist %= 360;

  if(cw_dist <= ccw_dist)
  {
    return 1;//Rotate CW if distance is 180 or less
  }else{
    return 0;//Rotate CCW if distance is greater than 180
  }
} 


// Magic that makes the SparkMax work with CAN
void loopHeartbeats(){
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

    while(1){
      sendHeartbeat(Can0, 1);
      threads.delay(3);
      sendHeartbeat(Can0, 2);
      threads.delay(3);
      sendHeartbeat(Can0, 3);
      threads.delay(3);
      sendHeartbeat(Can0, 4);
      threads.delay(3);
      threads.yield();
    }
}


void setLED(int r_val, int b_val, int g_val)
{
    for(int i = 0; i < NUM_LEDS; ++i)
    {
      leds[i] = CRGB(r_val, b_val, g_val);
      FastLED.show();
      delay(10);
    }
}


// Parse `input` into `args` separated by `delim`
// Ex: "ctrl,led,on" => {ctrl,led,on}
// Equivalent to Python's `.split()`
void parseInput(const String input, std::vector<String>& args, char delim) {
    //Modified from https://forum.arduino.cc/t/how-to-split-a-string-with-space-and-store-the-items-in-array/888813/9

    // Index of previously found delim
    int lastIndex = -1;
    // Index of currently found delim
    int index = -1;
    // because lastIndex=index, lastIndex starts at -1, so with lastIndex+1, first search begins at 0

    // if empty input for some reason, don't do anything
    if(input.length() == 0)
        return;

    unsigned count = 0;
    while (count++, count < 200 /*arbitrary limit on number of delims because while(true) is scary*/) {
        lastIndex = index;
        // using lastIndex+1 instead of input = input.substring to reduce memory impact
        index = input.indexOf(delim, lastIndex+1);
        if (index == -1) { // No instance of delim found in input
            // If no delims are found at all, then lastIndex+1 == 0, so whole string is passed.
            // Otherwise, only the last part of input is passed because of lastIndex+1.
            args.push_back(input.substring(lastIndex+1));
            // Exit the loop when there are no more delims
            break;
        } else { // delim found
            // If this is the first delim, lastIndex+1 == 0, so starts from beginning
            // Otherwise, starts from last found delim with lastIndex+1
            args.push_back(input.substring(lastIndex+1, index));
        }
    }

    // output is via vector<String>& args
}