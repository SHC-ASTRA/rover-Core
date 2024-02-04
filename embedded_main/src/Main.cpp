// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <utility/imumaths.h>
// Our own resources
//#include "AstraMotors.h"
//#include "AstraCAN.h"
//#include "AstraSensors.h"
#include "TeensyThreads.h"
#include "AstraSubroutines.h"





using namespace std;

#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)

//Sensor declarations
Adafruit_BNO055 bno = Adafruit_BNO055();

//Setting up for CAN0 line
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

//AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Motor1(2, 1, false, 50, 0.50F);//FL
AstraMotors Motor2(4, 1, false, 50, 0.50F);//BL
AstraMotors Motor3(1, 1, true, 50, 0.50F);//FR
AstraMotors Motor4(3, 1, true, 50, 0.50F);//BR

AstraMotors motorList[4] = {Motor1, Motor2, Motor3, Motor4};//Left motors first, Right motors Second



void turnCW(){
  motorList[0].setDuty(0.2);
  motorList[1].setDuty(0.2);
  motorList[2].setDuty(-0.2);
  motorList[3].setDuty(-0.2);
}


void turnCCW(){
  motorList[0].setDuty(-0.2);
  motorList[1].setDuty(-0.2);
  motorList[2].setDuty(0.2);
  motorList[3].setDuty(0.2);
}


bool rotateTo(float direction){
  bool success = 0;
  bool turningRight;
  int startTime = millis(); 
  int expectedTime;
  float difference;
  //difference = abs(direction - /*get rotation*/);
  expectedTime = difference * 500;
  if(0/*sin(direction - get rotation)>0*/){
    turningRight = 1;
  }else{
    turningRight = 0;
  }
  while(millis() - startTime < expectedTime){
    if(0/*!((get rotation < direction + 2) && (get rotation > direction - 2))*/){
     if(turningRight){
       turnCW();
     }else{
       turnCCW();
     }
    }else{
      success = 1;
    }
  }
  return success;
}


bool rotate(float amount){
  return rotateTo(/*get rotation*/ + amount);
} 




unsigned long lastAccel;
unsigned long lastDuty;
unsigned long lastHB;

unsigned long lastIdentify;

void loopHeartbeats(){
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

    while(1){
      sendHeartbeat(Can0, 1);
      threads.delay(30);
      sendHeartbeat(Can0, 2);
      threads.delay(30);
      sendHeartbeat(Can0, 3);
      threads.delay(30);
      sendHeartbeat(Can0, 4);
      threads.delay(30);
      threads.yield();
    }

}


void setup() {

  //-----------------//
  // Initialize Pins //
  //-----------------//
  
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    digitalWrite(LED_PIN, HIGH);

    delay(5000);
    digitalWrite(LED_PIN, LOW);

    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

  //--------------------//
  // Initialize Sensors //
  //--------------------//
    if(!bno.begin())
    {
      Serial.println("!BNO failed to start...");
    }else{
      Serial.println("BNO055 Started Successfully");
    }

  //Start heartbeat thread
  //TEMPORARY FIX, until we get a dedicated microcontroller for heartbeat propogation
  threads.addThread(loopHeartbeats);

}



//------------//
// Begin Loop //
//------------//

void loop() {
  // Required To make the bmp not do stupid shit,
  // I am keeping it in this version so that I don't forget about it
  //Serial.println(bmp.temperature);
  

  // Accelerate the motors
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
          //sendHeartbeat(Can0, i+1);
          sendDutyCycle(Can0, motorList[i].getID(), motorList[i].getDuty());
          //Serial.println("Sending Duty Cycle");
        }
        /* Debug print duty cycles every 1 second
        if(millis()-lastDuty >= 1000)
        {
          lastDuty = millis();
          
          Serial.print("Duty1: ");
          Serial.print(motorList[0].getDuty());
          Serial.print("\tDuty2: ");
          Serial.println(motorList[1].getDuty());

          Serial.print("Duty3: ");
          Serial.print(motorList[2].getDuty());
          Serial.print("\tDuty4: ");
          Serial.println(motorList[3].getDuty());
        }
        */
    }else{
        //pass for RPM control mode
    }
 

  }


  //------------------//
  // Command Receiving //
  //------------------//
  //
  // Commands will be received as a comma separated value string
  // Ex: "ctrl,1,1,1,1" or "speedMultiplier,0.5" or "sendHealthPacket"
  // The program parses the string so that each piece of data can be used individually
  // For examples of parsing data you can use the link below
  // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // Command is equal to a line in the serial
    command.trim();                                 // I don't know why this is here, but it is important
    string delimiter = ",";                         // The key that decides where the command should be split
    size_t pos = 0;                                 // Standard parse variable
    string token;                                   // The current piece of the string being used.
    string scommand = command.c_str();         // Converts the Arduino String into a C++ string since they are different things
    pos = scommand.find(delimiter);
    token = scommand.substr(0, pos);
    String prevCommand;

    if (token == "ctrl") {                          // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        if(command != prevCommand)
        {
          scommand.erase(0, pos + delimiter.length());

          prevCommand = command;

          for(int i = 0; i < 3; i+= 2){
            token = scommand.substr(0, pos);
            pos = scommand.find(delimiter);
            motorList[i].setDuty(stof(token));
            motorList[i+1].setDuty(stof(token));
            /*
            Serial.print("SetDUTY [");
            Serial.print(i);
            Serial.print("]: ");
            Serial.println(stof(token)); 
            */
            scommand.erase(0, pos + delimiter.length());
          }
        }else{
          //pass if command if control command is same as previous
        }
    }else if (token == "speedMultiplier") {         // Is looking for a command that looks like "ctrl,x" where 0<x<1
      scommand.erase(0, pos + delimiter.length());
      token = scommand.substr(0, pos);
      pos = scommand.find(delimiter);
      //Motor1.setMotorMultiplier(stof(token));
    } else if (token == "led_on") {
      digitalWrite(LED_PIN, HIGH);
    } else if (token == "led_off") {
      digitalWrite(LED_PIN, LOW);
    } else if (token == "ping") {
      Serial.println("pong");
    } else if (token == "time") {
      Serial.println(millis());
    }else if (token == "auto") {                          
        if(command != prevCommand)
        {
          scommand.erase(0, pos + delimiter.length());
          prevCommand = command;
          pos = scommand.find(delimiter);
          token = scommand.substr(0, pos);


          if(token == "turningTo"){
            scommand.erase(0, pos + delimiter.length());
            pos = scommand.find(delimiter);
            token = scommand.substr(0, pos);
            rotateTo(stoi(token));
          }
        }else{
          //pass if command if control command is same as previous
        }
    }


  }

}
