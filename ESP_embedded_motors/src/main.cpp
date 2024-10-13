// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <vector>
// Our own resources
#include "project/CORE.h"
#include "AstraMisc.h"
#include "AstraMotors.h"
#include "AstraCAN.h"

using namespace std;

//Setting up for CAN0 line
AstraFCAN Can0;

//AstraMotors(AstraFCAN setCanObject, int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Motor1(Can0, 2, 1, false, 50, 1.00F);  // Front Left
AstraMotors Motor2(Can0, 4, 1, false, 50, 1.00F);  // Back Left
AstraMotors Motor3(Can0, 1, 1, true, 50, 1.00F);  // Front Right
AstraMotors Motor4(Can0, 3, 1, true, 50, 1.00F);  // Back Right

AstraMotors motorList[4] = {Motor1, Motor2, Motor3, Motor4};//Left motors first, Right motors Second


//Prototypes
void turnCW();
void turnCCW();
void Stop();
void goForwards(float speed);
void goBackwards(float speed);
void loopHeartbeats();
void parseInput(const String input, std::vector<String>& args, char delim); // parse command to args[]
void safety_timeout();


unsigned long lastAccel;
unsigned long lastDuty;
unsigned long lastHB;
unsigned long lastFeedback;
unsigned long lastCtrlCmd;

String feedback;

unsigned long clockTimer = millis();
unsigned long heartBeatNum = 1;


void setup() 
{

  //-----------------//
  // Initialize Pins //
  //-----------------//
  
    pinMode(LED_BUILTIN, OUTPUT);
    Serial1.begin(SERIAL_BAUD);
    Serial.begin(SERIAL_BAUD);
    digitalWrite(LED_BUILTIN, HIGH);

    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);

    // Initalization for using CAN with the sparkmax
    /* Old code for reference
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    */

    Can0.setPins(CAN_REV_TX, CAN_REV_RX);
	
    // You can set custom size for the queues - those are default
    Can0.setRxQueueSize(5);
	  Can0.setTxQueueSize(5);

    // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
    // but you can easily convert it from numerical value using .convertSpeed()
    Can0.setSpeed(Can0.convertSpeed(500));

    // You can also just use .begin()..
    if(Can0.begin()) 
    {
        Serial1.println("CAN bus started!");
    } 
    else 
    {
        Serial1.println("CAN bus failed!");
    }

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


void loop() 
{
  
  // Accelerate the motors
  // Every 50 milliseconds, update the speed for all motors
  safety_timeout();

  if(millis()-lastAccel >= 50)
  {

    lastAccel = millis();
    for(int i = 0; i < 4; i++)
    {
      motorList[i].UpdateForAcceleration();
    }

    if(motorList[0].getControlMode() == 1) //send the correct duty cycle to the motors
    {

      for(int i = 0; i < 4; i++)
      {
        sendDutyCycle(Can0, motorList[i].getID(), motorList[i].getDuty());
      }
        
    }
    else
    {
        //pass for RPM control mode
    }
 
  }



  //----------------------------------//
  // send heartbeat                   //
  //----------------------------------//

  if((millis()-lastFeedback)>=3)
  {
    sendHeartbeat(Can0, heartBeatNum % 4);
    lastFeedback = millis();
    heartBeatNum++;
  }

  if (heartBeatNum > 99999)
  {
    heartBeatNum = 1;
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

  if (Serial1.available()) 
  {

    String command = Serial1.readStringUntil('\n');  // Command is equal to a line in the Serial1
    command.trim();
    Serial.println(command);
    String prevCommand;

    std::vector<String> args = {};
    parseInput(command, args, ',');

    //

    if (args[0] == "ctrl") // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
    {   

      //Serial1.println("ctrl cmd received");
      lastCtrlCmd = millis();
      if(command != prevCommand)
      {

        //Serial1.println("NEW COMMAND RECEIVED");

        prevCommand = command;

        motorList[0].setDuty(args[1].toFloat());
        motorList[1].setDuty(args[1].toFloat());

        motorList[2].setDuty(args[2].toFloat());
        motorList[3].setDuty(args[2].toFloat());

        
      }
      else
      {
        //pass if command if control command is same as previous
      }

    }
    else if (args[0] == "brake") 
    {

      if(args[1] == "on") 
      {
          setParameter(Can0, 1, 6, 1);
          setParameter(Can0, 2, 6, 1);
          setParameter(Can0, 3, 6, 1);
          setParameter(Can0, 4, 6, 1);
      }

      else if(args[1] == "off")
      {
          setParameter(Can0, 1, 6, 0);
          setParameter(Can0, 2, 6, 0);
          setParameter(Can0, 3, 6, 0);
          setParameter(Can0, 4, 6, 0);
      }

    }
    else if (args[0] == "auto") // Commands for autonomy
    { 

      lastCtrlCmd = millis();
      if(command != prevCommand)
      {

        if(args[1] == "forwards") // auto,forwards
        {  
          goForwards(args[2].toFloat());
        }

        else if(args[1] == "backwards") // auto,backwards
        { 
          goBackwards(args[2].toFloat());
        }

        else if(args[1] == "TurnCW") // auto,backwards
        { 
          turnCW();
        }

        else if(args[1] == "TurnCCW") // auto,backwards
        { 
          turnCCW();
        }

        else if(args[1] == "stop") // auto,stop
        {  
          Stop();
        }

      }
      else
      {
        //pass if command if control command is same as previous
      }

    }

    else if (args[0] == "ping") 
    {
      Serial1.println("pong");
    } 

    else if (args[0] == "time") 
    {
      Serial1.println(millis());
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

void safety_timeout()
{

  if(millis() - lastCtrlCmd > 2000)//if no control commands are received for 2 seconds
  {

    lastCtrlCmd = millis();//just update the var so this only runs every 2 seconds.
    Stop();
    Serial1.println("No Control / Safety Timeout");
  }

}

// Bypasses the acceleration to make the rover turn clockwise
// Should only be used for autonomy
void turnCW()
{
  sendDutyCycle(Can0, 2, 0.6);
  sendDutyCycle(Can0, 4, 0.6);
  sendDutyCycle(Can0, 1, 0.6);
  sendDutyCycle(Can0, 3, 0.6);
}

// Bypasses the acceleration to make the rover turn counterclockwise
// Should only be used for autonomy
void turnCCW()
{
  sendDutyCycle(Can0, 2, -0.6);
  sendDutyCycle(Can0, 4, -0.6);
  sendDutyCycle(Can0, 1, -0.6);
  sendDutyCycle(Can0, 3, -0.6);
}

// Bypasses the acceleration to make the rover stop
// Should only be used for autonomy, but it could probably be used elsewhere
void Stop()
{
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
void goForwards(float speed)
{
  motorList[0].setDuty(speed);
  motorList[1].setDuty(speed);
  motorList[2].setDuty(speed);
  motorList[3].setDuty(speed);
}

// Tells the rover to go backwards
// Does not bypass acceleration
// Autonomy
void goBackwards(float speed)
{
  float temp = (-1) * speed;
  motorList[0].setDuty(temp);
  motorList[1].setDuty(temp);
  motorList[2].setDuty(temp);
  motorList[3].setDuty(temp);
}

// Parse `input` into `args` separated by `delim`
// Ex: "ctrl,led,on" => {ctrl,led,on}
// Equivalent to Python's `.split()`
void parseInput(const String input, std::vector<String>& args, char delim) 
  {
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
        if (index == -1) // No instance of delim found in input
        {
            // If no delims are found at all, then lastIndex+1 == 0, so whole string is passed.
            // Otherwise, only the last part of input is passed because of lastIndex+1.
            args.push_back(input.substring(lastIndex+1));
            // Exit the loop when there are no more delims
            break;
        } 
        else // delim found
        {
            // If this is the first delim, lastIndex+1 == 0, so starts from beginning
            // Otherwise, starts from last found delim with lastIndex+1
            args.push_back(input.substring(lastIndex+1, index));
        }

    }

    // output is via vector<String>& args
}