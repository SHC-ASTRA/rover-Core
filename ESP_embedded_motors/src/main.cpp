// Includes
#include <Arduino.h>
#include <cmath>
// Our own resources
#if defined(TESTBED)
#   include "project/TESTBED.h"
#else
#   include "project/CORE.h"
#endif
#include "AstraMisc.h"
#include "AstraMotors.h"
#ifdef OLD_ASTRACAN_ENABLE
#   include "AstraCAN.h"
#else
#   include "AstraREVCAN.h"
#endif

using namespace std;

// Setting up for CAN0 line
AstraCAN Can0;

// AstraMotors(AstraCAN* setCanObject, int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Motor1(&Can0, MOTOR_ID_FL, CTRL_DUTYCYCLE, false, 1000, 1.0);  // Front Left
AstraMotors Motor2(&Can0, MOTOR_ID_BL, CTRL_DUTYCYCLE, false, 1000, 1.0);  // Back Left
AstraMotors Motor3(&Can0, MOTOR_ID_FR, CTRL_DUTYCYCLE, true, 1000, 1.0);   // Front Right
AstraMotors Motor4(&Can0, MOTOR_ID_BR, CTRL_DUTYCYCLE, true, 1000, 1.0);   // Back Right

AstraMotors* motorList[4] = {&Motor1, &Motor2, &Motor3, &Motor4};  //Left motors first, right motors second

// Use Serial when using directly with Laptop, use Serial1 when using over UART with main ESP32
// Purposefully override TESTBED.h for motor mcu for testing
#ifdef DEBUG
#   define COMMS_UART Serial
#endif

// #define DEBUG_STATUS


//------------//
// Prototypes //
//------------//

void turnCW();
void turnCCW();
void Stop();
void Brake(bool enable);
void goForwards(float speed);
void goBackwards(float speed);
void loopHeartbeats();
void safety_timeout();


//--------//
// Timing //
//--------//

unsigned long lastMotorStatus = 0;
unsigned long lastAccel;
unsigned long lastDuty;
unsigned long lastHB;
unsigned long lastFeedback = 0;
unsigned long lastCtrlCmd;
unsigned long clockTimer = millis();
unsigned long heartBeatNum = 1;
uint32_t lastBlink = 0;
bool ledState = false;
String feedback;


//-------//
// Setup //
//-------//

void setup() 
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial1.begin(SERIAL_BAUD);
    Serial.begin(SERIAL_BAUD);
    digitalWrite(LED_BUILTIN, HIGH);

    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);

    // Setup CAN
    if (Can0.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX)) 
    {
        COMMS_UART.println("CAN bus started!");
    } 
    else 
    {
        COMMS_UART.println("CAN bus failed!");
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
    // Blink the LED
    if (millis() - lastBlink >= 1000) 
    {
        lastBlink = millis();
        ledState = !ledState;
        if (ledState)
            digitalWrite(LED_BUILTIN, HIGH);
        else
            digitalWrite(LED_BUILTIN, LOW);
    }

    // Every 50 milliseconds, update the speed for all motors
    // Accelerate the motors
    if (millis() - lastAccel >= 50)
    {
        lastAccel = millis();
        for (int i = 0; i < 4; i++)
        {
            motorList[i]->accelerate();
        }
    }

    //----------------------------------//
    // send heartbeat                   //
    //----------------------------------//

    if ((millis()-lastFeedback)>=3)
    {
        sendHeartbeat(Can0, heartBeatNum);
        lastFeedback = millis();
        heartBeatNum++;
        if (heartBeatNum > 4)
        {
            heartBeatNum = 1;
        }
    }

    safety_timeout();


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
    // Ex: "ctrl,1,1" or "speedMultiplier,0.5" or "sendHealthPacket"
    // commands will be deliminated by "," and put into vector<String> args

    if (COMMS_UART.available()) 
    {
        String command = COMMS_UART.readStringUntil('\n');  // Command is equal to a line in the Serial1
        command.trim();
#ifdef DEBUG
        Serial.println(command);
#endif
        String prevCommand;

        std::vector<String> args = {}; 
        parseInput(command, args);


        if (args[0] == "ping") 
        {
#ifndef DEBUG
            COMMS_UART.println("pong");
#else
            Serial.println("pong");
            Serial1.println("pong");
#endif
        } 
        else if (args[0] == "time") 
        {
            COMMS_UART.println(millis());
        }
#if defined(DEBUG) && !defined(OLD_ASTRACAN_ENABLE)
        else if (args[0] == "id") {
            CAN_identifySparkMax(2, Can0);
        }
        else if (args[0] == "speed" && checkArgs(args, 1)) {
            CAN_sendSpeed(MOTOR_ID_BL, args[1].toFloat(), Can0);
        }
        else if (args[0] == "newduty") {
            Serial.print("Setting duty cycle ");
            Serial.println(args[1].toFloat());
            CAN_sendDutyCycle(1, args[1].toFloat(), Can0);
            CAN_sendDutyCycle(2, args[1].toFloat(), Can0);
            CAN_sendDutyCycle(3, args[1].toFloat(), Can0);
            CAN_sendDutyCycle(4, args[1].toFloat(), Can0);
        }
        else if (args[0] == "stop") {
            Serial.println("Stopping all motors");
            for (int i = 0; i < 4; i++)
            {
                CAN_sendDutyCycle(i, 0, Can0);
                Stop();
            }
        }
#endif

        else if (args[0] == "ctrl") // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        {   
            lastCtrlCmd = millis();
            if (command != prevCommand)
            {
                prevCommand = command;

                if (checkArgs(args, 2)) 
                {
                    motorList[0]->setDuty(args[1].toFloat());
                    motorList[1]->setDuty(args[1].toFloat());

                    motorList[2]->setDuty(args[2].toFloat());
                    motorList[3]->setDuty(args[2].toFloat());
                } else {
                    for (int i = 0; i < 4; i++)
                        motorList[i]->setDuty(args[1].toFloat());
                }
            }
        }

        else if (args[0] == "brake") 
        {
            if (args[1] == "on") 
            {
                Brake(true);
#ifdef DEBUG
                Serial.println("Setting brakemode on.");
#endif
            }

            else if (args[1] == "off")
            {
                Brake(false);
#ifdef DEBUG
                Serial.println("Setting brakemode off.");
#endif
            }
        }

        else if (args[0] == "auto") // Commands for autonomy
        { 
            lastCtrlCmd = millis();
            if (command != prevCommand)
            {
                if (args[1] == "forwards") // auto,forwards
                {  
                    goForwards(args[2].toFloat());
                }

                else if (args[1] == "backwards") // auto,backwards
                { 
                    goBackwards(args[2].toFloat());
                }

                else if (args[1] == "TurnCW") // auto,backwards
                { 
                    turnCW();
                }

                else if (args[1] == "TurnCCW") // auto,backwards
                { 
                    turnCCW();
                }

                else if (args[1] == "stop") // auto,stop
                {  
                    Stop();
                }
            }
        }

    }

    // Check for incoming CAN messages
    static CanFrame rxFrame;
    if (ESP32Can.readFrame(rxFrame, 1)) {
        // Decode the ID

        uint32_t msgId = rxFrame.identifier;
        // Pull out device ID and API ID
        uint8_t deviceId = msgId & 0x3F;
        uint32_t apiId = (msgId >> 6) & 0x3FF;

        if (apiId == 0x61) {
            for (int i = 0; i < 4; i++) {
                if (deviceId == motorList[i]->getID()) {
                    motorList[i]->parseStatus1(rxFrame.data);
                    break;
                }
            }
        } else if (apiId == 0x62) {
            for (int i = 0; i < 4; i++) {
                if (deviceId == motorList[i]->getID()) {
                    motorList[i]->parseStatus2(rxFrame.data);
                    break;
                }
            }
        }

#if defined(DEBUG_STATUS)
        // Log message if it seems interesting
        if (apiId == 0x99 || apiId == 0x60 || apiId == 0x61 || apiId == 0x62 || apiId == 0x63 || apiId == 0x64) {
            Serial.print(apiId, HEX);
            Serial.print(" from ");
            Serial.print(deviceId);
            // Message data:
            if (rxFrame.data_length_code == 0)
                Serial.print(" - no data.");
            else {
                Serial.print(" - data: (");
                Serial.print(rxFrame.data_length_code);
                Serial.print(" B) ");
                for (int i = 0; i < rxFrame.data_length_code; i++) {
                    Serial.print(rxFrame.data[i], HEX);
                    Serial.print(" ");
                }
            }
            Serial.println();
        }
#endif
    }

#if defined(DEBUG)
    if (millis() - lastMotorStatus > 2000) {
        lastMotorStatus = millis();

        // Status 1
        Serial.print(millis() - Motor1.status1.timestamp);
        Serial.print(" ms ago: ");
        Serial.print(Motor1.status1.motorTemperature);
        Serial.print(" *C; ");
        Serial.print(Motor1.status1.busVoltage);
        Serial.print(" V; ");
        Serial.print(Motor1.status1.outputCurrent);
        Serial.println(" A");
        
        // Status 2
        Serial.print(millis() - Motor2.status1.timestamp);
        Serial.print(" ms ago: ");
        Serial.print(Motor2.status2.sensorPosition);
        Serial.println(" position");
    }
#endif
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
    if (millis() - lastCtrlCmd > 2000)  // if no control commands are received for 2 seconds
    {
        lastCtrlCmd = millis();
        COMMS_UART.println("No Control, Safety Timeout");
        Stop();
    }
}

// Bypasses the acceleration to make the rover turn clockwise
// Should only be used for autonomy
void turnCW()
{
    for (int i = 0; i < 4; i++)
        motorList[i]->sendDuty(0.6);
}

// Bypasses the acceleration to make the rover turn counterclockwise
// Should only be used for autonomy
void turnCCW()
{
    for (int i = 0; i < 4; i++)
        motorList[i]->sendDuty(-0.6);
}

// Bypasses the acceleration to make the rover stop
// Should only be used for autonomy, but it could probably be used elsewhere
void Stop()
{
    for (int i = 0; i < 4; i++)
        motorList[i]->sendDuty(0.0);
}

// Enables or disables brake mode for all motors
void Brake(bool enable) {
    for (int i = 0; i < 4; i++)
        motorList[i]->setBrake(enable);
}

// Tells the rover to go forwards
// Does not bypass acceleration
// Autonomy
void goForwards(float speed)
{
    for (int i = 0; i < 4; i++ )
        motorList[i]->setDuty(speed);
}

// Tells the rover to go backwards
// Does not bypass acceleration
// Autonomy
void goBackwards(float speed)
{
    float temp = (-1) * speed;
    for (int i = 0; i < 4; i++ )
        motorList[i]->setDuty(temp);
}
