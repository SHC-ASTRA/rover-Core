/**
 * @file Main.cpp
 * @author Charles Marmann (cmm0077@uah.edu)
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Controls REV motors on ASTRA's Core submodule
 *
 */

//------------//
//  Includes  //
//------------//

#include <Arduino.h>

#include <cmath>

#include "AstraMisc.h"
#include "AstraMotors.h"  // includes AstraREVCAN.h

// Project header
#include "CoreMotorMCU.h"


//------------//
//  Settings  //
//------------//

// #define DEBUG_STATUS

#ifdef DEBUG
#    define COMMS_UART Serial  // Talk directly over USB
#else
#    define COMMS_UART Serial1  // UART between Main-Motor
#endif

#ifdef TESTBED
#   define WHEEL_GEARBOX 64
#else
#   define WHEEL_GEARBOX 100
#endif


//---------------------//
//  Component classes  //
//---------------------//

// AstraMotors(int setMotorID, int setCtrlMode, bool setInverted, int setGearBox)
AstraMotors Motor1(MOTOR_ID_FL, sparkMax_ctrlType::kDutyCycle, false, WHEEL_GEARBOX);  // Front Left
AstraMotors Motor2(MOTOR_ID_BL, sparkMax_ctrlType::kDutyCycle, false, WHEEL_GEARBOX);  // Back Left
AstraMotors Motor3(MOTOR_ID_FR, sparkMax_ctrlType::kDutyCycle, true, WHEEL_GEARBOX);   // Front Right
AstraMotors Motor4(MOTOR_ID_BR, sparkMax_ctrlType::kDutyCycle, true, WHEEL_GEARBOX);   // Back Right

AstraMotors* motorList[4] = {&Motor1, &Motor2, &Motor3, &Motor4};  // Left motors first, right motors second


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;
unsigned long lastAccel = 0;
unsigned long lastHB = 0;
int heartBeatNum = 1;
unsigned long lastCtrlCmd = 0;
unsigned long lastMotorStatus = 0;


//--------------//
//  Prototypes  //
//--------------//

void turnCW();
void turnCCW();
void Stop();
void Brake(bool enable);
void goForwards(float speed);
void goBackwards(float speed);
void loopHeartbeats();
void driveMeters(float meters);
float getDriveSpeed();


//------------------------------------------------------------------------------------------------//
//  Setup
//------------------------------------------------------------------------------------------------//
//
//
//------------------------------------------------//
//                                                //
//      ////////    //////////    //////////      //
//    //                //        //        //    //
//    //                //        //        //    //
//      //////          //        //////////      //
//            //        //        //              //
//            //        //        //              //
//    ////////          //        //              //
//                                                //
//------------------------------------------------//
void setup() {
    //--------//
    //  Pins  //
    //--------//

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);
    Serial1.begin(COMMS_UART_BAUD);

    // Setup CAN
    if (ESP32Can.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX))
        COMMS_UART.println("CAN bus started!");
    else
        COMMS_UART.println("CAN bus failed!");

    //-----------//
    //  Sensors  //
    //-----------//

    //--------------------//
    //  Misc. Components  //
    //--------------------//

}


//------------------------------------------------------------------------------------------------//
//  Loop
//------------------------------------------------------------------------------------------------//
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
    //----------//
    //  Timers  //
    //----------//

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

    // Accelerate motors; update the speed for all motors
    if (millis() - lastAccel >= 50)
    {
        lastAccel = millis();
        for (int i = 0; i < 4; i++)
        {
            motorList[i]->accelerate();
        }
    }

    // Heartbeat for REV motors
    if (millis() - lastHB >= 3)
    {
        lastHB = millis();
        CAN_sendHeartbeat(heartBeatNum);
        heartBeatNum++;
        if (heartBeatNum > 4)
        {
            heartBeatNum = 1;
        }
    }

    // Safety timeout
    if (millis() - lastCtrlCmd > 2000)  // if no control commands are received for 2 seconds
    {
        lastCtrlCmd = millis();

        // Only ignore safety timeout if all motors are rotating
        bool allRotating = true;
        for (int i = 0; i < 4; i++)
        {
            if (!motorList[i]->isRotToPos())
            {
                allRotating = false;
                break;
            }
        }
        if (!allRotating) {
            COMMS_UART.println("No Control, Safety Timeout");
            Stop();
        }
    }

    // Motor status debug printout
    if (millis() - lastMotorStatus > 500) {
        lastMotorStatus = millis();

        for (int i = 0; i < 4; i++) {
            if (millis() - motorList[i]->status1.timestamp > 500)  // Don't send outdated data
                continue;
            COMMS_UART.printf("motorstatus,%d,%d,%d,%d\n", motorList[i]->getID(), int(motorList[i]->status1.motorTemperature * 10),
                int(motorList[i]->status1.busVoltage * 10), int(motorList[i]->status1.outputCurrent * 10));
        }
    }


    //-------------//
    //  CAN Input  //
    //-------------//

    static CanFrame rxFrame;
    if (ESP32Can.readFrame(rxFrame, 1)) {
        uint8_t deviceId = rxFrame.identifier & 0x3F;  // [5:0]
        uint32_t apiId = (rxFrame.identifier >> 6) & 0x3FF;  // [15:6]

#if defined(DEBUG_STATUS)
        // Log message if it seems interesting
        if (apiId == 0x99 || (apiId & 0x60) == 0x60 || (apiId & 0x300) == 0x300) {
            printREVFrame(rxFrame);
        }
#endif

        if ((apiId & 0x60) == 0x60) {  // Periodic status
            for (int i = 0; i < 4; i++) {
                if (deviceId == motorList[i]->getID()) {
                    motorList[i]->parseStatus(apiId, rxFrame.data);
                    break;
                }
            }
        }
        else if ((apiId & 0x300) == 0x300) {  // Parameter
            printREVParameter(rxFrame);
#ifdef DEBUG
            Serial.print("From frame: ");
            printREVFrame(rxFrame);
#endif
        }
    }


    //------------------//
    //  UART/USB Input  //
    //------------------//
    //
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
    if (COMMS_UART.available()) {
        String input = COMMS_UART.readStringUntil('\n');

        input.trim();
        std::vector<String> args = {};
        parseInput(input, args);

#ifdef DEBUG
        Serial.println(input);
#endif
        String prevCommand;

        //--------//
        //  Misc  //
        //--------//

        /**/ if (args[0] == "ping")
        {
            Serial.println("pong");
            Serial1.println("pong");
        }

        else if (args[0] == "time") 
        {
            COMMS_UART.println(millis());
        }

        //-----------//
        //  Sensors  //
        //-----------//

        //----------//
        //  Motors  //
        //----------//

        else if (args[0] == "ctrl") // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        {   
            lastCtrlCmd = millis();
            if (input != prevCommand)
            {
                prevCommand = input;

                if (checkArgs(args, 2))
                {
                    motorList[0]->setDuty(args[1].toFloat());
                    motorList[1]->setDuty(args[1].toFloat());

                    motorList[2]->setDuty(-1 * args[2].toFloat());
                    motorList[3]->setDuty(-1 * args[2].toFloat());
                } else if (checkArgs(args, 1))
                {
                    motorList[0]->setDuty(args[1].toFloat());
                    motorList[1]->setDuty(args[1].toFloat());

                    motorList[2]->setDuty(-1 * args[1].toFloat());
                    motorList[3]->setDuty(-1 * args[1].toFloat());
                }
            }
        }

        else if (args[0] == "ctrl_send") {
            lastCtrlCmd = millis();

            if (checkArgs(args, 2))
            {
                motorList[0]->sendDuty(args[1].toFloat());
                motorList[1]->sendDuty(args[1].toFloat());

                motorList[2]->sendDuty(-1 * args[2].toFloat());
                motorList[3]->sendDuty(-1 * args[2].toFloat());
            } else if (checkArgs(args, 1))
            {
                motorList[0]->sendDuty(args[1].toFloat());
                motorList[1]->sendDuty(args[1].toFloat());

                motorList[2]->sendDuty(-1 * args[1].toFloat());
                motorList[3]->sendDuty(-1 * args[1].toFloat());
            }
        }

        else if (args[0] == "brake") 
        {
            if (args[1] == "on") 
            {
                Brake(true);
            }

            else if (args[1] == "off")
            {
                Brake(false);
            }
        }

        else if (args[0] == "auto") // Commands for autonomy
        { 
            lastCtrlCmd = millis();
            if (input != prevCommand)
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

        else if (args[0] == "forward") {
            driveMeters(args[1].toFloat());
        }

        else if (args[0] == "id") {
            CAN_identifySparkMax(args[1].toInt());
        }

        else if (args[0] == "stop") {
            Stop();
        }

#ifdef DEBUG
        else if (args[0] == "speed" && checkArgs(args, 1)) {
            CAN_sendVelocity(MOTOR_ID_BL, args[1].toFloat());
        }
#endif
    }
}


//------------------------------------------------------------------------------------------------//
//  Function definitions
//------------------------------------------------------------------------------------------------//
//
//
//----------------------------------------------------//
//                                                    //
//    //////////    //          //      //////////    //
//    //            //\\        //    //              //
//    //            //  \\      //    //              //
//    //////        //    \\    //    //              //
//    //            //      \\  //    //              //
//    //            //        \\//    //              //
//    //            //          //      //////////    //
//                                                    //
//----------------------------------------------------//

// Bypasses the acceleration to make the rover turn clockwise
// Should only be used for autonomy
void turnCW()
{
    float speed = 0.6;
    motorList[0]->sendDuty(speed);
    motorList[1]->sendDuty(speed);
    motorList[2]->sendDuty(-1 * speed);
    motorList[3]->sendDuty(-1 * speed);
}

// Bypasses the acceleration to make the rover turn counterclockwise
// Should only be used for autonomy
void turnCCW()
{
    float speed = 0.6;
    motorList[0]->sendDuty(-1 * speed);
    motorList[1]->sendDuty(-1 * speed);
    motorList[2]->sendDuty(speed);
    motorList[3]->sendDuty(speed);
}

// Bypasses the acceleration to make the rover stop
// Should only be used for autonomy, but it could probably be used elsewhere
void Stop()
{
    for (int i = 0; i < 4; i++) {
        motorList[i]->stop();
    }
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

void driveMeters(float meters) {
    const float degrees = (meters / WHEEL_CIRCUMFERENCE) * 360.0;

    // Left motors
    Motor1.turnByDeg(degrees);
    Motor2.turnByDeg(degrees);
    // Right motors
    Motor3.turnByDeg(-1 * degrees);
    Motor4.turnByDeg(-1 * degrees);
}

float getDriveSpeed() {
    float sum;
    for (int i = 0; i < 4; i++) {
        sum += abs(motorList[i]->status1.sensorVelocity);
    }
    const float avgSpeed = sum / 4;  // RPM
    const float gearBox = 64;  // 64:1 for testbed
    return (avgSpeed / gearBox) * WHEEL_CIRCUMFERENCE / 60;  // meters per second
}
