/**
 * @file Main.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @author Charles Marmann (cmm0077@uah.edu)
 * @brief Core Embedded Main MCU
 *
 */

//------------//
//  Includes  //
//------------//

#include <Arduino.h>
#include <cmath>
#include <utility/imumaths.h>
#include <FastLED.h>
// Our own resources
#if defined(TESTBED)
#   include "project/TESTBED.h"
#else
#   include "project/CORE.h"
#endif
#include "AstraMisc.h"
#include "AstraVicCAN.h"
#include "AstraSensors.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

// strip 1: 1-40
// strip 2: 41-82
// strip 3: 83-124
// strip 4: 125-166
// CCW: 1,2,3,4
#define NUM_LEDS 166


//---------------------//
//  Component classes  //
//---------------------//

// LED Strip
int led_rbg[3] = {0, 300, 0}; //When using multiple colors, use 255 max, when doing R/B/G use 800-900 for best brightness
int led_counter = 0;
CRGB leds[NUM_LEDS];

//Sensor declarations

Adafruit_BMP3XX bmp;

SFE_UBLOX_GNSS myGNSS;

Adafruit_BNO055 bno;

AstraCAN Can0;


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

unsigned long clockTimer = 0;
unsigned long lastFeedback = 0;
unsigned long lastCtrlCmd = 0;


//--------------//
//  Prototypes  //
//--------------//

int findRotationDirection(float current_direction, float target_direction);
bool autoTurn(int time,float target_direction);
String outputBno();
String outputBmp();
void outputGPS();
void setLED(int r_val, int b_val, int g_val);
void safety_timeout();


//--------//
//  Misc  //
//--------//

String feedback;


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


    //-----------//
    //  MCU LED  //
    //-----------//

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);


    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);
    COMMS_UART.begin(COMMS_UART_BAUD);

    if(Can0.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX))
        Serial.println("CAN bus started!");
    else
        Serial.println("CAN bus failed!");


    //-----------//
    //  Sensors  //
    //-----------//

    if(!bno.begin()) 
        Serial.println("!BNO failed to start...");
    else 
        Serial.println("BNO055 Started Successfully");

    if(!bmp.begin_I2C()) 
        Serial.println("bmp not working");
    else 
        Serial.println("bmp is working");

    if(!myGNSS.begin()) 
        Serial.println("GPS not working");
    else 
        Serial.println("GPS is working");

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


    //--------------------//
    //  Misc. Components  //
    //--------------------//

    FastLED.addLeds<WS2812B, PIN_LED_STRIP, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(255);
    for(int i = 0; i < NUM_LEDS; ++i)
    {
      leds[i] = CRGB(led_rbg[0], led_rbg[1], led_rbg[2]);
      FastLED.show();
      delay(10);
    }
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
#ifdef BLINK
    if (millis() - lastBlink > 1000) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
#endif

    if((millis()-lastFeedback)>=2000)
    {
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

        //getPosition(myGNSS, gpsData);
        pullBNOData(bno, bnoData2);
        
        lastFeedback = millis();
    }

    // Safety timeout if no ctrl command for 2 seconds
    safety_timeout();


    //-------------//
    //  CAN Input  //
    //-------------//

    static CanFrame rxFrame;
    if(Can0.readFrame(rxFrame, 100)) {
        Serial.printf("Received frame: %03X  \r\n", rxFrame.identifier);
        // Vehicle CAN code will go here
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
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');

        input.trim();                   // Remove preceding and trailing whitespace
        std::vector<String> args = {};  // Initialize empty vector to hold separated arguments
        parseInput(input, args, ',');   // Separate `input` by commas and place into args vector
        args[0].toLowerCase();          // Make command case-insensitive
        String command = args[0];       // To make processing code more readable

        String prevCommand;

        //--------//
        //  Misc  //
        //--------//
        /**/ if (command == "ping") {
            Serial.println("pong");
        }

        else if (command == "time") {
            Serial.println(millis());
        }

        else if (command == "led") {
            if (args[1] == "on")
                digitalWrite(LED_BUILTIN, HIGH);
            else if (args[1] == "off")
                digitalWrite(LED_BUILTIN, LOW);
            else if (args[1] == "toggle") {
                ledState = !ledState;
                digitalWrite(LED_BUILTIN, ledState);
            }
        }

        //-----------//
        //  Sensors  //
        //-----------//

        else if (args[0] == "data") // Send data out
        {

            if(args[1] == "sendGPS") // data,sendGPS
            {
                outputGPS();
            }

            else if(args[1] == "sendIMU") // data,sendIMU
            {
                Serial.println(outputBno());
            }

            else if(args[1] == "sendBMP") // data,sendBMP
            {
                Serial.println(outputBmp());
            }

            else if(args[1] == "everything") // data,everything
            { 
                //Serial.println(outputGPS());
                Serial.println(outputBno());
                Serial.println(outputBmp());
            }
            
            else if(args[1] == "getOrientation") // data,getOrientation
            { 
                Serial.printf("orientation,%f\n", getBNOOrient(bno));
            }
        }

        //------------//
        //  Physical  //
        //------------//

        else if (args[0] == "ctrl" || args[0] == "ctrl_send" || args[0] == "brake") // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        {
            Serial1.println(command);
        }

        else if (args[0] == "joystick_ctrl")  // Takes X and Y position of controller's joystick
        {
            if (checkArgs(args, 2))
            {
                #define JOYSTICK_MAX 1.0

                // Inputs
                float joy_x = args[1].toFloat();
                float joy_y = args[2].toFloat();
                // Outputs
                float left_motor_duty;
                float right_motor_duty;

                // Speed rover will drive is distance of joystick away from center
                float driveSpeed = map(sqrt(joy_x*joy_x + joy_y*joy_y), 0, JOYSTICK_MAX, 0, 1);

                // Use positive joy_x by default

                left_motor_duty = joy_y >= 0 ? driveSpeed : -1 * driveSpeed;  // Positive forwards, negative backwards
                right_motor_duty = map(joy_y, -JOYSTICK_MAX, JOYSTICK_MAX, -1 * driveSpeed, driveSpeed);

                // Flip if joy_x negative
                if (joy_x < 0) {
                    const float temp = left_motor_duty;
                    left_motor_duty = right_motor_duty;
                    right_motor_duty = temp;
                }

                // Send to motor mcu
                Serial1.print("ctrl,");
                Serial1.print(left_motor_duty);
                Serial1.print(",");
                Serial1.println(right_motor_duty);
            }
        }

        else if (args[0] == "auto") // Commands for autonomy
        {

            lastCtrlCmd = millis();
            if(command != prevCommand)
            {

                if(args[1] == "turningTo") // auto,turningTo
                {

                    bool success = false;

                    success = autoTurn(args[2].toFloat(),args[3].toFloat());
                    if(success)
                    {
                        Serial.println("turningTo,success");
                    }
                    else
                    {
                        Serial.println("turningTo,fail");
                    }

                }

                else if(args[1] == "forwards") // auto,forwards
                {  
                    Serial1.println(command);
                }

                else if(args[1] == "backwards") // auto,backwards
                { 
                    Serial1.println(command);
                }

                else if(args[1] == "stop") // auto,stop
                {  
                    Serial1.println(command);
                }

            }
            else
            {
                //pass if command if control command is same as previous
            }

        }

        else if (args[0] == "led_set") //set LED strip color format: led_set,r,b,g
        {
            for(int i = 0; i < 3; i++)
            {
                led_rbg[i] = args[i+1].toInt();
            }
            
            setLED(led_rbg[0], led_rbg[1], led_rbg[2]);
        }
    }

    // Relay data from the motor controller back over USB
    if (COMMS_UART.available())
    {
        String input = COMMS_UART.readStringUntil('\n');
        input.trim();
        Serial.println(input);
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

void safety_timeout(){
  if(millis() - lastCtrlCmd > 2000)//if no control commands are received for 2 seconds
  {
    lastCtrlCmd = millis();//just update the var so this only runs every 2 seconds.

    Serial1.println("ctrl,0,0");
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

// Specify where the rover should turn
// and how long it should take before
// the rover decides that it has failed
// to turn
bool autoTurn(int time, float target_direction)
{

    int startTime = millis(); 
    unsigned long expectedTime;
    expectedTime = time;
    
    float current_direction = getBNOOrient(bno);
    bool turningRight = findRotationDirection(current_direction, target_direction);

    Serial.printf("StartTime: %d, Expected: %d",(int)startTime, (int)expectedTime);


    while (millis() - startTime < expectedTime)
    {

        current_direction = getBNOOrient(bno);
        if (!((current_direction < target_direction + 2) && (current_direction > target_direction - 2)))
        {
        
            turningRight = findRotationDirection(current_direction, target_direction);

            Serial.print("Turning to: ");
            Serial.println(target_direction);
            Serial.print("Currently at: ");
            Serial.println(getBNOOrient(bno));

            if (turningRight)
            {
                Serial1.println("auto,TurnCW");
            }
            else
            {
                Serial1.println("auto,TurnCW");
            }

        }
        else
        {
            Serial1.println("auto,stop");
            return true;
        }

    }

    Serial1.println("auto,stop");
    return false;
}


// Finds out which direction the rover should turn
int findRotationDirection(float current_direction, float target_direction)
{
    int cw_dist = target_direction - current_direction + 360;
    cw_dist %= 360;
    int ccw_dist = current_direction - target_direction + 360; 
    ccw_dist %= 360;

    if (cw_dist <= ccw_dist)
    {
        return 1;//Rotate CW if distance is 180 or less
    }
    else
    {
        return 0;//Rotate CCW if distance is greater than 180
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
