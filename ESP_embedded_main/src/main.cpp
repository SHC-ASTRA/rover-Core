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
#if defined(TESTBED)
#   include "project/TESTBED.h"
#else
#   include "project/CORE.h"
#endif
#include "AstraMisc.h"
#include "AstraMotors.h"
#include "AstraCAN.h"
#include "AstraSensors.h"

using namespace std;


//REMOVE_LED#define NUM_LEDS 166
//strip 1: 1-40
//strip 2: 41-82
//strip 3: 83-124
//strip 4: 125-166
//CCW: 1,2,3,4

//REMOVE_LEDint led_rbg[3] = {0, 300, 0}; //When using multiple colors, use 255 max, when doing R/B/G use 800-900 for best brightness
//REMOVE_LEDint led_counter = 0;

//REMOVE_LEDCRGB leds[NUM_LEDS];


//Sensor declarations

Adafruit_BMP3XX bmp;

SFE_UBLOX_GNSS myGNSS;

Adafruit_BNO055 bno = Adafruit_BNO055();


//Setting up for CAN0 line
AstraCAN Can0;

//Prototypes
int findRotationDirection(float current_direction, float target_direction);
bool autoTurn(int time,float target_direction);
String outputBno();
String outputBmp();
void outputGPS();
//REMOVE_LEDvoid setLED(int r_val, int b_val, int g_val);
void safety_timeout();


String feedback;

unsigned long clockTimer = millis();
unsigned long lastFeedback;
unsigned long lastCtrlCmd;


void setup() 
{

    //-----------------//
    // Initialize Pins //
    //-----------------//

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(SERIAL_BAUD);
    Serial1.begin(SERIAL_BAUD);
    digitalWrite(LED_BUILTIN, HIGH);

    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);

    
    // Setup CAN
    if(Can0.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX)) 
    {
        Serial.println("CAN bus started!");
    } 
    else 
    {
        Serial.println("CAN bus failed!");
    }

    // Old, may or may not be needed
    // pinMode(20, INPUT_PULLUP); //Needed for IMU to work on PCB


    //REMOVE_LEDFastLED.addLeds<WS2812B, PIN_LED_STRIP, GRB>(leds, NUM_LEDS);
    //REMOVE_LEDFastLED.setBrightness(255);
    //REMOVE_LEDfor(int i = 0; i < NUM_LEDS; ++i)
    //REMOVE_LED{
    //REMOVE_LED  leds[i] = CRGB(led_rbg[0], led_rbg[1], led_rbg[2]);
    //REMOVE_LED  FastLED.show();
    //REMOVE_LED  delay(10);
    //REMOVE_LED}


    //--------------------//
    // Initialize Sensors //
    //--------------------//

    if(!bno.begin()) 
    {
        Serial.println("!BNO failed to start...");
    } 
    else 
    {
        Serial.println("BNO055 Started Successfully");
    }

    if(!bmp.begin_I2C()) 
    {
        Serial.println("bmp not working");
    } 
    else 
    {
        Serial.println("bmp is working");
    }

    if(!myGNSS.begin()) 
    {
        Serial.println("GPS not working");
    }
    else 
    {
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

    //----------------------------------//
    // Runs something at a set interval //
    // Useful for testing               //
    //----------------------------------//

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

    if (Serial.available()) 
    {

        String command = Serial.readStringUntil('\n');
        command.trim();

        String prevCommand;

        std::vector<String> args = {};
        parseInput(command, args, ',');

        if (args[0] == "ping") 
        {
            Serial.println("pong");
        } 

        else if (args[0] == "time") 
        {
            Serial.println(millis());
        }

        else if (args[0] == "ctrl") // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        {                          
            Serial1.println(command);
        }

        else if (args[0] == "speedMultiplier") // Is looking for a command that looks like "ctrl,x" where 0<x<1
        {
            Serial1.println(command);
        }

        else if (args[0] == "brake") 
        {
            Serial1.println(command);
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

        else if (args[0] == "led_set") //set LED strip color format: led_set,r,b,g
        {   
            for(int i = 0; i < 3; i++)
            {
                //REMOVE_LEDled_rbg[i] = args[i+1].toInt();
            }
            
            //REMOVE_LEDsetLED(led_rbg[0], led_rbg[1], led_rbg[2]);

        } 

    }

    // // Relay data from the motor controller back over USB
    // if (COMMS_UART.available())
    // {
    //     Serial.println(COMMS_UART.readStringUntil('\n').trim());
    // }

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
    //REMOVE_LEDfor(int i = 0; i < NUM_LEDS; ++i)
    //REMOVE_LED{
    //REMOVE_LED  leds[i] = CRGB(r_val, b_val, g_val);
    //REMOVE_LED  FastLED.show();
    //REMOVE_LED  delay(10);
    //REMOVE_LED}
}
