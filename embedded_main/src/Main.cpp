// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <utility/imumaths.h>
#include <FastLED.h>
// Our own resources
#include "AstraMotors.h"
#include "AstraCAN.h"
#include "AstraSensors.h"
#include "TeensyThreads.h"

using namespace std;


#define LED_STRIP_PIN     10
#define NUM_LEDS 38

int led_rbg[3] = {0, 300, 0}; //When using multiple colors, use 255 max, when doing R/B/G use 800-900 for best brightness
int led_counter = 0;

CRGB leds[NUM_LEDS];


#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)


#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

SFE_UBLOX_GNSS myGNSS;

//Sensor declarations
Adafruit_BNO055 bno = Adafruit_BNO055();

//Setting up for CAN0 line
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

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
void outputBno();
void outputBmp();
void outputGPS();
void setLED(int r_val, int b_val, int g_val);




unsigned long lastAccel;
unsigned long lastDuty;
unsigned long lastHB;


unsigned long clockTimer = millis();


void setup() {

  //-----------------//
  // Initialize Pins //
  //-----------------//
  
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    digitalWrite(LED_PIN, HIGH);

    delay(2000);
    digitalWrite(LED_PIN, LOW);

    // Initalization for using CAN with the sparkmax
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

    pinMode(20, INPUT_PULLUP); //Needed for IMU to work on PCB


    FastLED.addLeds<WS2812, LED_STRIP_PIN, GRB>(leds, NUM_LEDS);
    for(int i = 0; i < NUM_LEDS; ++i)
    {
      leds[i] = CRGB(led_rbg[0], led_rbg[1], led_rbg[2]);
      FastLED.show();
      delay(10);
    }




  //--------------------//
  // Initialize Sensors //
  //--------------------//
    if(!bno.begin())
    {
      Serial.println("!BNO failed to start...");
    }else{
      Serial.println("BNO055 Started Successfully");
    }if(!bmp.begin_I2C()) {
      Serial.println("bmp not working");
    }else{
      Serial.println("bmp is working");
    }if(!myGNSS.begin()){
      Serial.println("GPS not working");
    }else{
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
  
  

  //----------------------------------//
  // Runs something at a set interval //
  // Useful for testing               //
  //----------------------------------//

  // if(1){
  //   if((millis()-clockTimer)>50){
  //     clockTimer = millis();
  //     Serial.println("explode");
  //     sendDutyCycle(Can0, 1, 0.05);
  //     delay(1000);
  //     sendDutyCycle(Can0, 1, -0.05);
  //     delay(1000);
  //   }
  // }

  sendDutyCycle(Can0, 1, 0.05);

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