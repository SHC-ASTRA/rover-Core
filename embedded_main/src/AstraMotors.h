//Header file for the ASTRA CAN class
//it is called Amotor so that it dosn't mess with other libraries
#pragma once
#include <Arduino.h>
#include <iostream>
#include <string>
#include <Servo.h>
#include <cmath>
#include <cstdlib>
#include <Wire.h>
#include <SPI.h>



class AstraMotors {
    int controlMode;                //0- Speed  1-Duty Cycle

    int currentMotorSpeed;         // Current speed of the motor
    int setMotorSpeed;             // What the speed of the motor should be

    float currentDutyCycle;
    float setDutyCycle;
    float dutyCycleAccel;

    int motorID;

    int maxSpeed;
    float maxDuty;

    bool inverted;                // Inverts the speed of the motor, this should be -1 for motor 3 and 4

public:
    //AstraMotors();                                      // Startup function
    //AstraMotors(bool inv);                              // Creates an object with inverted = -1
    AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty);
    float convertControllerValue(float stickvalue);    // Converts the joystick value -1 < 0 < 1 to 1700 < 1500 < 1300
    //void setMotorMultiplier(float val);                 // Set the speedMultiplier variable
    void setSpeed(float val);                           // Set the setMotorSpeed variable
    int getSpeed();                                     // Get the current speed
    int getID();                                        //Get the motor's set CAN ID
    void setDuty(float val);
    float getDuty();
    float getSetDuty();

    int getControlMode();
    
    void UpdateForAcceleration();                       // Update the current speed to try and match the setMotorSpeed variable

};
