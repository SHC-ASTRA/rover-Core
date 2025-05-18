/**
 * @file CoreMotorMCU.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Core Motor MCU pins
 *
 */
#pragma once


#if !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)

//------------------------------------------------------------------------------------------------//
//   DOIT ESP32 Devkit V1 (URC 2025, Core V2)
//------------------------------------------------------------------------------------------------//

#    warning "Pins not added because this PCB is not going to be used."


#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)

//------------------------------------------------------------------------------------------------//
//   Feather ESP32 (URC 2025, Core V1)
//------------------------------------------------------------------------------------------------//

// CAN
#    define CAN_TX 13
#    define CAN_RX 12

// REV Motor IDs
#    define MOTOR_ID_FL 2  // REV motor ID for front left wheel
#    define MOTOR_ID_FR 1  // REV motor ID for front right wheel
#    define MOTOR_ID_BL 4  // REV motor ID for back left wheel
#    define MOTOR_ID_BR 3  // REV motor ID for back right wheel

#    define WHEEL_CIRCUMFERENCE 1.064  // Wheel's circumference in meters

#    define MOTOR_AMOUNT 4


#endif
