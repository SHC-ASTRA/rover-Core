/**
 * @file CoreMainMCU.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Core Main MCU pins
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

// Voltage Dividers
#    define PIN_VDIV_5V 33
#    define PIN_VDIV_BATT 15
#    define PIN_VDIV_12V 32
#    define PIN_VDIV_3V3 14

// LED Strip
#    define PIN_LED_STRIP 4

#    define MOTOR_AMOUNT 4


#endif
