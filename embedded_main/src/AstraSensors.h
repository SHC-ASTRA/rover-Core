#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>


void pullBNOData(Adafruit_BNO055 &bno, float (& bno_data)[7])
{
    sensors_event_t event;
    bno.getEvent(&event);

    bno_data[0] = event.gyro.x;//pitch
    bno_data[1] = event.gyro.z;//roll
    bno_data[2] = event.gyro.y;//yaw

    bno_data[3] = event.acceleration.x;//pitch
    bno_data[4] = event.acceleration.z;//roll
    bno_data[5] = event.acceleration.y;//yaw

    bno_data[6] = event.orientation.x;//Absolute Orientation/Heading 

    /*
    float mag_p = event.magnetic.x;//pitch
    float mag_r = event.magnetic.z;//roll
    float mag_y = event.magnetic.y;//yaw
    */
}

float getBNOOrient(Adafruit_BNO055 &bno)
{
    sensors_event_t event;
    bno.getEvent(&event);

    return event.orientation.x;//Absolute Orientation/Heading 
}