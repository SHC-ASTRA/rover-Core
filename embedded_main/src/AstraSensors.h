#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 

#define SEALEVELPRESSURE_HPA (1013.25)


void calibrateBNO(Adafruit_BNO055 &bno)
{
    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;
    EEPROM.get(eeAddress, bnoID);
    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;
    bno.getSensor(&sensor);
    if(bnoID!=sensor.sensor_id)
    {
        eeAddress+=sizeof(long);
        EEPROM.get(eeAddress, calibrationData);
        bno.setSensorOffsets(calibrationData);
        foundCalib = true;
    }
    bno.setExtCrystalUse(true);
    sensors_event_t event;
    bno.getEvent(&event);
    if(!foundCalib)
    {
        while(!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(100);
        }
    }
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;
    EEPROM.put(eeAddress, bnoID);
    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
}

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

void initializeBMP(Adafruit_BMP3XX &bmp)
{
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void pullBMPData(Adafruit_BMP3XX &bmp, float(& bmp_data)[3])
{
    bmp_data[0] = bmp.temperature;
    bmp_data[1] = bmp.pressure/100.0;
    bmp_data[2] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

float getBNOOrient(Adafruit_BNO055 &bno)
{
    sensors_event_t event;
    bno.getEvent(&event);
    return event.orientation.x;//Absolute Orientation/Heading 
}

void getPosition(SFE_UBLOX_GNSS &myGNSS, float(& gps_data)[3])
{
    gps_data[0] = myGNSS.getLatitude()/10000000.0;
    gps_data[1] = myGNSS.getLongitude()/10000000.0;
    gps_data[2] = uint8_t(myGNSS.getSIV());
}

String getUTC(SFE_UBLOX_GNSS &myGNSS)
{
    return String(myGNSS.getHour())+"-"+String(myGNSS.getMinute())+"-"+String(myGNSS.getSecond());
}