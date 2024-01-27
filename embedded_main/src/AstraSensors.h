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

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(Adafruit_BNO055 &bno)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(Adafruit_BNO055 &bno)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(Adafruit_BNO055 &bno)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

bool isCalibrated(Adafruit_BNO055 &bno)
{
    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    const int eeAddress = 0;
    long bnoID;
    sensor_t sensor;
    EEPROM.get(eeAddress, bnoID);   
    bno.getSensor(&sensor);
    if(bnoID != sensor.sensor_id)
        return false;
    return true;
}

void loadCalibration(Adafruit_BNO055 &bno)
{
    adafruit_bno055_offsets_t calibrationData;
    int eeAddress;
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    displaySensorOffsets(calibrationData);
    bno.setSensorOffsets(calibrationData);
    displaySensorDetails(bno);
    displaySensorStatus(bno);
    bno.setExtCrystalUse(1);
}

void calibrateBNO(Adafruit_BNO055 &bno)
{
    sensors_event_t event;
    sensor_t sensor;
    while(!bno.isFullyCalibrated())
    {
        bno.getEvent(&event);
        displayCalStatus(bno);
        delay(100);
    }
    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    int eeAddress = 0;
    bno.getSensor(&sensor);
    long bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
    delay(500);
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