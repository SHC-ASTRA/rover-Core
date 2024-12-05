/**
 * @file AstraWheelBase.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @brief 
 * 
 */

#include "AstraWheelBase.h"


AstraWheelBase::AstraWheelBase(AstraMotors* setMotorList[4]) {
    for (int i = 0; i < 4; i++)
        motorList[i] = setMotorList[i];
}

void AstraWheelBase::setDuty(float duty) {
    motorList[0]->setDuty(duty);
    motorList[1]->setDuty(duty);
    motorList[2]->setDuty(duty * -1);
    motorList[3]->setDuty(duty * -1);
}

void AstraWheelBase::setDuty(float dutyLeft, float dutyRight) {
    motorList[0]->setDuty(dutyLeft);
    motorList[1]->setDuty(dutyLeft);
    motorList[2]->setDuty(dutyRight);
    motorList[3]->setDuty(dutyRight);
}

void AstraWheelBase::setSpeed(float speed) {
    motorList[0]->setSpeed(speed);
    motorList[1]->setSpeed(speed);
    motorList[2]->setSpeed(speed * -1);
    motorList[3]->setSpeed(speed * -1);
}

void AstraWheelBase::setSpeed(float speedLeft, float speedRight) {
    motorList[0]->setSpeed(speedLeft);
    motorList[1]->setSpeed(speedLeft);
    motorList[2]->setSpeed(speedRight);
    motorList[3]->setSpeed(speedRight);
}

void AstraWheelBase::goForwards(float duty) {
    setDuty(duty);
}

void AstraWheelBase::goBackwards(float duty) {
    setDuty(duty * -1);
}

void AstraWheelBase::turnCW(float duty) {
    motorList[0]->setDuty(duty);
    motorList[1]->setDuty(duty);
    motorList[2]->setDuty(duty);
    motorList[3]->setDuty(duty);
}

void AstraWheelBase::turnCCW(float duty) {
    motorList[0]->setDuty(duty * -1);
    motorList[1]->setDuty(duty * -1);
    motorList[2]->setDuty(duty * -1);
    motorList[3]->setDuty(duty * -1);
}

void AstraWheelBase::Stop() {
    for (int i = 0; i < 4; i++)
        motorList[i]->stop();
}

void AstraWheelBase::Brake(bool enable) {
    for (int i = 0; i < 4; i++)
        motorList[i]->setBrake(enable);
}

void AstraWheelBase::driveMeters(float meters) {
    const float degrees = (meters / WHEEL_CIRCUMFERENCE) * 360.0;

    // Left motors
    motorList[0]->turnByDeg(degrees);
    motorList[1]->turnByDeg(degrees);
    // Right motors
    motorList[2]->turnByDeg(degrees * -1);
    motorList[3]->turnByDeg(degrees * -1);
}

float AstraWheelBase::getDriveSpeed() {
    float sum;
    for (int i = 0; i < 4; i++) {
        sum += abs(motorList[i]->status1.sensorVelocity);
    }
    const float avgSpeed = sum / 4;  // RPM
    const float gearBox = 64;  // 64:1 for testbed
    return (avgSpeed / gearBox) * WHEEL_CIRCUMFERENCE / 60;  // meters per second
}
