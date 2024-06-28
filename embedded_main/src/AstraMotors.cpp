#include "AstraMotors.h"


long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

AstraMotors::AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty) {
    controlMode = setCtrlMode; //0-Speed 1-Duty Cycle

    currentDutyCycle = 0;
    setDutyCycle = 0;
    
    currentMotorSpeed = 0;
    setMotorSpeed = 0;   

    maxSpeed = setMaxSpeed;
    maxDuty = setMaxDuty;

	dutyCycleAccel = 0.05;
	
	motorID = setMotorID;

    inverted = inv;
}

//Add the funky graph
float AstraMotors::convertControllerValue(float stickValue){
    float output = stickValue;

    if(!inverted){
        output *= -1;
    }

    if(controlMode == 0)//speed Control mode
    {
        output = map(output, -1, 1, (-1 * maxSpeed), maxSpeed);
    }else{//duty cycle control mode
        output = map(output, -1, 1, (-1 * maxDuty), maxDuty);
    }
    
    return output;
}


int AstraMotors::getControlMode(){
    return controlMode;
}


void AstraMotors::setSpeed(float val){//controller input value
    if(abs(val) <= 0.02)
    {
        setMotorSpeed = 0;
    }else{
        setMotorSpeed = convertControllerValue(val);
    }
    
}

int AstraMotors::getSpeed(){
    return currentMotorSpeed;
}

int AstraMotors::getID(){
    return motorID;
}


void AstraMotors::setDuty(float val){//controller input value
    if(abs(val) <= 0.02)
    {
        setDutyCycle = 0;
    }else{
        setDutyCycle = convertControllerValue(val);
    }
}

float AstraMotors::getDuty(){
    return currentDutyCycle;
}
float AstraMotors::getSetDuty(){
    return setDutyCycle;
}


void AstraMotors::UpdateForAcceleration(){
	float dCThreshold = 0.1;
    float cD = currentDutyCycle;
    float sD = setDutyCycle;

	//if(controlMode == 1){
    if(setDutyCycle != 0){
		if((cD <= sD + 0.1) && (cD >= sD - 0.1)){//if within 0.1 of desired. Just set it, don't gradually accelerate
			currentDutyCycle = setDutyCycle;
		}else if(cD < sD - dCThreshold){//increment if below set
			currentDutyCycle += dutyCycleAccel;
		}else if(cD > sD + dCThreshold){//decrement if above set
			currentDutyCycle -= dutyCycleAccel;
		}else{
			if((cD > 0 && sD < 0) || (cD < 0 && sD > 0))//if sticks in opposite direction, quick stop
            {
                currentDutyCycle = 0;
                setDutyCycle = 0;
            }
            currentDutyCycle = 0;
		}
    }else{//if set 0
        currentDutyCycle = 0;
    }
	//}

}