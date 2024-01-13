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
	float dutyCycleThreshold = 1;

	//if(controlMode == 1)
	//{
		if((currentDutyCycle <= setDutyCycle + 1.0) && (currentDutyCycle >= setDutyCycle - 1.0))
		{
			currentDutyCycle = setDutyCycle;
		}else if(currentDutyCycle < setDutyCycle - dutyCycleThreshold){
			currentDutyCycle += dutyCycleAccel;
		}else if(currentDutyCycle > setDutyCycle + dutyCycleThreshold){
			currentDutyCycle -= dutyCycleAccel;
		}else{
			if((currentDutyCycle > 0 && setDutyCycle < 0) || (currentDutyCycle < 0 && setDutyCycle > 0))
            {
                currentDutyCycle = 0;
            }
            currentDutyCycle = 0;
		}

        

	//}

}