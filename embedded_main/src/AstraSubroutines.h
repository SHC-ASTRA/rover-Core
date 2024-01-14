#include "AstraCAN.h"
#include "AstraMotors.h"
#include "AStraSensors.h"



/* bool orientDir(float orientGoal, long timeout, Adafruit_BNO055 &bno)
{
    float orientation = getBNOOrient(bno);//current orientation of rover
    unsigned long startTime = millis();

    //determine direction to turn 


    while(true)//repeating until timeout
    {


        if(millis()-startTime < timeout) return false;
    }

    return false; //False fallback
}
*/