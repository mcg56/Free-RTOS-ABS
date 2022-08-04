#include "wheels.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "utils/ustdlib.h"
#include <stdio.h>

#define ALPHA_MAX 29.1


float calculateSteeringAngle(float steeringWheelDuty)
{
    return ALPHA_MAX*((steeringWheelDuty - 50.0)/45.0);
}


void calculateWheelRadii(Wheel* innerRear, Wheel* innerFront, Wheel* outerRear, Wheel* outerFront, float alpha)
{
    innerRear->turnRadius = 2.5/tan(fabs(alpha*PI/180.0));
    innerFront->turnRadius = 2.5/sin(fabs(alpha*PI/180.0));
    outerRear->turnRadius = innerRear->turnRadius + 1.5;
    outerFront->turnRadius = innerFront->turnRadius + 1.5;
}


void calculateWheelSpeedsFromRadii(Wheel* leftFront, Wheel* leftRear, Wheel* rightFront, Wheel* rightRear, float carSpeed)
{
    float centerRadius = (leftRear->turnRadius + rightRear->turnRadius)/2;
    float angularSpeed = carSpeed/centerRadius;

    leftFront->speed = angularSpeed*leftFront->turnRadius;
    leftRear->speed = angularSpeed*leftRear->turnRadius;
    rightFront->speed = angularSpeed*rightFront->turnRadius;
    rightRear->speed = angularSpeed*rightRear->turnRadius;
}


void calculateWheelPwmFreq(Wheel* leftFront, Wheel* leftRear, Wheel* rightFront, Wheel* rightRear)
{
    leftFront->pulseHz = PULSES_PER_REV*leftFront->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(1.0*PI);
    leftRear->pulseHz = PULSES_PER_REV*leftRear->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(1.0*PI);
    rightFront->pulseHz = PULSES_PER_REV*rightFront->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(1.0*PI);
    rightRear->pulseHz = PULSES_PER_REV*rightRear->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(1.0*PI);
}

void detectWheelSlip(Wheel* leftFront, Wheel* leftRear, Wheel* rightFront, Wheel* rightRear, char *slipArray, uint8_t condition, bool pedal, uint8_t pressure)
{
    int8_t m = -1;
    int8_t c;
    int8_t y = pressure;
    int8_t x;
    uint8_t minspeed = 10;
    if (pedal){
        if (condition == 0) {
            c = 120;
        } else if (condition == 1){
            c = 100;
        } else if (condition == 2){
            c = 80;
        }

        for (int8_t i = 0; i < 4; i++){
            if (i == 0) {
                x = leftFront->speed;
            } else if (i == 1){
                 x = leftRear->speed;
            } else if (i == 2){
                 x = rightFront->speed;
            } else if (i == 3){
                 x = rightRear->speed;
            }
            if ((x >= minspeed) && (y >= m*x + c) ){
                slipArray[i] = 1;
            } else {
                slipArray[i] = 0;
            } //TO DO SET FREQ Here 
        }
    } else {
        slipArray[0] = 0;
        slipArray[1] = 0;
        slipArray[2] = 0;
        slipArray[3] = 0;
    }
}

