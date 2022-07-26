#include "wheels.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "utils/ustdlib.h"
#include <stdio.h>

Wheel leftFront  = {0, 0, 0};
Wheel leftRear   = {0, 0, 0};
Wheel rightFront = {0, 0, 0};
Wheel rightRear  = {0, 0, 0};

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
    leftFront->pulseHz = PULSES_PER_REV*leftFront->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(2*PI);
    leftRear->pulseHz = PULSES_PER_REV*leftRear->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(2*PI);
    rightFront->pulseHz = PULSES_PER_REV*rightFront->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(2*PI);
    rightRear->pulseHz = PULSES_PER_REV*rightRear->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(2*PI);
}