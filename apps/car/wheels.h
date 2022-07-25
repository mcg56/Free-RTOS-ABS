#ifndef WHEELS_H_
#define WHEELS_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "utils/ustdlib.h"

#define PULSES_PER_REV 20     // (#)
#define WHEEL_DIAMETER 0.5    // (m)
#define KPH_TO_MS_SCALE_FACTOR (1000/3600)
#define PI (3.141592653589793)

/**
 * @brief Structure for storing information about car wheels
 * @param turnRadius The turn radius of the wheel (m)
 * @param speed The tangential velocity of the wheel (km/h)
 * @param pulseHz The pulse rate of wheel pwm encoder (Hz) 
 */
typedef struct {
    float turnRadius; //m
    float speed; //km/h
    float pulseHz; //Hz
} Wheel;

float calculateSteeringAngle(float steeringWheelDuty);
void calculateWheelRadii(Wheel* innerRear, Wheel* innerFront, Wheel* outerRear, Wheel* outerFront, float alpha);
void calculateWheelSpeedsFromRadii(Wheel* leftFront, Wheel* leftRear, Wheel* rightFront, Wheel* rightRear, float carSpeed);
void calculateWheelPwmFreq(Wheel* leftFront, Wheel* leftRear, Wheel* rightFront, Wheel* rightRear);

#endif