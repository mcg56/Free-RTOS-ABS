#ifndef WHEELS_H_
#define WHEELS_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "utils/ustdlib.h"

#define PULSES_PER_REV 20.0     // (#)
#define WHEEL_DIAMETER 0.5    // (m)
#define KPH_TO_MS_SCALE_FACTOR (1000.0/3600.0)
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

/**
 * @brief function to calculate the steering angle
 * @param steeringWheelDuty Steering wheel pwm duty
 * @return alpha steering angle
 */
float calculateSteeringAngle(float steeringWheelDuty);

/**
 * @brief function to calculate the turn radius of each wheel
 * @param innerRear Inner rear wheel
 * @param innerFront Inner front wheel
 * @param outerRear Outer rear wheel
 * @param outerFront Outer front wheel
 * @param alpha steering angle
 * @return None
 */
void calculateWheelRadii(Wheel* innerRear, Wheel* innerFront, Wheel* outerRear, Wheel* outerFront, float alpha);

/**
 * @brief function to calculate the turn radius of each wheel based on turn radius and speed.
 * Calculates cars angular velocity while turning at centre of read axle (half of two rear radii).
 * This method is different to what phillip did and results in slightly different output (might
 * be wrong).
 * @param leftFront Left front wheel struct
 * @param leftRear Left rear wheel struct                
 * @param rightFront Right front wheel struct
 * @param rightRear Right rear wheel struct
 * @param carSpeed car speed
 * @return None
 */
void calculateWheelSpeedsFromRadii(Wheel* leftFront, Wheel* leftRear, Wheel* rightFront, Wheel* rightRear, float carSpeed);

/**
 * @brief Function to calculate the pwm pulse frequency of each wheel based on wheel speed. I think it should
 * divide by 2*pi as this is how many radians are in a circle, but the formula in the
 * specification says to only divide by pi.
 * @param leftFront Left front wheel struct
 * @param leftRear Left rear wheel struct                
 * @param rightFront Right front wheel struct
 * @param rightRear Right rear wheel struct
 * @return None
 */
void calculateWheelPwmFreq(Wheel* leftFront, Wheel* leftRear, Wheel* rightFront, Wheel* rightRear);


/**
 * @brief Function to calculate whether or not any wheel is slipping
 * specification says to only divide by pi.
 * @param leftFront     Left front wheel struct
 * @param leftRear      Left rear wheel struct                
 * @param rightFront    Right front wheel struct
 * @param rightRear     Right rear wheel struct
 * @param carSpeed      Speed of the vehicle
 * @param condition     Condition of the road
 * @return Bool         Wheel slip condition
 */
void detectWheelSlip(Wheel* leftFront, Wheel* leftRear, Wheel* rightFront, Wheel* rightRear,char *slipArray, uint8_t condition, bool pedal, uint8_t pressure);





#endif