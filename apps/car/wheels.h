#ifndef WHEELS_H_
#define WHEELS_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "utils/ustdlib.h"

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
 * @brief Task to update the wheel information and signal to PWM generators to update the frequencies
 * @param args Unused
 * @return No return
 */
void updateWheelInfoTask(void* args);

void toggleABSTask(void* args);

#endif