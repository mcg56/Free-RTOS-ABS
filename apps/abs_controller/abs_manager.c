/**********************************************************
 *
 * abs_manager.c - Main controlling file for the 
 *      ABS information.
 *
 * T.R Peterson, M.C Gardyne
 * Last modified:  24.7.22
 **********************************************************/
#include "libs/lib_pwm/ap_pwm_input.h"
#include "abs_manager.h"
#include "math.h"


#define DIAMETER                0.5
#define HALF_DUTY               45
#define MAX_ANGLE               29.1
#define TRACK                   1.5

/**
 * This will contain:
 * 
 * 1. The calculation for what brake output is required given the PWM signals.
 * 2. Probably the LED stuff unless we make another file
 * 
 */

CarAttributes_t mondeo;

typedef enum {
    REAR_LEFT = 0, 
    REAR_RIGHT, 
    FRONT_LEFT, 
    FRONT_RIGHT
} Wheels;

/**
 * @brief Calculates the hypothetical vehicle velocity from wheel speed
 * @param -
 * @return -
 */
uint32_t calcCarVel(uint32_t wheel)
{
    // Vehicle turning angle
    float alpha = mondeo.steeringAngle*(M_PI/180);

    // Definition of turn radii from wheels.c may try combine later 
    float innerRear = 2.5/tan(fabs(alpha*M_PI/180.0));
    float innerFront = 2.5/sin(fabs(alpha*M_PI/180.0));
    float outerRear = innerRear + TRACK;
    float outerFront = innerFront + TRACK;
    float normWheelOrder[4] = {innerRear, outerRear, innerRear, outerFront};
    float currentRadii;
    if (alpha >= 0) {
        currentRadii = normWheelOrder[wheel];        
    } else {
        if ((wheel%2) == 0) {
            currentRadii = normWheelOrder[wheel+1];
        } else {
            currentRadii = normWheelOrder[wheel-1];
        }
    }
    uint32_t result = mondeo.wheelVel[wheel]*(outerFront/currentRadii);
    return result;

}
/**
 * @brief Calculates the wheel velocity based on input frequency
 * @param frequency Duty cycle of steering input
 * @return Int (wheel speed)
 */
uint32_t calcWheelVel(uint32_t frequency)
{
    return (3.6*DIAMETER*M_PI*frequency)/(20);
}

/**
 * @brief Calculates the turn angle based off input PWM signal
 * @param duty Duty cycle of steering input
 * @return Float (steering angle)
 */
float calcAngle(uint32_t duty)
{
    return (50-duty)*(MAX_ANGLE / HALF_DUTY);
}


bool checkSlippySloppy(void)
{
    bool slip = false;
    
    PWMSignal_t steering = getPWMInputSignal("Steering");
    mondeo.steeringAngle = calcAngle(steering.duty);

    mondeo.wheelVel[REAR_LEFT] = calcWheelVel(getPWMInputSignal("LR").frequency);
    mondeo.wheelVel[REAR_RIGHT] = calcWheelVel(getPWMInputSignal("RR").frequency);
    mondeo.wheelVel[FRONT_LEFT] = calcWheelVel(getPWMInputSignal("LF").frequency);
    mondeo.wheelVel[FRONT_RIGHT] = calcWheelVel(getPWMInputSignal("RF").frequency);


    // Calculate hypothetical speed
    uint32_t calcHypoVel[4];

    calcHypoVel[REAR_LEFT] = calcCarVel(REAR_LEFT);
    calcHypoVel[REAR_RIGHT] = calcCarVel(REAR_RIGHT);
    calcHypoVel[FRONT_LEFT] = calcCarVel(FRONT_LEFT);
    calcHypoVel[FRONT_RIGHT] = calcCarVel(FRONT_RIGHT);
    mondeo.carVel = 0;
    for (uint32_t i = 0; i<4;i++){
        if (calcHypoVel[i] > mondeo.carVel) {
            mondeo.carVel = calcHypoVel[i];
        }
    }
    // Change to percentage
    for (uint32_t i = 0; i<4;i++){
        if ((mondeo.carVel - calcHypoVel[i])>5) {
            slip = true;
        }
    }
    char str[100];
    //sprintf(str, "Not sliping %d\r\n\n", mondeo.carVel);
    sprintf(str, "speed %d\r\n\n", getPWMInputSignal("LR").frequency);
    UARTSend(str);
    return slip;
}
