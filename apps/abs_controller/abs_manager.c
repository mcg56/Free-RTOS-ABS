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
#include <FreeRTOS.h>
#include <queue.h>
#include "driverlib/timer.h"
#include "brake_output.h"


#define DIAMETER                0.5     // Wheel diameter (m)
#define HALF_DUTY               45      // Half of duty range (95-5)/2
#define MID_DUTY                50      // Mid of duty between 95-5
#define MAX_ANGLE               29.1    // Max absolute turn angle (deg)
#define TRACK                   1.5     // Wheel seperation (m)
#define TICKS_PER_REV           20      // Encoder pulses per single wheel revolution
#define TOLERANCE               10       // 5% velocity difference
#define FACTOR                  3.6     // km/h to m/s
#define SCALE_FACTOR            100     // Scale result to percentage
/**
 * This will contain:
 * 
 * 1. The calculation for what brake output is required given the PWM signals.
 * 2. Probably the LED stuff unless we make another file
 * 
 */

CarAttributes_t mondeo; // Probably not very reliable

typedef enum {
    REAR_LEFT = 0, 
    REAR_RIGHT, 
    FRONT_LEFT, 
    FRONT_RIGHT
} Wheels;

/**
 * @brief Calculates the hypothetical vehicle velocity based on outer-most wheel from individual wheel velocity
 * @param - Target wheel
 * @return - Hypothetical vehicle velocity
 */
uint32_t calcCarVel(uint32_t wheel)
{
    // Vehicle turning angle
    float alpha = mondeo.steeringAngle*(M_PI/180);

    float innerRear = 0;
    float innerFront = 0;
    float outerRear = 0;
    float outerFront = 0;
    uint32_t result = mondeo.wheelVel[wheel];

    if (alpha != 0) {
        innerRear = 2.5/tan(fabs(alpha));
        innerFront = 2.5/sin(fabs(alpha));
        outerRear = innerRear + TRACK;
        outerFront = innerFront + TRACK;
        // Definition of turn radii from wheels.c may try combine later 
        float normWheelOrder[NUM_WHEELS] = {innerRear, outerRear, innerFront, outerFront};
        float currentRadii;

        if (alpha > 0) {
            currentRadii = normWheelOrder[wheel];        
        } else {
            // If turning opposite way swap inner/outer pairs
            if ((wheel%2) == 0) { 
                currentRadii = normWheelOrder[wheel+1];
            } else {
                currentRadii = normWheelOrder[wheel-1];
            }
        }
        result = mondeo.wheelVel[wheel]*(outerFront/currentRadii);
    }
    return result;
}

/**
 * @brief Calculates the wheel velocity based on input frequency
 * @param frequency Duty cycle of steering input
 * @return Int (wheel speed)
 */
uint32_t calcWheelVel(uint32_t frequency)
{
    return (FACTOR*DIAMETER*M_PI*frequency)/(TICKS_PER_REV);
}

/**
 * @brief Calculates the turn angle based off input PWM signal
 * @param duty Duty cycle of steering input
 * @return Float (steering angle)
 */
float calcAngle(uint32_t duty)
{
    return (MID_DUTY-duty)*(MAX_ANGLE / HALF_DUTY);
}

/**
 * @brief Checks if vehicle is slipping or not
 * @param void
 * @return Boolean slip true/false
 */
void checkSlip(void)
{
    uint8_t state = ABS_OFF;
    mondeo.sold = false;
    
    mondeo.steeringAngle = calcAngle(getPWMInputSignal("Steering").duty);

    mondeo.wheelVel[REAR_LEFT] = calcWheelVel(getPWMInputSignal("LR").frequency);
    mondeo.wheelVel[REAR_RIGHT] = calcWheelVel(getPWMInputSignal("RR").frequency);
    mondeo.wheelVel[FRONT_LEFT] = calcWheelVel(getPWMInputSignal("LF").frequency);
    mondeo.wheelVel[FRONT_RIGHT] = calcWheelVel(getPWMInputSignal("RF").frequency);


    // Calculate hypothetical speed
    uint32_t calcHypoVel[NUM_WHEELS];
    calcHypoVel[REAR_LEFT] = calcCarVel(REAR_LEFT);
    calcHypoVel[REAR_RIGHT] = calcCarVel(REAR_RIGHT);
    calcHypoVel[FRONT_LEFT] = calcCarVel(FRONT_LEFT);
    calcHypoVel[FRONT_RIGHT] = calcCarVel(FRONT_RIGHT);
    mondeo.carVel = 0;
    for (uint32_t i = 0; i < NUM_WHEELS; i++){
        if (calcHypoVel[i] > mondeo.carVel) {
            mondeo.carVel = calcHypoVel[i];
        }
    }
    //good
    // 5% at the moment may need tuned
    for (uint32_t i = 0; i < NUM_WHEELS; i++){
        if ((mondeo.carVel - calcHypoVel[i]) > 5) {
            state = ABS_ON;
        }
        //(((mondeo.carVel - calcHypoVel[i])/mondeo.carVel)*SCALE_FACTOR > TOLERANCE)
    }
    char str[100];
    //sprintf(str, "Not sliping %d\r\n\n", mondeo.steeringAngle); 
    //sprintf(str, "diff %d\r\n\n", (mondeo.carVel - calcHypoVel[REAR_LEFT]));
    //UARTSend(str);
    //sprintf(str, "State %d\r\n\n", state);
    //sprintf(str, "RL %d\r\n\n", ((mondeo.carVel - calcHypoVel[REAR_LEFT])/mondeo.carVel)*SCALE_FACTOR);
    sprintf(str, "RL %d\r\n\n", calcHypoVel[REAR_LEFT]);
    UARTSend(str); 
    sprintf(str, "RR %d\r\n\n", calcHypoVel[REAR_RIGHT]);
    UARTSend(str); 
    sprintf(str, "FL %d\r\n\n", calcHypoVel[FRONT_LEFT]);
    UARTSend(str); 
    sprintf(str, "FR %d\r\n\n", calcHypoVel[FRONT_RIGHT]);
    UARTSend(str); 
    setABS(state);
}

/**
 * @brief Regularly scheduled task for updating all PWM signals
 * 
 * @return None
 */
void checkSlipTask(void* args)
{
    (void)args;
    const TickType_t xDelay = 200 / portTICK_PERIOD_MS; // TO DO: magic number

    // PWMSignalQueue = xQueueCreate(8, sizeof(PWMSignal_t*)); TESTING

    while (true) 
    {
        checkSlip();
        vTaskDelay(xDelay);
    }   
}
