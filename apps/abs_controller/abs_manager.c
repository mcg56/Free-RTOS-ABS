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
#include <stdio.h>
#include <queue.h>
#include "driverlib/timer.h"
#include "brake_output.h"


#define DIAMETER                0.5     // Wheel diameter (m)
#define HALF_DUTY               45      // Half of duty range (95-5)/2
#define MID_DUTY                50      // Mid of duty between 95-5
#define MAX_ANGLE               29.1    // Max absolute turn angle (deg)
#define TRACK                   1.5     // Wheel seperation (m)
#define TICKS_PER_REV           20      // Encoder pulses per single wheel revolution
#define TOLERANCE               10      // 10% velocity difference as per guide
#define FACTOR                  3.6     // m/s to km/h
#define SCALE_FACTOR            100     // Scale result to percentage
#define MIN_VELOCITY            10      // Minimum required velocity for ABS to function (m/s)

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
    // Radians conversion for math.h
    float alpha = mondeo.steeringAngle*(M_PI/180);

    // Initialise values to zero 
    float innerRear = 0;
    float innerFront = 0;
    float outerRear = 0;
    float outerFront = 0;

    // If alpha equals zero no conversion required
    uint32_t velocity = mondeo.wheelVel[wheel];

    if (alpha != 0) {
        // Radii calculations (Definition of turn radii from wheels.c may try combine later )
        innerRear = 2.5/tan(fabsf(alpha));
        innerFront = 2.5/sin(fabsf(alpha));
        outerRear = innerRear + TRACK;
        outerFront = innerFront + TRACK;

        // Initialise left hand turn orders array
        float normWheelOrder[NUM_WHEELS] = {innerRear, outerRear, innerFront, outerFront};
        float currentRadii = 0;
        
        if (alpha > 0) { 
            currentRadii = normWheelOrder[wheel];        
        } else {
            // If turning opposite way swap inner/outer pairs of turn array
            if ((wheel%2) == 0) { 
                currentRadii = normWheelOrder[wheel+1];
            } else {
                currentRadii = normWheelOrder[wheel-1];
            }
        }
        // Apply normalisation factor to standardise outer most wheel as vehicle velocity
        velocity = mondeo.wheelVel[wheel]*(outerFront/currentRadii);
    }
    return velocity;
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
float calcAngle(int32_t duty)
{   
    return (MID_DUTY-duty-1)*(MAX_ANGLE / HALF_DUTY);
}

/**
 * @brief Checks if vehicle is slipping or not
 * @param void
 * @return Boolean slip true/false
 */
void checkSlip(void)
{
    uint8_t state = ABS_OFF;
    mondeo.sold = false; // GIGITY GIGITY
    
    // Update steering angle
    mondeo.steeringAngle = calcAngle(getPWMInputSignal("Steering").duty);

    // Calculate individual wheel velocities
    // TO DO: confirm with mark what happens if some or all are zero
    mondeo.wheelVel[REAR_LEFT] = calcWheelVel(getPWMInputSignal("LR").frequency);
    mondeo.wheelVel[REAR_RIGHT] = calcWheelVel(getPWMInputSignal("RR").frequency);
    mondeo.wheelVel[FRONT_LEFT] = calcWheelVel(getPWMInputSignal("LF").frequency);
    mondeo.wheelVel[FRONT_RIGHT] = calcWheelVel(getPWMInputSignal("RF").frequency);

    // SPlit and task notify
    // Calculate hypothetical speed
    int32_t calcHypoVel[NUM_WHEELS];
    calcHypoVel[REAR_LEFT] = calcCarVel(REAR_LEFT);
    calcHypoVel[REAR_RIGHT] = calcCarVel(REAR_RIGHT);
    calcHypoVel[FRONT_LEFT] = calcCarVel(FRONT_LEFT);
    calcHypoVel[FRONT_RIGHT] = calcCarVel(FRONT_RIGHT);

    // Reset velocity for next computation
    mondeo.carVel = 0;

    // Loop through each hypotheticle car velocity and update car velocity to maximum value
    for (uint32_t i = 0; i < NUM_WHEELS; i++){
        if (calcHypoVel[i] > mondeo.carVel) {
            mondeo.carVel = calcHypoVel[i];
        }
    }

    // Loop through each hypotheticle car velocity and compare to maximum car velocity.
    // If absolute differnce greater than 10% and car velocity greater than 10 m/s turn ABS on
    for (uint32_t i = 0; i < NUM_WHEELS; i++){
        float diff = ((mondeo.carVel - calcHypoVel[i])*SCALE_FACTOR)/mondeo.carVel;
        if ((diff > TOLERANCE) && (mondeo.carVel > MIN_VELOCITY)) {
            state = ABS_ON;
            //TODO ABS state debouncing (make sure can activate within 0.5 sec)
        }
    }
   
    setABS(state);
}

/**
 * @brief Regularly scheduled task for updating all PWM signals
 * @return None
 */
void checkSlipTask(void* args)
{
    (void)args;
    const TickType_t xDelay = 400 / portTICK_PERIOD_MS; // TO DO: magic number

    // PWMSignalQueue = xQueueCreate(8, sizeof(PWMSignal_t*)); TESTING

    while (true) 
    {
        checkSlip();
        vTaskDelay(xDelay);
    }   
}

// TESTING
// char str[100];
// gcvt(diff, 3, str);
// sprintf(str, "diff %d\r\n\n", ((mondeo.carVel - calcHypoVel[i])/mondeo.carVel)*100);
// UARTSend(str); 
// sprintf(str, "\r\n\n");
// UARTSend(str)
