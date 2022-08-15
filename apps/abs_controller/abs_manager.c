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
#include "pwm_info.h"


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
#define NUM_ABS_POLLS           2       //
#define MIN_BRAKE_DUTY          5       //

#define UPDATE_CAR_TASK_RATE    50     // [ms]

//*************************************************************
// Function prototype
//*************************************************************
void checkVelTask(void* args);
void updateCarTask(void* args);
void updateCar(void);
void checkVelTask(void* args);
int getSteeringAngle (void);
uint32_t calcCarVel(uint32_t wheel);
uint32_t calcWheelVel(uint32_t frequency);
float calcAngle(int32_t duty);


CarAttributes_t mondeo; // Probably not very reliable
TaskHandle_t checkVelHandle;
TaskHandle_t updateCarHandle;

typedef enum {
    REAR_LEFT = 0, 
    REAR_RIGHT, 
    FRONT_LEFT, 
    FRONT_RIGHT
} Wheels;


void
initABSManager (void)
{
    mondeo.absState = ABS_OFF;
    xTaskCreate(&updateCarTask, "updateCar", 256, NULL, 0, &updateCarHandle);
    xTaskCreate(&checkVelTask, "checkVel", 256, NULL, 0, &checkVelHandle);
}

int 
getSteeringAngle (void)
{
    return mondeo.steeringAngle;
}


/**
 * @brief Calculates the hypothetical vehicle velocity based on outer-most wheel from individual wheel velocity
 * @param - Target wheel
 * @return - Hypothetical vehicle velocity
 */
uint32_t 
calcCarVel(uint32_t wheel)
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
uint32_t 
calcWheelVel(uint32_t frequency)
{
    return (FACTOR*DIAMETER*M_PI*frequency)/(TICKS_PER_REV);
}

/**
 * @brief Calculates the turn angle based off input PWM signal
 * @param duty Duty cycle of steering input
 * @return Float (steering angle)
 */
float 
calcAngle(int32_t duty)
{   
    return (MID_DUTY-duty-1)*(MAX_ANGLE / HALF_DUTY);
}

/**
 * @brief Updates car attributes
 * @param 
 * @return 
 */
void 
updateCar(void)
{   

    // GIGITY GIGITY
    mondeo.sold = false; 

    // Update steering angle
    mondeo.steeringAngle = calcAngle(getPWMInputSignal(STEERING_ID).duty);
    mondeo.brake = getPWMInputSignal(BRAKE_PEDAL_ID).duty;


    // Calculate individual wheel velocities
    mondeo.wheelVel[REAR_LEFT] = calcWheelVel(getPWMInputSignal(RL_WHEEL_ID).frequency);
    mondeo.wheelVel[REAR_RIGHT] = calcWheelVel(getPWMInputSignal(RR_WHEEL_ID).frequency);
    mondeo.wheelVel[FRONT_LEFT] = calcWheelVel(getPWMInputSignal(FL_WHEEL_ID).frequency);
    mondeo.wheelVel[FRONT_RIGHT] = calcWheelVel(getPWMInputSignal(FR_WHEEL_ID).frequency);
    // Tell checkVel to compare speeds
    xTaskNotifyGiveIndexed( checkVelHandle, 0 );
    
}

/**
 * @brief Updates car attributes
 * @param void
 * @return 
 */
void 
checkVelTask(void* args)
{   
    (void)args; 

    while (true)
    {
        // Wait until a task has notified it to run
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        
        static uint8_t absCount = 0;
        uint8_t absValue = ABS_OFF;
        

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
        // If absolute differnce greater than 10% and car velocity greater than 10 m/s turn ABS on and if the brakes are on
        for (uint32_t i = 0; i < NUM_WHEELS; i++){
            float diff = ((mondeo.carVel - calcHypoVel[i])*SCALE_FACTOR)/mondeo.carVel;
            if ((diff > TOLERANCE) && (mondeo.carVel > MIN_VELOCITY) && (mondeo.brake > MIN_BRAKE_DUTY)) {
                absValue = ABS_ON;
            } 
        }

        if (absValue != mondeo.absState)
        {
        	absCount++;
        	if (absCount >= NUM_ABS_POLLS)
        	{
        		mondeo.absState = absValue;
        		absCount = 0;
        	}
        }
        else
        	absCount = 0;


        setABS(mondeo.absState);
        setABSDuty (mondeo.brake);
        // char str[100];
        // sprintf(str, "Abs state %d\r\n\n", mondeo.absState);
        // UARTSend(str); 

    }
}


/**
 * @brief Regularly scheduled task for updating all PWM signals
 * @return None
 */
void 
updateCarTask(void* args)
{
    (void)args;
    const TickType_t xDelay = UPDATE_CAR_TASK_RATE / portTICK_PERIOD_MS; // TO DO: magic number
    while (true) 
    {
        updateCar();
        vTaskDelay(xDelay);
    }   
}


