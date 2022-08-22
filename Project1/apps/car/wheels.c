/** @file   wheels.c
    @author A.J Eason A. Musalov
    @date   22/08/22
    @brief  Module for controlling wheel attributes. Functions and tasks
            for calculating steering angle, wheel radii, wheel speed, slip
            etc.
*/

#include "wheels.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_memmap.h>
#include <driverlib/gpio.h>
#include "stdlib.h"
#include "utils/ustdlib.h"
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "car_state.h"
#include "car_pwm.h"
#include "utils/ustdlib.h"
#include "Project1/libs/lib_pwm/pwm_output.h"
#include "user_interface.h"

//*************************************************************
// Private Constant Definitions
//*************************************************************
#define ALPHA_MAX               29.1 // degrees
#define PULSES_PER_REV          20.0 // [#]
#define WHEEL_DIAMETER          0.5  // [m]
#define KPH_TO_MS_SCALE_FACTOR  (1000.0/3600.0)
#define PI                      (3.141592653589793)
#define F_50_DUTY               50.0 // [%]
#define F_45_DUTY               45.0 // [%]
#define F_180_DEG               180.0 // Degrees
#define CAR_LEN                 2.5 // [m]
#define CAR_WHEEL_SEPERATION    1.5 // [m]
#define MIN_SLIP_FRACTION       0.9 // 90%
#define SLIP_SCALE_FACTOR       1.25
#define DRY_Y_INTERCEPT         120 // [km/h]
#define WET_Y_INTERCEPT         100 // [km/h]
#define ICY_Y_INTERCEPT         80 // [km/h]
#define CAR_MIN_SLIP_SPEED      10 // [km/h]
#define NUM_WHEELS              4
#define UPDATE_WHEELS_TASK_PRIORITY 2
#define UPDATE_WHEELS_TASK_STACK_SIZE 256


//*****************************************************************************
// Global variables
//*****************************************************************************

TaskHandle_t updateWheelInfoHandle;



//*************************************************************
// Private function prototypes
//*************************************************************

/**
 * @brief Task to update the wheel information and signal to 
 * PWM generators to update the frequencies.
 * @param args Unused
 * @return No return
 */
void updateWheelInfoTask(void* args);


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
 * @brief function to calculate the turn radius of each wheel based on angle and speed.
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
 * @param wheel         wheel struct being detected
 * @param condition     Left rear wheel struct                
 * @param pressure      Right front wheel struct
 * @return Bool         Wheel slip condition
 */
bool detectWheelSlip(Wheel* wheel, uint8_t condition, uint8_t pressure);

//*****************************************************************************
// Functions
//*****************************************************************************

void initWheels(void)
{
    xTaskCreate(&updateWheelInfoTask, "update wheel info", UPDATE_WHEELS_TASK_STACK_SIZE, NULL, UPDATE_WHEELS_TASK_PRIORITY, &updateWheelInfoHandle);
    
    // Tell the wheel update task to run, which fills out the wheels speeds with starting info
    xTaskNotifyGiveIndexed(updateWheelInfoHandle, 0);
}

float calculateSteeringAngle(float steeringWheelDuty)
{
    // Formula and constants from project specifications guide
    return ALPHA_MAX*((steeringWheelDuty - F_50_DUTY)/F_45_DUTY);
}


void calculateWheelRadii(Wheel* innerRear, Wheel* innerFront, Wheel* outerRear, Wheel* outerFront, float alpha)
{
    // Formula and constants from project specifications guide
    innerRear->turnRadius = CAR_LEN/tan(fabs(alpha*PI/F_180_DEG));
    innerFront->turnRadius = CAR_LEN/sin(fabs(alpha*PI/F_180_DEG));
    outerRear->turnRadius = innerRear->turnRadius + CAR_WHEEL_SEPERATION;
    outerFront->turnRadius = innerFront->turnRadius + CAR_WHEEL_SEPERATION;
}


void calculateWheelSpeedsFromRadii(Wheel* leftFront, Wheel* leftRear, Wheel* rightFront, Wheel* rightRear, float carSpeed)
{
    // Find centre radius and assume this is travelling at cars true speed
    float centerRadius = (leftRear->turnRadius + rightRear->turnRadius)/2;
    // Use this to find cars angular velocity
    float angularSpeed = carSpeed/centerRadius;

    // Set relative wheel speed based on turn radius
    leftFront->speed = angularSpeed*leftFront->turnRadius;
    leftRear->speed = angularSpeed*leftRear->turnRadius;
    rightFront->speed = angularSpeed*rightFront->turnRadius;
    rightRear->speed = angularSpeed*rightRear->turnRadius;
}

void calculateWheelPwmFreq(Wheel* leftFront, Wheel* leftRear, Wheel* rightFront, Wheel* rightRear)
{
    // Formula and constants from project specifications guide
    leftFront->pulseHz = PULSES_PER_REV*leftFront->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(PI);
    leftRear->pulseHz = PULSES_PER_REV*leftRear->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(PI);
    rightFront->pulseHz = PULSES_PER_REV*rightFront->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(PI);
    rightRear->pulseHz = PULSES_PER_REV*rightRear->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(PI);
}

bool detectWheelSlip(Wheel* wheel, Condition condition, uint8_t pressure)
{
    // Formula and constants from simple car/abs model description document
    const int8_t m = -1; // Slope of slipping curves
    int8_t c; // Y intercept of slipping curve

    // Set y intercepts of slipping curve based on road condition
    // Lower y intercept means car slips at less speed+braking
    if (condition == DRY) {
        c = DRY_Y_INTERCEPT; // [km/h]
    } else if (condition == WET){
        c = WET_Y_INTERCEPT; // [km/h]
    } else if (condition == ICY){
        c = ICY_Y_INTERCEPT; // [km/h]
    }

    // Check if the car is travelling above the minimum speed and the brake pressure is above the slip limit
    if ((wheel->speed >= CAR_MIN_SLIP_SPEED) && (pressure >= m*wheel->speed + c) ){
        // Set wheel speed to a lower value dependant on how far over slip limit we are
        wheel->speed = wheel->speed * MIN_SLIP_FRACTION - SLIP_SCALE_FACTOR*(pressure - (m*wheel->speed + c)); 
        // If above equaiton sets a negative wheel speed, set it to 0 (full lock)
        if(wheel->speed <= 0) wheel->speed = 0;
        wheel->slipping = true;
        return 1;
    } else {
        // If not, wheel is not slipping and speed stays the same
        wheel->slipping = false;
        return 0;
    }
}


void updateWheelInfoTask(void* args)
{
    (void)args; // unused
    while(true) {
        // Wait until a task has notified it to run, when the car state has changed
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

        // Wait until we can take the mutex to be able to use car state shared resource
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        // We have obtained the mutex, now can run the task

        // Get the car state info
        float carSpeed = getCarSpeed();
        uint8_t steeringDuty = getSteeringDuty();
        uint8_t roadCondition = getRoadCondition();
        bool pedalState = getPedalState();
        uint8_t brakePedalPressure = getBrakePedalPressureDuty();
        Wheel leftFront = getleftFront();
        Wheel leftRear = getleftRear();
        Wheel rightFront = getRightFront();
        Wheel rightRear = getRightRear();
        Wheel* wheelArray[4] = {&leftFront, &leftRear, &rightFront, &rightRear};

        // First calculate steering angle from duty
        float alpha = calculateSteeringAngle((float)steeringDuty);

        // Then set car wheel radii and speed based on steering angle
        if (alpha == 0) // Driving straight
        {
            leftFront.speed = carSpeed;
            leftFront.turnRadius = 0;

            leftRear.speed = carSpeed;
            leftRear.turnRadius = 0;

            rightFront.speed = carSpeed;
            rightFront.turnRadius = 0;

            rightRear.speed = carSpeed;
            rightRear.turnRadius = 0;
            
        } 
        else if (alpha < 0.0) //Turning left
        {
            calculateWheelRadii(&leftRear, &leftFront, &rightRear, &rightFront, alpha);
            calculateWheelSpeedsFromRadii(&leftFront, &leftRear, &rightFront, &rightRear, carSpeed);
        }
        else if (alpha > 0.0) //Turning right
        {
            calculateWheelRadii(&rightRear, &rightFront, &leftRear, &leftFront, alpha);
            calculateWheelSpeedsFromRadii(&leftFront, &leftRear, &rightFront, &rightRear, carSpeed);
        }

        // If brake is pressed, need to check for wheel slip
        if (pedalState)
        {
            // Loop through wheel array and check for slip
            for (int i=0; i < NUM_WHEELS; i++)
            {
                detectWheelSlip(wheelArray[i], roadCondition, brakePedalPressure);
            }
            //If all wheels slipping, need to artificially unslip one of them else the ABS wont be able to detect slip
            if (leftFront.slipping && rightFront.slipping &&  leftRear.slipping && rightRear.slipping)
            {
                leftFront.speed = carSpeed;
            }
        } else // Not braking, reset slips
        {
            for (int i=0; i < NUM_WHEELS; i++)
            {
                wheelArray[i]->slipping = false;
            }
        }
        
        // Calculate and set wheel encoder Hz (PRR)
        calculateWheelPwmFreq(&leftFront, &leftRear, &rightFront, &rightRear);

        // Set wheel paramaters of car_state to the new values
        setLeftFront(leftFront);
        setLeftRear(leftRear);
        setRightFront(rightFront);
        setRightRear(rightRear);
        setSteeringAngle(alpha);

        // Wheel info updated, add to PWM queue to update signals to ABS controller

        pwmOutputUpdate_t leftFrontPWM = {PWM_WHEEL_FIXED_DUTY, (uint32_t)leftFront.pulseHz, PWMHardwareDetailsLF};
        xQueueSendToBack(updatePWMQueue, &leftFrontPWM, 0);

        pwmOutputUpdate_t leftRearPWM = {PWM_WHEEL_FIXED_DUTY, (uint32_t)leftRear.pulseHz, PWMHardwareDetailsLR};
        xQueueSendToBack(updatePWMQueue, &leftRearPWM, 0);

        pwmOutputUpdate_t rightFrontPWM = {PWM_WHEEL_FIXED_DUTY, (uint32_t)rightFront.pulseHz, PWMHardwareDetailsRF};
        xQueueSendToBack(updatePWMQueue, &rightFrontPWM, 0);

        pwmOutputUpdate_t rightRearPWM = {PWM_WHEEL_FIXED_DUTY, (uint32_t)rightRear.pulseHz, PWMHardwareDetailsRR};
        xQueueSendToBack(updatePWMQueue, &rightRearPWM, 0);

        // Give the mutex back
        xSemaphoreGive(carStateMutex);
    }
}