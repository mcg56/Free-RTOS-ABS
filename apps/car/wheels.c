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
#include "libs/lib_pwm/pwm_output.h"
#include "ui.h"

#define ALPHA_MAX 29.1
#define PULSES_PER_REV 20.0     // (#)
#define WHEEL_DIAMETER 0.5    // (m)
#define KPH_TO_MS_SCALE_FACTOR (1000.0/3600.0)
#define PI (3.141592653589793)


//**********************Local function prototypes******************************



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
 * @param wheel         wheel struct being detected
 * @param condition     Left rear wheel struct                
 * @param pressure      Right front wheel struct
 * @return Bool         Wheel slip condition
 */
bool detectWheelSlip(Wheel* wheel, uint8_t condition, uint8_t pressure);



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

//TO DO: Make it take only one wheel, and loop through calling it with 4 wheels outside? Like wheel slip func
void calculateWheelPwmFreq(Wheel* leftFront, Wheel* leftRear, Wheel* rightFront, Wheel* rightRear)
{
    leftFront->pulseHz = PULSES_PER_REV*leftFront->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(1.0*PI);
    leftRear->pulseHz = PULSES_PER_REV*leftRear->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(1.0*PI);
    rightFront->pulseHz = PULSES_PER_REV*rightFront->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(1.0*PI);
    rightRear->pulseHz = PULSES_PER_REV*rightRear->speed*KPH_TO_MS_SCALE_FACTOR/WHEEL_DIAMETER/(1.0*PI);
}

bool detectWheelSlip(Wheel* wheel, uint8_t condition, uint8_t pressure)
{
    const int8_t m = -1;
    int8_t c;
    uint8_t minspeed = 10;
    if (condition == 0) {
        c = 120;
    } else if (condition == 1){
        c = 100;
    } else if (condition == 2){
        c = 80;
    }

    if ((wheel->speed >= minspeed) && (pressure >= m*wheel->speed + c) ){
        wheel->speed = wheel->speed * 0.9 - 1.25*(pressure - (m*wheel->speed + c)); // Set wheel speed to a lower value dependant on how far over slip limit we are
        if(wheel->speed <= 0) wheel->speed = 0;
        wheel->slipping = true;
        return 1;
    } else {
        wheel->slipping = false;
        return 0;
    }
}


/**
 * @brief Task to update the wheel information and signal to PWM generators to update the frequencies
 * @param args Unused
 * @return No return
 */
void updateWheelInfoTask(void* args)
{
    (void)args; // unused
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
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

        float alpha = calculateSteeringAngle((float)steeringDuty);
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
        
        if (pedalState)
        {
            for (int i=0; i < 4; i++)
            {
                detectWheelSlip(wheelArray[i], roadCondition, brakePedalPressure);
            }
            // If all wheels slipping, need to artificially unslip one of them else the ABS wont be able to detect slip
            if (leftFront.slipping && rightFront.slipping &&  leftRear.slipping && rightRear.slipping)
            {
                leftFront.slipping = false;
                leftFront.speed = carSpeed;
            }
        } else // Not braking, reset slips
        {
            for (int i=0; i < 4; i++)
            {
                wheelArray[i]->slipping = false;
            }
        }
        
        calculateWheelPwmFreq(&leftFront, &leftRear, &rightFront, &rightRear);

        // Set wheel paramaters of car_state to the new values
        setLeftFront(leftFront);
        setLeftRear(leftRear);
        setRightFront(rightFront);
        setRightRear(rightRear);
        setSteeringAngle(alpha);

        // Wheel info updated, add to PWM queue to update signals to ABS controller

        /* Sending wheel pwms 1 at a time may cause issues as it updates them one at a time so abs
        controller might think its slipping whne it just hasnt updated all wheels yet*/
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

        vTaskDelay(xDelay);


    }
}