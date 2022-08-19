/**********************************************************
car_state.c

Module which handles the getter and setter functions for the
car outputs. The module also handles the deceleration task.

A.J Eason A. Musalov
Last modified:  19/08/22
***********************************************************/

#include "car_state.h"
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/gpio.h>
#include <FreeRTOS.h>
#include <semphr.h>

//*************************************************************
// Private Constant Definitions
//*************************************************************

#define MILLISECOND_DIVIDER 1000.0
#define PERCENTAGE_DIVIDER 100.0
#define MAXIMUM_DECELERATION 15
#define DECELERATION_TASK_PERIOD 50
#define MIN_SPEED 0


//*****************************************************************************
// Global variables
//*****************************************************************************

TaskHandle_t decelerationTaskHandle;
SemaphoreHandle_t carStateMutex = NULL;
//*************************************************************
// Struct definitions
//*************************************************************

/**
 * @brief Private struture for storing car state
 * @param speed             Car speed
 * @param steeringWheelDuty Car steering wheel duty (%)
 * @param alpha Steering angle float
 * @param roadCondition         Road Condition
 * @param pedalState        Brake pedal toggle
 * @param brakePedalPressureDuty     Brake pedal pressure (%)
 * @param ABSBrakePressureDuty  Pressure returned from the ABS 
 * @param leftFront Left front wheel struct
 * @param leftRear Left rear wheel struct
 * @param rightFront Right front wheel struct
 * @param rightRear Right rear wheel struct
 * controller (should match brakePedalPressureDuty %) 
 */
typedef struct {
    float speed;
    uint8_t steeringWheelDuty;
    float alpha;
    Condition condition; 
    bool pedalState; 
    uint8_t brakePedalPressureDuty;
    uint8_t ABSBrakePressureDuty;
    bool ABSState;
    Wheel leftFront;
    Wheel leftRear;
    Wheel rightFront;
    Wheel rightRear;
} Car_t;

static const Wheel LF = {0, 0, 0, false};
static const Wheel LR = {0, 0, 0, false};
static const Wheel RF = {0, 0, 0, false};
static const Wheel RR = {0, 0, 0, false};

// Define local car state object
static Car_t carState = {.speed=50.0, .steeringWheelDuty=50, .alpha=0.0, .condition=0,
                         .pedalState=false, .brakePedalPressureDuty=50, .ABSBrakePressureDuty=5,
                         .ABSState=false, .leftFront=LF, .leftRear=LR, .rightFront=RF, .rightRear=RR};

//*************************************************************
// Private function prototypes
//*************************************************************

/**
 * @brief Task to update the speed of the car while braking.
 * deceleration is calculated by currentABSBrakeDuty*maxDecel*taskPeriodms/1000.0/100.0.
 * The task is started and stopped upon a brake press.
 * @param args Unused
 * @return No return
 */
void decelerationTask (void* args);

//*****************************************************************************
// Functions
//*****************************************************************************

void initCarState(void)
{
    // Create shared resource mutex
    carStateMutex = xSemaphoreCreateMutex();

    xTaskCreate(&decelerationTask, "decelerationTask", 256, NULL, 1, &decelerationTaskHandle);
    // Suspend deceleration task as brake initially off
    vTaskSuspend(decelerationTaskHandle);
}

//*****************************Getters***************************************
float getCarSpeed(void)
{
    return carState.speed;
}

uint8_t getSteeringDuty(void)
{
    return carState.steeringWheelDuty;
}

float getSteeringAngle(void)
{
    return carState.alpha;
}

Condition getRoadCondition(void)
{
    return carState.condition;
}

bool getPedalState(void)
{
    return carState.pedalState;
}

uint8_t getBrakePedalPressureDuty(void)
{
    return carState.brakePedalPressureDuty;
}

uint8_t getABSBrakePressureDuty(void)
{
    return carState.ABSBrakePressureDuty;
}

bool getABSState(void)
{
    return carState.ABSState;
}

Wheel getleftFront(void)
{
    return carState.leftFront;
}

Wheel getleftRear(void)
{
    return carState.leftRear;
}

Wheel getRightFront(void)
{
    return carState.rightFront;
}

Wheel getRightRear(void)
{
    return carState.rightRear;
}

//*****************************Setters***************************************
void setCarSpeed(float speed)
{
    carState.speed = speed;
}

void setSteeringDuty(uint8_t duty)
{
    carState.steeringWheelDuty = duty;
}

void setSteeringAngle(float angle)
{
    carState.alpha = angle;
}

void setRoadCondition(Condition condition)
{
    carState.condition = condition;
}

void setPedalState(bool state)
{
    carState.pedalState = state;
}

void setBrakePedalPressureDuty(uint8_t duty)
{
    carState.brakePedalPressureDuty = duty;
}

void setABSBrakePressureDuty(uint8_t duty)
{
    carState.ABSBrakePressureDuty = duty;
}

void setABSState(bool state)
{
    carState.ABSState = state;
}

void setLeftFront(Wheel wheel)
{
    carState.leftFront = wheel;
}

void setLeftRear(Wheel wheel)
{
    carState.leftRear = wheel;
}

void setRightFront(Wheel wheel)
{
    carState.rightFront = wheel;
}

void setRightRear(Wheel wheel)
{
    carState.rightRear = wheel;
}

void decelerationTask (void* args)
{
    (void)args;
    const float maxDecel = MAXIMUM_DECELERATION; // m/s^2
    const float taskPeriodms = DECELERATION_TASK_PERIOD; //ms
     
    
    while (true)
    {
        TickType_t wake_time = xTaskGetTickCount();
        
        // Wait until we can take the mutex to be able to use car state shared resource
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        // We have obtained the mutex, now can run the task

        //Obtain car speed
        float currentSpeed = getCarSpeed();
        //Obtain the brake pressure duty from ABS controller
        uint8_t currentABSBrakeDuty = getABSBrakePressureDuty();

        if (currentABSBrakeDuty != 0)
        {
            // Modify the speed dependant on brake pressure
            float newSpeed = currentSpeed - (float)currentABSBrakeDuty*maxDecel*taskPeriodms/MILLISECOND_DIVIDER/PERCENTAGE_DIVIDER;
            if (newSpeed <= MIN_SPEED) {
                    newSpeed = MIN_SPEED;
            }

            setCarSpeed(newSpeed);
        }
        
        

        // Tell the wheel update task to run
        xTaskNotifyGiveIndexed(updateWheelInfoHandle, 0);
        // Give the mutex back
        xSemaphoreGive(carStateMutex);
        vTaskDelayUntil(&wake_time, taskPeriodms);
    }   
}