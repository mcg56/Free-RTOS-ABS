#include "car_state.h"
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include <FreeRTOS.h>
#include <semphr.h>

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
    uint16_t speed;
    uint8_t steeringWheelDuty;
    float alpha;
    uint8_t roadCondition; 
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
static Car_t carState = {.speed=50, .steeringWheelDuty=50, .alpha=0.0, .roadCondition=0,
                         .pedalState=false, .brakePedalPressureDuty=50, .ABSBrakePressureDuty=5,
                         .ABSState=false, .leftFront=LF, .leftRear=LR, .rightFront=RF, .rightRear=RR};

SemaphoreHandle_t carStateMutex = NULL;

//*****************************Getters***************************************
uint16_t getCarSpeed(void)
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

uint8_t getRoadCondition(void)
{
    return carState.roadCondition;
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
void setCarSpeed(uint16_t speed)
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

void setRoadCondition(uint8_t condition)
{
    carState.roadCondition = condition;
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