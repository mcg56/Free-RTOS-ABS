#include "car_state.h"
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"

/**
 * @brief Private struture for storing car state
 * @param speed             Car speed
 * @param steeringWheelDuty Car steering wheel duty (%)
 * @param roadCondition         Road Condition
 * @param pedalState        Brake pedal toggle
 * @param brakePedalPressureDuty     Brake pedal pressure (%)
 * @param ABSBrakePressureDuty  Pressure returned from the ABS 
 * controller (should match brakePedalPressureDuty %) 
 */
typedef struct {
    uint8_t speed; //m
    uint8_t steeringWheelDuty; //km/h
    uint8_t roadCondition; 
    bool pedalState; 
    uint8_t brakePedalPressureDuty;
    uint8_t ABSBrakePressureDuty;
} Car_t;

// Define local car state object
static Car_t carState = {50, 50, 0, 0, 50, 5};

// Mutex to ensure only one task can access car state struct at once (get/set etc)
//SemaphoreHandle_t carStateMutex = xSemaphoreCreateMutex();

// Getters
uint8_t getCarSpeed(void)
{
    return carState.speed;
}

uint8_t getSteeringDuty(void)
{
    return carState.steeringWheelDuty;
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


// Setters
void setCarSpeed(uint8_t speed)
{
    carState.speed = speed;
}

void setSteeringDuty(uint8_t duty)
{
    carState.steeringWheelDuty = duty;
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