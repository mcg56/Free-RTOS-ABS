#ifndef CAR_STATE_H_
#define CAR_STATE_H_

#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include "wheels.h"

/**
 * @brief Structure for storing information about car wheels
 * @param turnRadius The turn radius of the wheel (m)
 * @param speed The tangential velocity of the wheel (km/h)
 * @param pulseHz The pulse rate of wheel pwm encoder (Hz) 
 * @param slipping Boolean showing if this wheel is slipping or not
 */
typedef struct {
    float turnRadius; //m
    float speed; //km/h
    float pulseHz; //Hz
    bool slipping; 
} Wheel;

void initCarState(void);

float getCarSpeed(void);
uint8_t getSteeringDuty(void);
float getSteeringAngle(void);
uint8_t getRoadCondition(void);
bool getPedalState(void);
uint8_t getBrakePedalPressureDuty(void);
uint8_t getABSBrakePressureDuty(void);
bool getABSState(void);
Wheel getleftFront(void);
Wheel getleftRear(void);
Wheel getRightFront(void);
Wheel getRightRear(void);


void setCarSpeed(float speed);
void setSteeringDuty(uint8_t duty);
void setSteeringAngle(float angle);
void setRoadCondition(uint8_t condition);
void setPedalState(bool state);
void setBrakePedalPressureDuty(uint8_t duty);
void setABSBrakePressureDuty(uint8_t duty);
void setABSState(bool state);
void setLeftFront(Wheel wheel);
void setLeftRear(Wheel wheel);
void setRightFront(Wheel wheel);
void setRightRear(Wheel wheel);

void decelerationTask (void* args);

// Mutex to ensure only one task can access the shared resource car state struct at once (get/set etc)
extern SemaphoreHandle_t carStateMutex;

extern TaskHandle_t decelerationTaskHandle;

#endif