#ifndef CAR_STATE_H_
#define CAR_STATE_H_

#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include <FreeRTOS.h>
#include <semphr.h>

uint8_t getCarSpeed(void);
uint8_t getSteeringDuty(void);
uint8_t getRoadCondition(void);
bool getPedalState(void);
uint8_t getBrakePedalPressureDuty(void);
uint8_t getABSBrakePressureDuty(void);

void setCarSpeed(uint8_t speed);
void setSteeringDuty(uint8_t duty);
void setRoadCondition(uint8_t condition);
void setPedalState(bool state);
void setBrakePedalPressureDuty(uint8_t duty);
void setABSBrakePressureDuty(uint8_t duty);

// Mutex to ensure only one task can access car state struct at once (get/set etc)
extern SemaphoreHandle_t carStateMutex;

#endif