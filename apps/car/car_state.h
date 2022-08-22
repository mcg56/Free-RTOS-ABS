/**********************************************************
car_state.h

Header file for controlling the getter and setter functions for
the car outputs. The module also handles the deceleration task.

A.J Eason A. Musalov
Last modified:  19/08/22
***********************************************************/
#ifndef CAR_STATE_H_
#define CAR_STATE_H_

#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include "wheels.h"

//*****************************************************************************
// Structures
//*****************************************************************************

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

//*****************************************************************************
// Enumerated data types
//*****************************************************************************

typedef enum {
    DRY,
    WET,
    ICY
} Condition;


//*************************************************************
// Function handles
//*************************************************************

/**
 * @brief Initialises the car state module. Creates 
 * deceleration task.
 * @return None
 */
void initCarState(void);

/**
 * @brief Get the speed of the car.
 * @return float - Car Speed km/h
 */
float getCarSpeed(void);
/**
 * @brief Get the steering wheel duty cycle.
 * @return uint8_t - Steering wheel duty
 */
uint8_t getSteeringDuty(void);
/**
 * @brief Get the angle of the sterring wheel.
 * @return float - Steering wheel angle
 */
float getSteeringAngle(void);
/**
 * @brief Get the condition of the road: DRY, WET, ICY.
 * @return Condition - Road condition
 */
Condition getRoadCondition(void);
/**
 * @brief Get the state of the brake pedal.
 * @return bool - Brake pedal state
 */
bool getPedalState(void);
/**
 * @brief Get the duty cycle from the brake pressure.
 * @return uint8_t - Brake pressure duty cycle
 */
uint8_t getBrakePedalPressureDuty(void);
/**
 * @brief Get the ABS brake pressure duty cycle output.
 * @return uint8_t - ABS brake pressure duty
 */
uint8_t getABSBrakePressureDuty(void);
/**
 * @brief Get the state of the ABS.
 * @return bool - State of ABS
 */
bool getABSState(void);
/**
 * @brief Get the information of the left front wheel
 * @return Wheel - Left front wheel info
 */
Wheel getleftFront(void);
/**
 * @brief Get the information of the left rear wheel
 * @return Wheel - Left rear wheel info
 */
Wheel getleftRear(void);
/**
 * @brief Get the information of the right front wheel
 * @return Wheel - Right front wheel info
 */
Wheel getRightFront(void);
/**
 * @brief Get the information of the right rear wheel
 * @return Wheel - right rear wheel info
 */
Wheel getRightRear(void);

/**
 * @brief Set the speed of the car
 * @param float - Speed km/h
 * @return None
 */
void setCarSpeed(float speed);
/**
 * @brief Sets the duty cycle of the car steering wheel
 * @param uint8_t - duty cycle %
 * @return None
 */
void setSteeringDuty(uint8_t duty);
/**
 * @brief Sets the steering wheel angle
 * @param float - Angle deg
 * @return None
 */
void setSteeringAngle(float angle);
/**
 * @brief Sets the condition of the road
 * @param Condition DRY, WET, ICY
 * @return None
 */
void setRoadCondition(Condition condition);
/**
 * @brief Sets the state of the brake pedal
 * @param bool brake state
 * @return None
 */
void setPedalState(bool state);
/**
 * @brief Sets the duty cycle of the brake pressure
 * @param uint8_t - Duty cycle %
 * @return None
 */
void setBrakePedalPressureDuty(uint8_t duty);
/**
 * @brief Sets the duty cycle of the ABS brake pressure
 * @param uint8_t - Duty cycle %
 * @return None
 */
void setABSBrakePressureDuty(uint8_t duty);
/**
 * @brief Sets the state of the ABS from the ABS controller
 * @param bool - ABS State
 * @return None
 */
void setABSState(bool state);
/**
 * @brief Sets the left front wheel information
 * @param Wheel - Wheel information
 * @return None
 */
void setLeftFront(Wheel wheel);
/**
 * @brief Sets the left rear wheel information
 * @param Wheel - Wheel information
 * @return None
 */
void setLeftRear(Wheel wheel);
/**
 * @brief Sets the right front wheel information
 * @param Wheel - Wheel information
 * @return None
 */
void setRightFront(Wheel wheel);
/**
 * @brief Sets the right rear wheel information
 * @param Wheel - Wheel information
 * @return None
 */
void setRightRear(Wheel wheel);


// Mutex to ensure only one task can access the shared resource car state struct at once (get/set etc)
extern SemaphoreHandle_t carStateMutex;
extern TaskHandle_t decelerationTaskHandle;


#endif