#ifndef ABS_MANAGER_H_
#define ABS_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>

#define NUM_WHEELS              4


//*************************************************************
// Type Definitions
//*************************************************************

/**
 * @brief Struture for storing car data
 * @param wheelVel          Individual wheel velocity
 * @param steeringAngle     Steering angle               
 * @param carVel            Car velocity (max of wheel signals)
 */
typedef struct {
    int16_t wheelVel[NUM_WHEELS];
    int16_t carVel;
    uint8_t absState;
    uint8_t brake;
    float steeringAngle;
} CarAttributes_t;

/**
 * @brief Enumerator for car wheels
 */
typedef enum {
    REAR_LEFT = 0, 
    REAR_RIGHT, 
    FRONT_LEFT, 
    FRONT_RIGHT
} Wheels;

/**
 * @brief Regularly scheduled task for checking if the vehicle is slipping
 * 
 * @return None
 */
extern void 
initABSManager (void);

/**
 * @brief Pass the car steering angle out of the module
 * 
 * @return int - Steering angle
 */
extern int 
getSteeringAngle (void);


#endif