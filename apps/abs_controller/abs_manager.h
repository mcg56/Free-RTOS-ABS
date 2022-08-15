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
    int32_t wheelVel[NUM_WHEELS];
    int32_t carVel;
    uint8_t absState;
    uint32_t brake;
    float steeringAngle;
} CarAttributes_t;

typedef enum {
    REAR_LEFT = 0, 
    REAR_RIGHT, 
    FRONT_LEFT, 
    FRONT_RIGHT
} Wheels;


/**
 * @brief Regularly scheduled task for checking if the vehicle is slipping
 * @return None
 */
extern void 
initABSManager (void);

extern int 
getSteeringAngle (void);


#endif