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
    float steeringAngle;
    int32_t carVel;
    bool sold;
    uint8_t absState;
    uint32_t brake;
} CarAttributes_t;


/**
 * @brief Regularly scheduled task for checking if the vehicle is slipping
 * @return None
 */
extern void 
initABSManager (void);

extern int 
getSteeringAngle (void);


#endif