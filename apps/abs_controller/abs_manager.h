#ifndef ABS_MANAGER_H_
#define ABS_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>


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
    uint32_t wheelVel[4];
    uint32_t steeringAngle;
    uint32_t carVel;
} CarAttributes_t;

//*************************************************************
// Function handles
//*************************************************************


// extern void checkSlippySloppy(PWMSignal_t inputSignals);
extern bool checkSlippySloppy(void);

#endif