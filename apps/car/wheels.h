/** @file   wheels.h
    @author A.J Eason A. Musalov
    @date   22/08/22
    @brief  Header file for controlling wheel attributes. Has
            initialisation function handle and task handle.
*/

#ifndef WHEELS_H_
#define WHEELS_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "utils/ustdlib.h"
#include <FreeRTOS.h>
#include <task.h>

//*****************************************************************************
// Global variables
//*****************************************************************************

extern TaskHandle_t updateWheelInfoHandle;

//*************************************************************
// Function handles
//*************************************************************

/**
 * @brief Initialises the wheel module. Creates 
 * wheel update task.
 * @return None
 */
void initWheels(void);


#endif