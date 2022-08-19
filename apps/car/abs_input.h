/**********************************************************
abs_input.h

Header file for reading and processing the ABS input
signal from the ABS controller. Contains task handle for
processing ABS input task and an initialisation function
handle.

A.J Eason A. Musalov
Last modified:  18/08/22
***********************************************************/

#ifndef ABS_INPUT_H_
#define ABS_INPUT_H_

#include <FreeRTOS.h>
#include <task.h>

//*****************************************************************************
// Global variables
//*****************************************************************************
extern TaskHandle_t processABSInputSignalTaskHandle;

//*************************************************************
// Function handles
//*************************************************************

/**
 * @brief Initialises the abs_input module. Initialises
 * timer and creates tasks.
 * @return None
 */
void initABSInput(void);


#endif