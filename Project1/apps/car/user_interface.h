/** @file   user_interface.h
    @author A.J Eason A. Musalov
    @date   22/08/22
    @brief  Header file for controlling the UART USB user interface. Has
            initialisation function handle and task handle.
*/

#ifndef UI_H_
#define UI_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "utils/ustdlib.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "wheels.h"
#include "car_state.h"

//*****************************************************************************
// Global variables
//*****************************************************************************

extern TaskHandle_t processUserInputsTaskHandle;
extern TaskHandle_t updateUARTTaskHandle;

//*************************************************************
// Function handles
//*************************************************************


/**
 * @brief Prints the titles of  the car information to the UART UI.
 * @return None
 */
void vt100_print_text(void);

/**
 * @brief Prints the car turning radius to the UART UI.
 * @return None
 */
void vt100_print_radii(char LF[6],char LR[6],char RF[6],char RR[6]);

/**
 * @brief Prints the wheel frequency to the UART UI.
 * @return None
 */
void vt100_print_prr(char LF[6],char LR[6],char RF[6],char RR[6]);

/**
 * @brief Prints the road condition to the UART UI.
 * @return None
 */
void vt100_print_condition(Condition condition);

/**
 * @brief Initialises the UART user interdace. Initialises OLED, buttons,
 * and USB UART. Also creates the user input and UART 
 * task. 
 * @return None
 */
void initUserInterface(void);


#endif

