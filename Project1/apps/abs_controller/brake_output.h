#ifndef BRAKE_OUTPUT_H_
#define BRAKE_OUTPUT_H_

/** @file   brake_output.h
    @author T. Peterson, M. Gardyne
    @date   22/08/22
    @brief  Controls the output brake signal to car simulator
*/

//*************************************************************
// Type Definitions
//*************************************************************

/**
 * @brief ABS states
 */
enum absStates {ABS_OFF = 0, ABS_ON};

//*************************************************************
// Function handles
//*************************************************************

/**
 * @brief Initialise brake output module
 * 
 * @return None
 */
extern void
initBrakeOutput (void);

/**
 * @brief External function to set ABS
 * @return None
 * 
 */
extern void
setABS (enum absStates state);

/**
 * @brief External access to toggle the ABS
 * 
 * @return None
 */
extern void
toggleABS (void);

/**
 * @brief Sets the duty cycle of the ABS signal
 * 
 * @param duty - Percentage duty cycle
 * @return int - 1 if successful, 0 if failed
 */
extern int 
setABSDuty (uint8_t duty);

/**
 * @brief Passes the ABS state out of the module
 * 
 * @return enum absStates - Current ABS state
 */
extern enum absStates 
getABSState (void);

/**
 * @brief Passes the ABS duty out of the module
 * 
 * @return int ABSDuty - The current ABS duty
 */
extern uint8_t 
getABSDuty (void);


#endif