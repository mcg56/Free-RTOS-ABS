#ifndef BRAKE_OUTPUT_H_
#define BRAKE_OUTPUT_H_

/**********************************************************
 *
 * brake_output.h - Manages brake output signal
 * 
 *
 * T.R Peterson, M.C Gardyne
 * Last modified:  19.8.22
 **********************************************************/

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
void
initBrakeOutput (void);

/**
 * @brief External function to set ABS
 * @return None
 * 
 */
void
setABS (enum absStates state);

/**
 * @brief External access to toggle the ABS
 * 
 * @return None
 */
void
toggleABS (void);

/**
 * @brief Sets the duty cycle of the ABS signal
 * 
 * @param duty - Percentage duty cycle
 * @return int - 1 if successful, 0 if failed
 */
int 
setABSDuty (uint8_t duty);

/**
 * @brief Passes the ABS state out of the module
 * 
 * @return enum absStates - Current ABS state
 */
enum absStates 
getABSState (void);

/**
 * @brief Passes the ABS duty out of the module
 * 
 * @return int ABSDuty - The current ABS duty
 */
uint8_t 
getABSDuty (void);


#endif