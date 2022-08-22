#ifndef PWM_INPUTS_H_
#define PWM_INPUTS_H_

/** @file   pwm_input.h
    @author T. Peterson, M. Gardyne
    @date   22/08/22
    @brief  Manages and updates the duty 
    and frequency of a set of input PWM signals.

    IMPORTANT - This assumes all signals are on port B or port C
*/

#include <stdint.h>
#include <stddef.h>

//*************************************************************
// Constant Definitions
//*************************************************************

#define ID_LEN                  20
#define ABS_PWM_ID              "ABS"
#define ABS_PWM_MIN_FREQ        30  // [Hz]
#define CAR_PWM_MIN_FREQ        500 // [Hz]

//*************************************************************
// Type Definitions
//*************************************************************
/**
 * @brief Information regarding any one PWM signal.
 * @param id - String identifier for the signal
 * @param gpioPort - GPIO port base the PWM signal is attached to
 * @param gpioPin - GPIO pin the PWM signal is attached to
 * @param duty - Duty cycle of the frequency [%]
 * @param frequency - Frequency of the signal [Hz]
 */
typedef struct {
    char id[ID_LEN];
    uint32_t gpioPort;
    uint32_t gpioPin;
    uint32_t duty;
    uint32_t frequency;
} PWMSignal_t;

//*************************************************************
// Function handles
//*************************************************************

/**
 * @brief Initialise the PWM input manager module
 * 
 * @param PWMMinFreq - Minimum PWM frequency to be read
 * @return None
 */
extern void 
initPWMInputManager (uint16_t PWMMinFreq);

/**
 * @brief Add a PWM signal to the list of registered input signals
 * 
 * @param newSignal - PWM signal to be added
 * @return Bool - true if successful, false if failed
 */
extern int 
registerPWMSignal (PWMSignal_t newSignal);

/**
 * @brief Updates specific PWM signal information
 * 
 * @return Count off failed PWM signal updates
 */
extern int 
updatePWMInput(char* id);

/**
 * @brief Sets the PWM minimum frequency for the module
 * 
 * @param PWMMinFreq - Minimum PWM frequency
 * @return None
 */
extern void 
setPWMMinFeq (uint16_t PWMMinFreq);

/**
 * @brief Returns the identified PWM Input signal
 * 
 * @param id String identifier for desired signal
 * @return Structure of PWM signal
 */
extern PWMSignal_t 
getPWMInputSignal (char* id);

/**
 * @brief Returns the number of PWM signals being tracked
 * 
 * @return int - Number of signals
 */
extern int 
getCountPWMInputs (void);

/**
 * @brief Provides the IDs of all tracking input signals
 * 
 * @param ids - An array to fill the IDs with
 * @param len - The length of the ids list
 */
extern void
getIDList (char* ids[], size_t len);

#endif