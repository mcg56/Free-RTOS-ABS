#ifndef PWM_INPUTS_H_
#define PWM_INPUTS_H_

#include <stdint.h>

#define ID_LEN 20
#define ABSPWM_ID "ABS"
#define ABS_TIMEOUT_RATE        18 // [Hz]
#define CAR_TIMEOUT_RATE        50 // [Hz]

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
 * @return None
 */
extern void initPWMInputManager (uint8_t timeoutRate);

/**
 * @brief Add a PWM signal to the list of registered input signals
 * @param newSignal - PWM signal to be added
 * @return Bool - true if successful, false if failed
 */
extern int registerPWMSignal (PWMSignal_t newSignal);

/**
 * @brief Regularly scheduled task for updating all PWM signals
 * @return None
 */
extern void updateAllPWMInputsTask(void* args);

/**
 * @brief Updates specific PWM signal information
 * @return Count off failed PWM signal updates
 */
extern int updatePWMInput(char* id, uint8_t timeoutRate);

/**
 * @brief Tasks for managing the PWM signal update queue
 * @param args Task arguments
 */
extern void calculatePWMPropertiesTask(void* args);

/**
 * @brief Returns the identified PWM Input signal
 * @param id String identifier for desired signal
 * @return Structure of PWM signal
 */
extern PWMSignal_t getPWMInputSignal (char* id);

#endif