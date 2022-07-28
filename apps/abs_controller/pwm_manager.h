#ifndef PWM_MANAGER_H_
#define PWM_MANAGER_H_

#include <stdint.h>

#define ID_LEN 20

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
extern void initPWMManager (void);
extern int trackPWMSignal (PWMSignal_t newSignal);
extern void updateAllPWMInfo (void);
extern PWMSignal_t getPWMInputSignals (char* id);

#endif