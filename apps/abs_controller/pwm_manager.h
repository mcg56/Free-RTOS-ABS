#ifndef PWM_MANAGER_H_
#define PWM_MANAGER_H_

#include <stdint.h>

//*************************************************************
// Type Definitions
//*************************************************************
/**
 * @brief Information regarding any one PWM signal.
 * @param gpioPort - GPIO port base the PWM signal is attached to
 * @param gpioPin - GPIO pin the PWM signal is attached to
 * @param duty - Duty cycle of the frequency [%]
 * @param frequency - Frequency of the signal [Hz]
 * @param currRisingEdgeTS - Timestamp of the most recent rising edge
 * @param lastRisingEdgeTS - Timestamp of the previous rising edge
 * @param currFallingEdgeTS - Timestamp of the most recent falling edge
 */
typedef struct {
    uint32_t gpioPort;
    uint32_t gpioPin;
    uint32_t duty;
    uint32_t frequency;
    uint32_t currRisingEdgeTS;
    uint32_t lastRisingEdgeTS;
    uint32_t currFallingEdgeTS;
} PWMSignal_t;

/**
 * @brief Collection of all input PWM signals
 * @param FLWheel - Front left wheel PWM
 */
typedef struct {
    PWMSignal_t FLWheel;
} PWMInputSignals_t;


//*************************************************************
// Function handles
//*************************************************************
extern void initPWMManager (void);
extern void updateAllPWMInfo (void);
extern PWMInputSignals_t getPWMInputSignals (void);

#endif