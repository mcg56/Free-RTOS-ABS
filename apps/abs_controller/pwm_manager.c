/**********************************************************
 *
 * pwm_manager.c - Main controlling file for the 
 *      PWM information.
 *
 * T.R Peterson, M.C Gardyne
 * Last modified:  24.7.22
 **********************************************************/

/**
 * This will contain:
 * 
 * 1. The PWM structs and current information
 * 2. The interrupt management for when to read information
 * 3. Any output stuff required for the output brake signal
 * 
 * This will be the point of contact with ap_pwm.c
 * 
 */

#include <stdint.h>
#include <stdbool.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "driverlib/timer.h"

#include "libs/lib_pwm/ap_pwm.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"

#include "pwm_manager.h"

//*************************************************************
// Type Definitions
//*************************************************************

/**
 * @brief Identifies the number of edges found in an update
 */
static enum PWMEdgesFound {NONE, ONE_EDGE, TWO_EDGES};

//*************************************************************
// Function handles
//*************************************************************
static void FLWheelIntHandler (void);
static void calculatePWMProperties (PWMSignal_t* PWMSignal);
static void updatePWMInfo(PWMSignal_t* PWMSignal);

//*************************************************************
// Global variables
//*************************************************************
static PWMInputSignals_t PWMInputSignals;
static enum PWMEdgesFound edgeCount;

/**
 * @brief Initialise the primary timer
 * @return None
 */
static void
initResponseTimer (void)
{
    // The Timer0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    TimerDisable(TIMER0_BASE, TIMER_A);

    // Configure Timer0B as a 16-bit periodic timer.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC_UP);

    // Set timer value
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());

    // Enable timer
    TimerEnable(TIMER0_BASE, TIMER_A);
}

/**
 * @brief Initialise the pins for the front left wheel
 * @return None
 */
static void
initFLWheelPins (void)
{
    // Enable port peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Set pin 0,1 and 4 as input
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);

    // Set what pin interrupt conditions
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);

    // Register interrupt
    GPIOIntRegister(GPIO_PORTB_BASE, FLWheelIntHandler);

    // Enable pins
    GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_0);
}

/**
 * @brief Initialise the PWM Manager module
 * @return None
 */
void
initPWMManager (void)
{
    initResponseTimer();
    initFLWheelPins();
}

/**
 * @brief Record the time stamps for each edge of a wheel PWM
 * @return None
 */
static void
FLWheelIntHandler (void)
{
    // Clean up, clearing the interrupt
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0);

    if (GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_0))
    {
        PWMInputSignals.FLWheel.lastRisingEdgeTS = PWMInputSignals.FLWheel.currRisingEdgeTS;
        PWMInputSignals.FLWheel.currRisingEdgeTS = TimerValueGet(TIMER0_BASE, TIMER_A);

        edgeCount++;
    }
    else
    {
        PWMInputSignals.FLWheel.currFallingEdgeTS = TimerValueGet(TIMER0_BASE, TIMER_A);
    }
}

/**
 * @brief Updates all PWM signal information
 * @return None
 */
void 
updateAllPWMInfo(void)
{
    updatePWMInfo(&PWMInputSignals.FLWheel);
}

/**
 * @brief Enable and update the duty and frequency of a given PWM signal
 * @param PWMSignal PWM signal to update
 * @return None
 */
static void 
updatePWMInfo(PWMSignal_t* PWMSignal)
{
    edgeCount = NONE;

    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0); // Need to generalise these. Didn't seem to work first try

    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0);
    while (edgeCount != TWO_EDGES);
    GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_0);

    calculatePWMProperties(PWMSignal);
}

/**
 * @brief Update the duty and frequency given a PWM signal's edge timestamps
 * @param PWMSgnal PWM signal to update
 * @return None
 */
static void
calculatePWMProperties(PWMSignal_t* PWMSignal)
{
    // Account for overflow of the timer
    if (PWMSignal->currRisingEdgeTS < PWMSignal->lastRisingEdgeTS)
    {
        PWMSignal->frequency = SysCtlClockGet() / (PWMSignal->currRisingEdgeTS 
            + SysCtlClockGet() - PWMSignal->lastRisingEdgeTS);
    }
    else
    {
        PWMSignal->frequency = SysCtlClockGet() / (PWMSignal->currRisingEdgeTS 
            - PWMSignal->lastRisingEdgeTS);
    }

    // Currently no timer overflow protection - NEED TO IMPLEMENT
    PWMSignal->duty = 100 * (PWMSignal->currFallingEdgeTS - PWMSignal->lastRisingEdgeTS) /
        (PWMSignal->currRisingEdgeTS - PWMSignal->lastRisingEdgeTS);
}

/**
 * @brief Returns the PWM Input signal list
 * @return Structure of PWM signals
 */
PWMInputSignals_t
getPWMInputSignals (void)
{
    return PWMInputSignals;
}