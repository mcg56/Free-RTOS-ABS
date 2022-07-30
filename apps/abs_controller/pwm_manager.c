/**********************************************************
 *
 * pwm_manager.c - Main controlling file for the 
 *      PWM information.
 *
 * IMPORTANT - This assumes all signals are on port B
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
#include <string.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/watchdog.h>

#include <FreeRTOS.h>
#include <task.h>

#include "driverlib/timer.h"

#include "libs/lib_pwm/ap_pwm.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"

#include "pwm_manager.h"

#define LEN(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))
#define MAX_NUM_SIGNALS 6
#define TIMEOUT_RATE 20 // Hz

// TASK - change all the timer stuff to defines

// TASK - add a function to read a specific PWM

//*************************************************************
// Type Definitions
//*************************************************************

/**
 * @brief Identifies the number of edges found in an update
 */
static enum PWMEdgesFound {NONE, ONE_EDGE, TWO_EDGES};

/**
 * @brief Collection of all input PWM signals
 * @param signals - List of PWM signals
 * @param count - Count of PWM input signal
 * @param pins - Bitmask of the input signal pins
 */
typedef struct {
    PWMSignal_t signals[MAX_NUM_SIGNALS];
    int count;
    uint32_t pins;
} PWMInputSignals_t;

/**
 * @brief Record of edge timestamps for PWM calculations
 * @param currRisingEdge - Timestamp of the most recent rising edge
 * @param lastRisingEdge - Timestamp of the previous rising edge
 * @param currFallingEdge - Timestamp of the most recent falling edge
 */
typedef struct {
    uint32_t currRisingEdge;
    uint32_t lastRisingEdge;
    uint32_t currFallingEdge;
} edgeTimestamps_t;

//*************************************************************
// Function handles
//*************************************************************
static void PWMIntHandler (void);
static void PWMTimeoutHandler (void);
static void calculatePWMProperties (PWMSignal_t* PWMSignal, edgeTimestamps_t* edgeTimestamps);
static bool updatePWMInfo(PWMSignal_t* PWMSignal);

//*************************************************************
// Global variables
//*************************************************************
static PWMInputSignals_t PWMInputSignals;
static volatile enum PWMEdgesFound risingEdgeCount;
static volatile edgeTimestamps_t edgeTimestamps;
static volatile bool PWMReadTimeout = false;

/**
 * @brief Initialise the primary timer
 * @return None
 */
static void
initTimer (void)
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
 * @brief Initialise the pins of a given inputSignal
 * @param inputSignal - PWM signal to be initialised
 * @return None
 */
static void
initPWMInput (PWMSignal_t inputSignal)
{
    // Enable port peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Set pin 0,1 and 4 as input
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, inputSignal.gpioPin);

    // Set what pin interrupt conditions
    GPIOIntTypeSet(GPIO_PORTB_BASE, inputSignal.gpioPin, GPIO_BOTH_EDGES);

    // Register interrupt
    GPIOIntRegister(GPIO_PORTB_BASE, PWMIntHandler);

    // Enable pins
    GPIOIntDisable(GPIO_PORTB_BASE, inputSignal.gpioPin);
}

/**
 * @brief Timer for reading PWM edges timeout
 * 
 */
static void
initPWMTimeoutTimer (void)
{
    // The Timer0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    TimerDisable(TIMER1_BASE, TIMER_A);

    // Configure Timer0B as a 16-bit periodic timer.
    TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC);

    // Register interrupt handler
    TimerIntRegister(TIMER1_BASE, TIMER_A, PWMTimeoutHandler);

    // Set prescaler
    TimerPrescaleSet(TIMER1_BASE, TIMER_A, 1);

    // Set timer value
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / TIMEOUT_RATE);

    // Enable interrupt
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Enable timer
    TimerDisable(TIMER1_BASE, TIMER_A);
}

/**
 * @brief Initialise the PWM Manager module
 * @return None
 */
void
initPWMManager (void)
{
    initTimer();
    initPWMTimeoutTimer();
}

/**
 * @brief Add a PWM signal to the list of tracked input signals
 * @param newSignal - PWM signal to be added
 * @return Bool - true if successful, false if failed
 */
int
trackPWMSignal (PWMSignal_t newSignal)
{
    if (PWMInputSignals.count == MAX_NUM_SIGNALS)
    {
        return false;
    }
    
    initPWMInput(newSignal);

    PWMInputSignals.signals[PWMInputSignals.count++] = newSignal;
    PWMInputSignals.pins |= newSignal.gpioPin;

    return true;
}

/**
 * @brief Record the time stamps for each edge of a wheel PWM
 * @return None
 */
static void
PWMIntHandler (void)
{
    if (GPIOPinRead (GPIO_PORTB_BASE, GPIOIntStatus(GPIO_PORTB_BASE, true)))
    {
        edgeTimestamps.lastRisingEdge = edgeTimestamps.currRisingEdge;
        edgeTimestamps.currRisingEdge = TimerValueGet(TIMER0_BASE, TIMER_A);

        risingEdgeCount++;
    }
    else
    {
        edgeTimestamps.currFallingEdge = TimerValueGet(TIMER0_BASE, TIMER_A);
    }

    // Clean up, clearing the interrupt
    GPIOIntClear(GPIO_PORTB_BASE, PWMInputSignals.pins);
}

/**
 * @brief PWM read watchdog timeout handler
 * 
 */
static void
PWMTimeoutHandler (void)
{
    TimerIntClear(TIMER1_BASE, TIMER_A);

    PWMReadTimeout = true;
}

/**
 * @brief Reset timeout
 * 
 */
static void
resetTimeout (void)
{
    TimerIntClear(TIMER0_BASE, TIMER_A);
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / TIMEOUT_RATE);

    PWMReadTimeout = false;
}

/**
 * @brief Updates all PWM signal information
 * @return None
 */
void 
updateAllPWMInfo(void)
{
    for (int i = 0; i < PWMInputSignals.count; i++)
    {
        updatePWMInfo(&PWMInputSignals.signals[i]);
    }   
}

/**
 * @brief Enable and update the duty and frequency of a given PWM signal
 * @param PWMSignal - PWM signal to update
 * @return Bool - Boolean representing whether the update was successful
 */
static bool 
updatePWMInfo(PWMSignal_t* PWMSignal)
{
    risingEdgeCount = NONE; // May need to be specific to PWM signal - not sure yet

    GPIOIntClear(GPIO_PORTB_BASE, PWMInputSignals.pins);
    resetTimeout();

    TimerEnable(TIMER1_BASE, TIMER_A);
    GPIOIntEnable(GPIO_PORTB_BASE, PWMSignal->gpioPin);
    while (risingEdgeCount != TWO_EDGES && !PWMReadTimeout);
    GPIOIntDisable(GPIO_PORTB_BASE, PWMSignal->gpioPin);
    TimerDisable(TIMER1_BASE, TIMER_A);

    if (!PWMReadTimeout)
    {
        calculatePWMProperties(PWMSignal, &edgeTimestamps);

        return true;
    }

    return false;
}

/**
 * @brief Update the duty and frequency given a PWM signal's edge timestamps
 * @param PWMSgnal - PWM signal to update
 * @param edgeTimestamps - Timestamps of the required edge points 
 * @return None
 */
static void
calculatePWMProperties(PWMSignal_t* PWMSignal, edgeTimestamps_t* edgeTimestamps)
{
    // Account for overflow of the timer
    if (edgeTimestamps->currRisingEdge < edgeTimestamps->lastRisingEdge)
    {
        PWMSignal->frequency = SysCtlClockGet() / (edgeTimestamps->currRisingEdge
            + SysCtlClockGet() - edgeTimestamps->lastRisingEdge);
    }
    else
    {
        PWMSignal->frequency = SysCtlClockGet() / (edgeTimestamps->currRisingEdge
            - edgeTimestamps->lastRisingEdge);
    }

    // Currently no timer overflow protection - TASK
    PWMSignal->duty = 100 * (edgeTimestamps->currFallingEdge - edgeTimestamps->lastRisingEdge) /
        (edgeTimestamps->currRisingEdge - edgeTimestamps->lastRisingEdge);
}

/**
 * @brief Returns the PWM Input signal list
 * @param id String identifier for desired signal
 * @return Structure of PWM signals
 */
PWMSignal_t
getPWMInputSignals (char* id)
{
    for (int i = 0; i < PWMInputSignals.count; i++)
    {
        if (strcmp(PWMInputSignals.signals[0].id, id) == 0)
        {
            return PWMInputSignals.signals[0];
        }
    }

    // TASK - what if the incorrect id is input? 
}