/**********************************************************
 *
 * ap_pwm_input.c - Main controlling file for the 
 *      PWM information.
 *
 * IMPORTANT - This assumes all signals are on port B
 * 
 * T.R Peterson, M.C Gardyne
 * Last modified:  24.7.22
 **********************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "driverlib/timer.h"

#include "ap_pwm_input.h"

//*************************************************************
// Constant Definitions
//*************************************************************

#define LEN(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))
#define MAX_NUM_SIGNALS         6

#define EDGE_TIMER_PERIPH       SYSCTL_PERIPH_TIMER0
#define EDGE_TIMER_BASE         TIMER0_BASE
#define EDGE_TIMER              TIMER_A
#define EDGE_TIMER_CONFIG       TIMER_CFG_A_PERIODIC_UP

#define TIMEOUT_TIMER_PERIPH    SYSCTL_PERIPH_TIMER2
#define TIMEOUT_TIMER_BASE      TIMER2_BASE
#define TIMEOUT_TIMER           TIMER_A
#define TIMEOUT_TIMER_CONFIG    TIMER_CFG_A_PERIODIC
#define TIMEOUT_TIMER_INT_FLAG  TIMER_TIMA_TIMEOUT
#define TIMEOUT_RATE            18 // [Hz]

#define PWM_GPIO_BASE           GPIO_PORTB_BASE
#define PWM_GPIO_PERIPH         SYSCTL_PERIPH_GPIOB

//*************************************************************
// Type Definitions
//*************************************************************

/**
 * @brief Identifies the number of edges found in an update
 */
enum PWMEdgesFound {NONE, ONE_EDGE, TWO_EDGES};

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
// Function prototype
//*************************************************************
static void PWMEdgeIntHandler (void);
static void PWMTimeoutHandler (void);
static void calculatePWMProperties (PWMSignal_t* PWMSignal);
static int updateAllPWMInputs(void);
static bool updatePWMInfo(PWMSignal_t* PWMSignal);
static void calculatePWMProperties(PWMSignal_t* PWMSignal);
static PWMSignal_t* findPWMInput(char* id);

//*************************************************************
// FreeRTOS handles
//*************************************************************
QueueHandle_t PWMSignalQueue = NULL;

//*************************************************************
// Global variables
//*************************************************************
static PWMInputSignals_t PWMInputSignals;
static volatile enum PWMEdgesFound risingEdgeCount;
static volatile edgeTimestamps_t edgeTimestamps;
static volatile bool PWMReadTimeout = false;

/**
 * @brief Initialise the timer used for edge tracking
 * @return None
 */
static void
initPWMEdgeTimer (void)
{
    SysCtlPeripheralEnable(EDGE_TIMER_PERIPH);

    TimerDisable(EDGE_TIMER_BASE, EDGE_TIMER);

    TimerConfigure(EDGE_TIMER_BASE, EDGE_TIMER_CONFIG);

    TimerLoadSet(EDGE_TIMER_BASE, EDGE_TIMER, SysCtlClockGet());

    TimerEnable(EDGE_TIMER_BASE, EDGE_TIMER);
}

/**
 * @brief Initialise the timer for PWM edge timeout
 * @return None
 */
static void
initPWMTimeoutTimer (void)
{
    SysCtlPeripheralEnable(TIMEOUT_TIMER_PERIPH);

    TimerDisable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);

    TimerConfigure(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER_CONFIG);

    TimerIntRegister(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER, PWMTimeoutHandler);

    TimerLoadSet(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER, SysCtlClockGet() / TIMEOUT_RATE);

    TimerIntEnable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER_INT_FLAG);

    TimerDisable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);
}

/**
 * @brief Initialise the pins of a given inputSignal
 * @param inputSignal - PWM signal to be initialised
 * @return None
 */
static void
initPWMInput (PWMSignal_t inputSignal)
{
    SysCtlPeripheralEnable(PWM_GPIO_PERIPH);

    GPIOPinTypeGPIOInput(PWM_GPIO_BASE, inputSignal.gpioPin);

    GPIOIntTypeSet(PWM_GPIO_BASE, inputSignal.gpioPin, GPIO_BOTH_EDGES);

    GPIOIntRegister(PWM_GPIO_BASE, PWMEdgeIntHandler);

    GPIOIntDisable(PWM_GPIO_BASE, inputSignal.gpioPin);
}

/**
 * @brief Initialise the PWM input manager module
 * @return None
 */
void
initPWMInputManager (void)
{
    initPWMEdgeTimer();

    initPWMTimeoutTimer();
}

/**
 * @brief Add a PWM signal to the list of registered input signals
 * @param newSignal - PWM signal to be added
 * @return Bool - true if successful, false if failed
 */
int
registerPWMSignal (PWMSignal_t newSignal)
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
PWMEdgeIntHandler (void)
{
    if (GPIOPinRead (PWM_GPIO_BASE, GPIOIntStatus(PWM_GPIO_BASE, true)))
    {
        edgeTimestamps.lastRisingEdge = edgeTimestamps.currRisingEdge;
        edgeTimestamps.currRisingEdge = TimerValueGet(EDGE_TIMER_BASE, EDGE_TIMER);

        risingEdgeCount++;
    }
    else
    {
        edgeTimestamps.currFallingEdge = TimerValueGet(EDGE_TIMER_BASE, EDGE_TIMER);
    }

    // if edge is two edges then queue edgeTimestamps


    GPIOIntClear(PWM_GPIO_BASE, PWMInputSignals.pins);
}

/**
 * @brief Interrupt handler for PWM reading timeout
 * @return None
 */
static void
PWMTimeoutHandler (void)
{
    TimerIntClear(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER_INT_FLAG);

    // TESTING
    // char str[30];
    // static int hitCount = 0;
    // sprintf(str, "Hit count = %d\r\n", hitCount++);
    // UARTSend(str);

    PWMReadTimeout = true;
}

/**
 * @brief Reset PWM timeout
 * @return None
 */
static void
resetTimeout (void)
{
    TimerIntClear(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER_INT_FLAG);

    TimerLoadSet(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER, SysCtlClockGet() / TIMEOUT_RATE);

    PWMReadTimeout = false;
}

/**
 * @brief Regularly scheduled task for updating all PWM signals
 * @return None
 */
void updateAllPWMInputsTask(void* args)
{
    (void)args;
    const TickType_t xDelay = 330 / portTICK_PERIOD_MS;

    // PWMSignalQueue = xQueueCreate(8, sizeof(PWMSignal_t*));

    while (true) 
    {
        updateAllPWMInputs();

        vTaskDelay(xDelay);
    }   
}

/**
 * @brief Updates all PWM signal information
 * @return Count off failed PWM signal updates
 */
static int 
updateAllPWMInputs(void)
{
    int failedUpdates = 0;

    for (int i = 0; i < PWMInputSignals.count; i++)
    {
        failedUpdates += updatePWMInfo(&PWMInputSignals.signals[i]);

        // printPWM(PWMInputSignals.signals[i].id); // TESTING
    }  

    return failedUpdates; 
}

/**
 * @brief Updates specific PWM signal information
 * @return Count off failed PWM signal updates
 */
int 
updatePWMInput(char* id)
{
    return updatePWMInfo(findPWMInput(id));
}

/**
 * @brief Enable and update the duty and frequency of a given PWM signal
 * @param PWMSignal - PWM signal to update
 * @return Bool - Boolean representing whether there was an error
 */
static bool 
updatePWMInfo(PWMSignal_t* PWMSignal)
{
    risingEdgeCount = NONE; // May need to be specific to PWM signal - not sure yet

    GPIOIntClear(PWM_GPIO_BASE, PWMInputSignals.pins);

    resetTimeout();

    // TASK - disable anything that might interrupt this section
    TimerEnable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);
    GPIOIntEnable(PWM_GPIO_BASE, PWMSignal->gpioPin);
    while (risingEdgeCount != TWO_EDGES && !PWMReadTimeout);
    GPIOIntDisable(PWM_GPIO_BASE, PWMSignal->gpioPin);
    TimerDisable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);

    if (!PWMReadTimeout)
    {
        calculatePWMProperties(PWMSignal);
        // xQueueSendToBack(PWMSignalQueue, &PWMSignal, portMAX_DELAY);

        return false;
    }

    PWMSignal->frequency = 0;
    PWMSignal->duty = 0;
    return true;
}

/**
 * @brief Tasks for managing the PWM signal update queue
 * @param args Task arguments
 */
void
calculatePWMPropertiesTask(void* args)
{
    (void)args;
    const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
    
    PWMSignal_t *PWMSignal;
    while (true) 
    {
        if (xQueueReceive(PWMSignalQueue, &PWMSignal, portMAX_DELAY) == pdPASS)
        {
            // char str[100];
            // sprintf(str, "ID : %d\r\n\n", PWMSignal->duty);
            // UARTSend(str);

            calculatePWMProperties(PWMSignal);
        }
        
        // TASK - wasn't dealing with timeout very well. Need to be able to do that

        vTaskDelay(xDelay);
    }  
}

/**
 * @brief Update the duty and frequency given a PWM signal's edge timestamps
 * @param PWMSgnal - PWM signal to update
 * @param edgeTimestamps - Timestamps of the required edge points 
 * @return None
 */
static void
calculatePWMProperties(PWMSignal_t* PWMSignal)
{
    // Account for overflow of the timer
    if (edgeTimestamps.currRisingEdge < edgeTimestamps.lastRisingEdge)
    {
        PWMSignal->frequency = SysCtlClockGet() / (edgeTimestamps.currRisingEdge
            + SysCtlClockGet() - edgeTimestamps.lastRisingEdge);
    }
    else
    {
        PWMSignal->frequency = SysCtlClockGet() / (edgeTimestamps.currRisingEdge
            - edgeTimestamps.lastRisingEdge);
    }

    // Currently no timer overflow protection - TASK
    PWMSignal->duty = 100 * (edgeTimestamps.currFallingEdge - edgeTimestamps.lastRisingEdge) /
        (edgeTimestamps.currRisingEdge - edgeTimestamps.lastRisingEdge);

    // TASK - if freq not zero and duty zero, then the duty is wrong
}

/**
 * @brief Locates the address of a desired PWM signal
 * @param id String identifier for desired signal
 * @return Pointer of desired PWM signal. Return NULL if not found
 */
static PWMSignal_t*
findPWMInput(char* id)
{
    for (int i = 0; i < PWMInputSignals.count; i++)
    {
        if (strcmp(PWMInputSignals.signals[i].id, id) == 0)
        {
            return &PWMInputSignals.signals[i];
        }
    }

    return NULL;
}

/**
 * @brief Returns the identified PWM Input signal
 * @param id String identifier for desired signal
 * @return Structure of PWM signal
 */
PWMSignal_t
getPWMInputSignal (char* id)
{
    return *findPWMInput(id);
}

// TESTING
// void
// printPWM(char* id)
// {
//     char str[100];
//     PWMSignal_t signal;
//     // Details of first PWM
//     signal = getPWMInputSignal(id);
//     sprintf(str, "Signal ID = %s\r\n", id);
//     UARTSend(str);
//     sprintf(str, "Frequency = %ld Hz\r\n", signal.frequency);
//     UARTSend(str);
//     sprintf(str, "Duty : %ld\r\n\n", signal.duty);
//     UARTSend(str);
// }