/**********************************************************
 *
 * ap_pwm_input.c - Main controlling file for the 
 *      PWM information.
 *
 * IMPORTANT - This assumes all signals are on port B or port C
 * 
 * T.R Peterson, M.C Gardyne
 * Last modified:  24.7.22
 **********************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stddef.h>


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
#define MAX_NUM_SIGNALS         6   // Maximum number of signals

#define EDGE_TIMER_PERIPH       SYSCTL_PERIPH_TIMER0
#define EDGE_TIMER_BASE         TIMER0_BASE
#define EDGE_TIMER              TIMER_A
#define EDGE_TIMER_CONFIG       TIMER_CFG_A_PERIODIC_UP
#define RISING_EDGE_TIMEOUT     2   // Minimum rising edges during an update

#define TIMEOUT_TIMER_PERIPH    SYSCTL_PERIPH_TIMER2
#define TIMEOUT_TIMER_BASE      TIMER2_BASE
#define TIMEOUT_TIMER           TIMER_A
#define TIMEOUT_TIMER_CONFIG    TIMER_CFG_A_PERIODIC
#define TIMEOUT_TIMER_INT_FLAG  TIMER_TIMA_TIMEOUT
#define TIMEOUT_DEFAULT_RATE    30  // [Hz]

#define PWM_GPIO_BASE           GPIO_PORTB_BASE
#define PWM_GPIO_PERIPH         SYSCTL_PERIPH_GPIOB | SYSCTL_PERIPH_GPIOC

//*************************************************************
// Type Definitions
//*************************************************************

/**
 * @brief Collection of all input PWM signals
 * 
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
 * 
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
// Function prototypes
//*************************************************************
static void setPWMTimeout (uint16_t timeoutRate);
static void PWMEdgeIntHandler (void);
static void PWMTimeoutHandler (void);
static void calculatePWMProperties (PWMSignal_t* PWMSignal);
static int updateAllPWMInputs(void);
static bool refreshPWMDetails(PWMSignal_t* PWMSignal);
static void calculatePWMProperties(PWMSignal_t* PWMSignal);
static PWMSignal_t* findPWMInput(char* id);

//*************************************************************
// FreeRTOS handles
//*************************************************************
QueueHandle_t PWMSignalQueue = NULL;

//*************************************************************
// Static variables
//*************************************************************
static PWMInputSignals_t PWMInputSignals;
static volatile int risingEdgeCount;
static volatile edgeTimestamps_t edgeTimestamps;
static volatile bool PWMReadTimeout = false;
static uint16_t PWMTimeoutRate = TIMEOUT_DEFAULT_RATE;

/**
 * @brief Initialise the timer used for edge tracking
 * 
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
 * 
 * @return None
 */
static void
initPWMTimeoutTimer (void)
{
    SysCtlPeripheralEnable(TIMEOUT_TIMER_PERIPH);

    TimerDisable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);

    TimerConfigure(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER_CONFIG);

    TimerIntRegister(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER, PWMTimeoutHandler);

    TimerLoadSet(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER, SysCtlClockGet() / PWMTimeoutRate);

    TimerIntEnable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER_INT_FLAG);

    TimerDisable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);
}

/**
 * @brief Initialise the pins of a given inputSignal
 * 
 * @param inputSignal - PWM signal to be initialised
 * @return None
 */
static void
initPWMInput (PWMSignal_t inputSignal)
{    
    GPIOPinTypeGPIOInput(inputSignal.gpioPort, inputSignal.gpioPin);

    GPIOIntTypeSet(inputSignal.gpioPort, inputSignal.gpioPin, GPIO_BOTH_EDGES);

    GPIOIntRegister(inputSignal.gpioPort, PWMEdgeIntHandler);

    GPIOIntDisable(inputSignal.gpioPort, inputSignal.gpioPin); 
}

/**
 * @brief Initialise the PWM input manager module
 * 
 * @param PWMMinFreq - Minimum PWM frequency to be read
 * @return None
 */
void
initPWMInputManager (uint16_t PWMMinFreq)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    setPWMMinFeq(PWMMinFreq);

    initPWMEdgeTimer();

    initPWMTimeoutTimer();
}


/**
 * @brief Sets the PWM minimum frequency
 * 
 * @param PWMMinFreq - Minimum PWM frequency
 * @return None
 */
void
setPWMMinFeq (uint16_t PWMMinFreq)
{
    if (PWMMinFreq == 0) return;

    // To read a signal of freq x, the timeout must be x/2 
    // This allows two rising edges to be registered starting 
    // at any point in the PWM signal
    setPWMTimeout(PWMMinFreq / 2); 

    // TO DO: Anything else to check if frequency is valid?
}

/**
 * @brief Sets the timeout for the PWM timeout timer
 * 
 * @param timeoutRate - Frequency of timer timout
 * @return None
 */
static void
setPWMTimeout (uint16_t timeoutRate)
{
    PWMTimeoutRate = timeoutRate;
}

/**
 * @brief Add a PWM signal to the list of registered input signals
 * 
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
 * 
 * @return None
 */
static void
PWMEdgeIntHandler (void)
{
    uint32_t base;

    if (GPIOIntStatus(GPIO_PORTB_BASE, true) != 0)
    {
        base = GPIO_PORTB_BASE;
    }
    else if (GPIOIntStatus(GPIO_PORTC_BASE, true) != 0)
    {
        base = GPIO_PORTC_BASE; 
    }

    if (GPIOPinRead (base, GPIOIntStatus(base, true)))
    {
        edgeTimestamps.lastRisingEdge = edgeTimestamps.currRisingEdge;
        edgeTimestamps.currRisingEdge = TimerValueGet(EDGE_TIMER_BASE, EDGE_TIMER);

        risingEdgeCount++;
    }
    else
    {
        edgeTimestamps.currFallingEdge = TimerValueGet(EDGE_TIMER_BASE, EDGE_TIMER);
    }

    // TO DO: if edge is two edges then queue edgeTimestamps

    GPIOIntClear(base, PWMInputSignals.pins);
}

/**
 * @brief Interrupt handler for PWM reading timeout
 * 
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
 * 
 * @return None
 */
static void
resetTimeout (void)
{
    TimerIntClear(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER_INT_FLAG);

    TimerLoadSet(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER, SysCtlClockGet() / PWMTimeoutRate);

    PWMReadTimeout = false;
}

/**
 * @brief Regularly scheduled task for updating all PWM signals
 * 
 * @return None
 */
void updateAllPWMInputsTask(void* args)
{
    (void)args;
    const TickType_t xDelay = 330 / portTICK_PERIOD_MS; // TO DO: magic number

    // PWMSignalQueue = xQueueCreate(8, sizeof(PWMSignal_t*)); TESTING

    while (true) 
    {
        updateAllPWMInputs(); // TO DO: Split into individual processes

        vTaskDelay(xDelay);
    }   
}

/**
 * @brief Updates all PWM signal information
 * 
 * @return Count off failed PWM signal updates
 */
static int 
updateAllPWMInputs(void)
{
    int failedUpdates = 0;

    for (int i = 0; i < PWMInputSignals.count; i++)
    {
        failedUpdates += refreshPWMDetails(&PWMInputSignals.signals[i]);

        // printPWM(PWMInputSignals.signals[i].id); // TESTING
    }  

    return failedUpdates; 
}

/**
 * @brief Updates specific PWM signal information
 * 
 * @return Count off failed PWM signal updates
 */
int 
updatePWMInput(char* id)
{
    return refreshPWMDetails(findPWMInput(id));
}

/**
 * @brief Refresh the duty and frequency of a given PWM signal
 * 
 * @param PWMSignal - PWM signal to refresh
 * @return Bool - Boolean representing whether there was an error
 */
static bool 
refreshPWMDetails(PWMSignal_t* PWMSignal)
{
    risingEdgeCount = 0; // May need to be specific to PWM signal - not sure yet

    GPIOIntClear(PWMSignal->gpioPort, PWMInputSignals.pins);

    resetTimeout();

    // TO DO - disable anything that might interrupt this section
    GPIOIntEnable(PWMSignal->gpioPort, PWMSignal->gpioPin);
    TimerEnable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);
    while (risingEdgeCount < RISING_EDGE_TIMEOUT && !PWMReadTimeout);
    TimerDisable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);
    GPIOIntDisable(PWMSignal->gpioPort, PWMSignal->gpioPin);

    if (!PWMReadTimeout)
    {
        calculatePWMProperties(PWMSignal);
        // xQueueSendToBack(PWMSignalQueue, &PWMSignal, portMAX_DELAY); TESTING

        return false;
    }

    PWMSignal->frequency = 0;
    PWMSignal->duty = 0;
    return true;
}


// TO DO: CURRENTLY UNUSED
/**
 * @brief Tasks for managing the PWM signal update queue
 * 
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

        vTaskDelay(xDelay);
    }  
}

/**
 * @brief Update the duty and frequency given a PWM signal's edge timestamps
 * 
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


    uint32_t duty = ceil(100 * (float)(edgeTimestamps.currFallingEdge - edgeTimestamps.lastRisingEdge) /
        (float)(edgeTimestamps.currRisingEdge - edgeTimestamps.lastRisingEdge));

    if (duty > 0 || duty < 100)
    {
        PWMSignal->duty = duty;
    }
}

/**
 * @brief Locates the address of a desired PWM signal
 * 
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
 * 
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

/**
 * @brief Returns the number of PWM signals being tracked
 * 
 * @return int - Number of signals
 */
int 
getCountPWMInputs (void)
{
    return PWMInputSignals.count;
}

/**
 * @brief Provides the IDs of all tracking input signals
 * 
 * @param ids - An array to fill the IDs with
 * @param len - The length of the ids list
 * @return None
 */
void
getIDList (char* ids[], size_t len)
{
    for (int i = 0; i < PWMInputSignals.count; i++)
    {
        if (i == (int)len) return;

        ids[i] = PWMInputSignals.signals[i].id;
    }
}