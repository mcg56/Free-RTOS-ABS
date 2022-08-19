/**********************************************************
 *
 * brake_output.c - File for output brake signal
 * 
 * IMPORTANT - Much of this needs to be moved to abs_manager
 *
 * T.R Peterson, M.C Gardyne
 * Last modified:  3.8.22
 **********************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/pwm.h>
#include <driverlib/pin_map.h>

#include <FreeRTOS.h>
#include <task.h>

#include "libs/lib_pwm/pwm_output.h"
#include "brake_output.h"
#include "status_led.h"

//*************************************************************
// Constant Definitions
//*************************************************************
#define ABS_DUTY_DEFAULT        50      // [%]
#define LED_BLINK_RATE_COEFF    0.08    // Coefficient for relating brake duty to blink rate

#define BRAKE_BASE	            PWM0_BASE
#define BRAKE_GEN               PWM_GEN_3
#define BRAKE_OUTNUM            PWM_OUT_7
#define BRAKE_OUTBIT            PWM_OUT_7_BIT
#define BRAKE_PERIPH_PWM	    SYSCTL_PERIPH_PWM0
#define BRAKE_PERIPH_GPIO       SYSCTL_PERIPH_GPIOC
#define BRAKE_GPIO_BASE         GPIO_PORTC_BASE
#define BRAKE_GPIO_CONFIG       GPIO_PC5_M0PWM7
#define BRAKE_GPIO_PIN          GPIO_PIN_5

#define PULSE_ABS_TASK_RATE     50 // [ms] (From requirements)
#define UPDATE_ABS_TASK_RATE    50 // [ms]

//*************************************************************
// Function prototype
//*************************************************************
void            pulseABS        (void);
static void     setABSState     (enum absStates state);
void            updateABSTask   (void* args);
void            pulseABSTask    (void* args);
void            updateABS       (void);
static void     toggleABSState  (void);

//*****************************************************************************
// Global variables
//*****************************************************************************
static enum absStates absState = ABS_OFF;
static uint8_t ABSDuty = ABS_DUTY_DEFAULT;

static PWMOutputHardwareDetails_t brakeSignal = {
    .base           = BRAKE_BASE, 
    .gen            = BRAKE_GEN, 
    .outnum         = BRAKE_OUTNUM, 
    .outbit         = BRAKE_OUTBIT, 
    .periphPWM      = BRAKE_PERIPH_PWM,
    .periphGPIO     = BRAKE_PERIPH_GPIO,
    .gpioBase       = BRAKE_GPIO_BASE,
    .gpioConfig     = BRAKE_GPIO_CONFIG,
    .gpioPin        = BRAKE_GPIO_PIN,
};

static TaskHandle_t pulseABSHandle;
static TaskHandle_t updateABSHandle;

/**
 * @brief Initialise brake output module
 * @return None
 */
void
initBrakeOutput (void)
{
    initializePWMGeneral (brakeSignal, 0, 0);
    initStatusLED ();

    xTaskCreate(&updateABSTask, "updateABS", 256, NULL, 0, &updateABSHandle);
    xTaskCreate(&pulseABSTask, "pulseABS", 256, NULL, 0, &pulseABSHandle);

    updateABS();
}

/**
 * @brief Pulse the ABS at a set rate
 * @return None
 */
void 
updateABSTask (void* args)
{
    (void)args;

    const TickType_t xDelay = UPDATE_ABS_TASK_RATE / portTICK_PERIOD_MS;

    while (true)
    {
        // Wait until a task has notified it to run
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

        updateABS();

        vTaskDelay(xDelay);
    }   
}

/**
 * @brief Update the ABS state
 * @return None
 */
void 
updateABS (void)
{
    switch (absState)
    {
        case ABS_ON:
            vTaskResume (pulseABSHandle);
            setStatusLEDState (BLINKING);
            break;
        case ABS_OFF:
            vTaskSuspend (pulseABSHandle);
            setPWMGeneral (500, ABSDuty, brakeSignal.base, brakeSignal.gen, brakeSignal.outnum);
            setStatusLEDState (FIXED_ON);
            break;
    }
}

/**
 * @brief Pulse the ABS at a set rate
 * @return None
 */
void 
pulseABSTask (void* args)
{
    (void)args;

    const TickType_t xDelay = PULSE_ABS_TASK_RATE / portTICK_PERIOD_MS; // Check timing

    while (true)
    {
        pulseABS();

        setStatusLEDBlinkRate(ABSDuty * LED_BLINK_RATE_COEFF);

        vTaskDelay(xDelay);
    }   
}

/**
 * @brief Change the ABS output
 * @return None
 */
void 
pulseABS (void)
{
    static bool pulseOn = true;

    if (pulseOn)
    {
        setPWMGeneral (500, ABSDuty, brakeSignal.base, brakeSignal.gen, brakeSignal.outnum);
        pulseOn = false;
    }
    else
    {
        setPWMGeneral (0, 0, brakeSignal.base, brakeSignal.gen, brakeSignal.outnum);
        pulseOn = true;
    }  
}

/**
 * @brief External function to set ABS
 * @return None
 */
void
setABS (enum absStates state)
{
    setABSState(state);

    // Tell the abs controller to update its status
    xTaskNotifyGiveIndexed( updateABSHandle, 0 );
}

/**
 * @brief Sets the ABS state
 * @param state The state to set the ABS to
 * @return None
 */
static void 
setABSState (enum absStates state)
{
    if (state == ABS_ON)
    {
        absState = ABS_ON;
    }
    else
    {
        absState = ABS_OFF;
    }
}

/**
 * @brief External access to toggle the ABS
 * @return None
 */
void
toggleABS (void)
{
    toggleABSState();

    // Tell the abs controller to update its status
    xTaskNotifyGiveIndexed( updateABSHandle, 0 ); 
}

/**
 * @brief Toggle the ABS state.
 * @return None
 */
static void 
toggleABSState (void)
{
    if (absState == ABS_ON)
    {
        setABSState(ABS_OFF);
    }
    else
    {
        setABSState(ABS_ON);
    }
}

/**
 * @brief Sets the duty cycle of the ABS signal
 * @param duty - Percentage duty cycle
 * @return int - 1 if successful, 0 if failed
 */
int 
setABSDuty (uint8_t duty)
{
    if (duty < 5 || duty > 95) return 0;

    ABSDuty = duty;

    return 1;
}

/**
 * @brief Passes the ABS state out of the module
 * @return enum absStates - Current ABS state
 */
enum absStates 
getABSState (void)
{
    return absState;
}

/**
 * @brief Passes the ABS duty out of the module
 * @return int ABSDuty - The current ABS duty
 */
uint8_t 
getABSDuty (void)
{
    return ABSDuty;
}