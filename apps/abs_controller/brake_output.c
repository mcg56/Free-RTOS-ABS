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

#include <FreeRTOS.h>
#include <task.h>

#include "libs/lib_pwm/ap_pwm_output.h"
#include "brake_output.h"

//*************************************************************
// Constant Definitions
//*************************************************************
#define ABS_DUTY_DEFAULT 50 // [%]

// This will be the output pin details but right now its just PC5

//*************************************************************
// Function prototype
//*************************************************************
void pulseABS (void);
static void setABSState (enum absStates state);
void updateABSTask (void* args);
void pulseABSTask (void* args);
void updateABS (void);
static void toggleABSState (void);

//*****************************************************************************
// Global variables
//*****************************************************************************
static enum absStates absState = ABS_OFF;
static uint8_t ABSDuty = ABS_DUTY_DEFAULT;

static TaskHandle_t pulseABSHandle;
static TaskHandle_t updateABSHandle;

/**
 * @brief Initialise brake output module
 * @return None
 */
void
initBrakeOutput (void)
{
    initialisePWM ();

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

    const TickType_t xDelay = 50 / portTICK_PERIOD_MS;

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
            vTaskResume(pulseABSHandle);
            break;
        case ABS_OFF:
            vTaskSuspend(pulseABSHandle);
            setPWM(500, ABSDuty);
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

    const TickType_t xDelay = 50 / portTICK_PERIOD_MS; // Check timing

    while (true)
    {
        pulseABS();

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
        setPWM (500, ABSDuty);
        pulseOn = false;
    }
    else
    {
        setPWM (0, 0);
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