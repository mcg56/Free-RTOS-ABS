/**********************************************************
 *
 * display.c - Displays information on the OrbitOLED.
 *
 * T.R Peterson, M.C Gardyne
 * Last modified:  6.8.22
 **********************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "utils/ustdlib.h"

#include "libs/lib_buttons/ap_buttons.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include "libs/lib_pwm/ap_pwm_input.h"

#include "display.h"
#include "brake_output.h"

//*************************************************************
// Constant Definitions
//*************************************************************


//*************************************************************
// Type Definitions
//*************************************************************

/**
 * @brief Collection of all input PWM signals
 * @param signals - List of PWM signals
 * @param count - Count of PWM input signal
 * @param pins - Bitmask of the input signal pins //UPDATE
 */
typedef struct {
    enum absStates absState;
    uint8_t duty;
} ABSScreen_t;

/**
 * @brief 
 * 
 */
typedef struct {
    PWMSignal_t pwmSignal;
} PWMSignalScreen_t;

//*************************************************************
// Function prototype
//*************************************************************
static void updateDisplayTask (void* args);
static void updateDisplay (void);
static void updateCurrentScreen (void);
static void OLEDDrawScreen (void);
static char* getABSStateName (enum absStates state);

//*****************************************************************************
// Global variables
//*****************************************************************************
static ABSScreen_t absScreen;

/**
 * @brief Initialise the display module
 * @return None
 */
void
initDisplay (void)
{
    OLEDInitialise ();

    xTaskCreate(&updateDisplayTask, "updateDisplay", 256, NULL, 0, NULL);
}

static void
updateDisplayTask (void* args)
{
    (void)args;

    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

    while (true)
    {
        updateDisplay();

        vTaskDelay(xDelay);
    }   
}

static void 
updateDisplay (void)
{
    updateCurrentScreen ();
}


static void
updateCurrentScreen (void)
{
    absScreen.absState = getABSState();
    absScreen.duty = getABSDuty();

    OLEDDrawScreen ();
}

static void
OLEDDrawScreen (void)
{
    char str[17]; // Display fits 16 characters wide.
	
    usnprintf (str, sizeof(str), "---Brake Info---");
    OLEDStringDraw (str, 0, 0);
    usnprintf (str, sizeof(str), "ABS State:   %s ", getABSStateName(absScreen.absState));
    OLEDStringDraw (str, 0, 1);
    usnprintf (str, sizeof(str), "ABS Duty:    %2d%%", absScreen.duty);
    OLEDStringDraw (str, 0, 2);
}


// Maybe shouldn't be in this file?
static char* 
getABSStateName (enum absStates state)
{
    switch (state)
    {
        case ABS_ON: return "ON";
        case ABS_OFF: return "OFF";
        default: return "Unknown";
    }
}