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
 * @brief Types of information displayed on screen
 */
enum screenType {ABS_INFO = 0, PWM_INFO};

/**
 * @brief ABS display information
 * 
 * @param absState - Current ABS State
 * @param duty - Duty cycle in Hz
 */
typedef struct {
    enum absStates absState;
    uint8_t duty;
} ABSScreen_t;

/**
 * @brief PWM display information
 * 
 * @param pwmSignal - PWM signal
 */
typedef struct {
    PWMSignal_t pwmSignal;
} PWMSignalScreen_t;

/**
 * @brief Display information
 * 
 * @param type - Type of information on the screen
 * @param content - Contents of the screen
 */
typedef struct {
    enum screenType type;
    union ScreenContent {
        ABSScreen_t absScreen;
        PWMSignalScreen_t pwmScreen;
    } content;
} Screen_t;

//*************************************************************
// Function prototype
//*************************************************************
static void updateDisplayTask (void* args);
static void updateDisplayButtonsTask (void* args);
static void updateDisplay (void);
static void updateSelectedScreen (void);
static void updateScreenIndex (void);
static void updateScreen (void);
static void OLEDDrawABSScreen (void);
static void OLEDDrawPWMScreen (void);
static void clearScreen (void);
static char* getABSStateName (enum absStates state);
static PWMSignal_t getSelectedPWM (void);

//*****************************************************************************
// Global variables
//*****************************************************************************
static Screen_t screen; 
static int screenIndex = 0;

/**
 * @brief Initialise the display module
 * 
 * @return None
 */
void
initDisplay (void)
{
    initButtons ();
    OLEDInitialise ();

    xTaskCreate(&updateDisplayTask, "updateDisplay", 256, NULL, 0, NULL);
    xTaskCreate(&updateDisplayButtonsTask, "updateButtons", 256, NULL, 0, NULL);
}

/**
 * @brief Task used to update the display module
 * 
 * @return None
 */
static void
updateDisplayTask (void* args)
{
    (void)args;

    const TickType_t xDelay = 100 / portTICK_PERIOD_MS; //TO DO: set rate

    while (true)
    {
        updateDisplay();

        vTaskDelay(xDelay);
    }   
}

//TO DO
static void
updateDisplayButtonsTask (void* args)
{
    (void)args;

    const TickType_t xDelay = 10 / portTICK_PERIOD_MS; //TO DO: set rate

    while (true)
    {
        updateButtons ();

        vTaskDelay(xDelay);
    }   
}

/**
 * @brief Update the OLED display information
 * 
 * @return None
 */
static void 
updateDisplay (void)
{
    updateSelectedScreen ();

    updateScreen ();
}

/**
 * @brief Update what screen has been selected by the user
 * 
 * @return None
 */
static void
updateSelectedScreen (void)
{
    updateScreenIndex();

    if (screenIndex == 0)
    {
        screen.content.absScreen.absState = getABSState();
        screen.content.absScreen.duty = getABSDuty();
        screen.type = ABS_INFO;
    }
    else
    {
        screen.content.pwmScreen.pwmSignal = getSelectedPWM();
        screen.type = PWM_INFO;
    }
}

/**
 * @brief Get the Selected PWM object at the screen index
 * 
 * @return PWMSignal_t - PWM signal
 */
static PWMSignal_t
getSelectedPWM (void)
{
    int numOfSignals = getCountPWMInputs();
    char* pwmIDs[numOfSignals];

    getIDList(pwmIDs, numOfSignals);

    return getPWMInputSignal(pwmIDs[screenIndex - 1]);
}

/**
 * @brief Update the screen index
 * 
 * @return None
 */
static void
updateScreenIndex (void)
{
    if (checkButton(LEFT) == PUSHED) // TO DO: Change to up and down
    {
        if (screenIndex == getCountPWMInputs())
        {
            screenIndex = 0;
        }
        else
        {
            screenIndex++;
        }
    }
}

/**
 * @brief Update the displaying screen information
 * 
 * @return None
 */
static void
updateScreen (void)
{
    // Clear screen if changing types
    static enum screenType prevScreenType = ABS_INFO;
    if (screen.type != prevScreenType)
    {
        clearScreen ();
        prevScreenType = screen.type;
    }

    switch (screen.type)
    {
        case ABS_INFO:
            OLEDDrawABSScreen ();
            break;
        case PWM_INFO:
            OLEDDrawPWMScreen ();
            break;
    } 
}

/**
 * @brief Updates the screen for an ABS information display
 * 
 * @return None
 */
static void
OLEDDrawABSScreen (void)
{
    char str[17]; // Display fits 16 characters wide.
	
    usnprintf (str, sizeof(str), "---Brake Info---");
    OLEDStringDraw (str, 0, 0);
    usnprintf (str, sizeof(str), "ABS State:   %s ", getABSStateName(screen.content.absScreen.absState));
    OLEDStringDraw (str, 0, 1);
    usnprintf (str, sizeof(str), "ABS Duty:    %2d%%", screen.content.absScreen.duty);
    OLEDStringDraw (str, 0, 2);
}

/**
 * @brief Updates the screen for a PWM information display
 * 
 * @return None
 */
static void
OLEDDrawPWMScreen (void)
{
    char str[17]; // Display fits 16 characters wide.
	
    usnprintf (str, sizeof(str), "----PWM Info----");
    OLEDStringDraw (str, 0, 0);
    usnprintf (str, sizeof(str), "ID:", screen.content.pwmScreen.pwmSignal.id);
    OLEDStringDraw (str, 0, 1);
    usnprintf (str, sizeof(str), "%s", screen.content.pwmScreen.pwmSignal.id);
    OLEDStringDraw (str, 16 - strlen(str), 1); // Right align ID
    usnprintf (str, sizeof(str), "Freq:     %3d Hz", screen.content.pwmScreen.pwmSignal.frequency);
    OLEDStringDraw (str, 0, 2);
    usnprintf (str, sizeof(str), "Duty:        %2d%%", screen.content.pwmScreen.pwmSignal.duty);
    OLEDStringDraw (str, 0, 3);
}

/**
 * @brief Clears the OLED screen
 *  
 * @return None
 */
static void
clearScreen (void)
{
    char str[17]; // Display fits 16 characters wide.
	
    usnprintf (str, sizeof(str), "                ");
    OLEDStringDraw (str, 0, 0);
    OLEDStringDraw (str, 0, 1);
    OLEDStringDraw (str, 0, 2);
    OLEDStringDraw (str, 0, 3);
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