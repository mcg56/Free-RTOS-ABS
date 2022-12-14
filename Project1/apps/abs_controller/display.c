/** @file   display.c
    @author T. Peterson, M. Gardyne
    @date   22/08/22
    @brief  Displays information on the OrbitOLED
*/

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
#include <queue.h>
#include <semphr.h>

#include "utils/ustdlib.h"

#include "Project1/libs/lib_buttons/buttons.h"
#include "Project1/libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include "Project1/libs/lib_pwm/pwm_input.h"

#include "display.h"
#include "brake_output.h"
#include "abs_manager.h"

//*************************************************************
// Constant Definitions
//*************************************************************
#define OLED_CHAR_WIDTH                     17 // OLED is 16 characters wide
#define MAX_ID_LEN                          12
#define MAX_STATE_LEN                       6
#define MAX_ANGLE_LEN                       4

#define UPDATE_DISPLAY_TASK_RATE            200 // [ms]
#define UPDATE_BUTTONS_TASK_RATE            10 // [ms]
#define UPDATE_DISPLAY_TASK_PRIORITY        2
#define UPDATE_BUTTONS_TASK_PRIORITY        2 
#define OLED_DRAW_TASK_PRIORITY             2

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

/**
 * @brief Information to write one line on the OLED display
 * 
 * @param str - String to write
 * @param col - Column to start string from
 * @param row - Row to draw string on
 */
typedef struct {
    char str[OLED_CHAR_WIDTH];
    int col;
    int row;
} OLEDDraw_t;

//*************************************************************
// Function prototype
//*************************************************************
static void         updateDisplayTask           (void* args);
static void         updateDisplayButtonsTask    (void* args);
static void         updateDisplay               (void);
static void         updateSelectedScreen        (void);
static void         updateScreenIndex           (void);
static void         updateScreen                (void);
static void         OLEDUpdateABSScreen         (void);
static void         OLEDUpdatePWMScreen         (void);
static void         OLEDDrawTemplate            (void);
static void         OLEDDrawABSTemplate         (void);
static void         OLEDDrawPWMTemplate         (void);
static void         OLEDDrawTask                (void* args);
static void         OLEDDraw                    (char str[], int col, int row);
static void         clearScreen                 (void);
static char*        getABSStateName             (enum absStates state);
static PWMSignal_t  getSelectedPWM              (void);

//*************************************************************
// FreeRTOS Handles
//*************************************************************
QueueHandle_t       OLEDDrawQueue;
SemaphoreHandle_t   OLEDDrawMutex;

//*****************************************************************************
// Global variables
//*****************************************************************************
static Screen_t     screen; 
static int          screenIndex = 0;

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

    OLEDDrawQueue = xQueueCreate(10, sizeof(OLEDDraw_t));

    OLEDDrawMutex = xSemaphoreCreateMutex();

    xTaskCreate(&updateDisplayTask, "updateDisplay", 256, NULL, UPDATE_DISPLAY_TASK_PRIORITY, NULL);
    xTaskCreate(&updateDisplayButtonsTask, "updateButtons", 256, NULL, UPDATE_BUTTONS_TASK_PRIORITY, NULL);
    xTaskCreate(&OLEDDrawTask, "OLEDDraw", 256, NULL, OLED_DRAW_TASK_PRIORITY, NULL);

    OLEDDrawTemplate ();
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

    const TickType_t xDelay = UPDATE_DISPLAY_TASK_RATE / portTICK_PERIOD_MS;

    while (true)
    {
        updateDisplay();

        vTaskDelay(xDelay);
    }   
}

/**
 * @brief Task to regularly update any button presses
 * 
 * @return None
 */
static void
updateDisplayButtonsTask (void* args)
{
    (void)args;

    const TickType_t xDelay = UPDATE_BUTTONS_TASK_RATE / portTICK_PERIOD_MS;

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
    if (checkButton(UP) == RELEASED)
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
    else if (checkButton(DOWN) == RELEASED)
    {
        if (screenIndex == 0)
        {
            screenIndex = getCountPWMInputs();
        }
        else
        {
            screenIndex--;
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
    static enum screenType prevScreenType;
    if (screen.type != prevScreenType)
    {
        clearScreen ();
        OLEDDrawTemplate ();
        prevScreenType = screen.type;
    }

    switch (screen.type)
    {
        case ABS_INFO:
            OLEDUpdateABSScreen ();
            break;
        case PWM_INFO:
            OLEDUpdatePWMScreen ();
            break;
    } 
}

/**
 * @brief Draws the screen template based on the selected screen
 * 
 * @return None
 */
static void
OLEDDrawTemplate (void)
{
    switch (screen.type)
    {
        case ABS_INFO:
            OLEDDrawABSTemplate ();
            break;
        case PWM_INFO:
            OLEDDrawPWMTemplate ();
            break;
    } 
}

/**
 * @brief Draws the template for the ABS information display
 * 
 * @return None
 */
static void
OLEDDrawABSTemplate (void)
{
    OLEDDraw_t lineToDraw;
	
    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "----Car Info----");
    lineToDraw.row = 0;
    lineToDraw.col = 0;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);

    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "Turn Angle:     ");
    lineToDraw.row = 1;
    lineToDraw.col = 0;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);

    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "ABS State:      ");
    lineToDraw.row = 2;
    lineToDraw.col = 0;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);

    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "ABS Duty:      %%");
    lineToDraw.row = 3;
    lineToDraw.col = 0;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);
}

/**
 * @brief Draws the template for the PWM information display
 * 
 * @return None
 */
static void
OLEDDrawPWMTemplate (void)
{
    OLEDDraw_t lineToDraw;
	
    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "----PWM Info----");
    lineToDraw.row = 0;
    lineToDraw.col = 0;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);

    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "ID:             ");
    lineToDraw.row = 1;
    lineToDraw.col = 0;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);

    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "Freq:           ");
    lineToDraw.row = 2;
    lineToDraw.col = 0;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0); 

    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "Duty:           ");
    lineToDraw.row = 3;
    lineToDraw.col = 0;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);    
}


/**
 * @brief Update only the variable information for the ABS screen
 * 
 * @return None
 */
static void
OLEDUpdateABSScreen (void)
{
    OLEDDraw_t lineToDraw;
	
    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "%d", getSteeringAngle());
    lineToDraw.row = 1;
    lineToDraw.col = OLED_CHAR_WIDTH - 1 - strlen(lineToDraw.str);
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);

    // Adds whitespace when changing from long IDs to shorter IDs
    int whiteSpaceLen = MAX_ANGLE_LEN - strlen(lineToDraw.str);
    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "                ");
    lineToDraw.str[whiteSpaceLen - 1] = 0;
    lineToDraw.col = OLED_CHAR_WIDTH - MAX_ANGLE_LEN;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);   

    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "%s", getABSStateName(screen.content.absScreen.absState));
    lineToDraw.row = 2;
    lineToDraw.col = OLED_CHAR_WIDTH - 1 - strlen(lineToDraw.str);
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);

    // Adds whitespace when changing from long IDs to shorter IDs
    whiteSpaceLen = MAX_STATE_LEN - strlen(lineToDraw.str);
    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "                ");
    lineToDraw.str[whiteSpaceLen - 1] = 0;
    lineToDraw.col = OLED_CHAR_WIDTH - MAX_STATE_LEN;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);   

    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "%2d%%", screen.content.absScreen.duty);
    lineToDraw.row = 3;
    lineToDraw.col = OLED_CHAR_WIDTH - 1 - strlen(lineToDraw.str);
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);
}

/**
 * @brief Update only the variable information for the PWM screen
 * 
 * @return None
 */
static void
OLEDUpdatePWMScreen (void)
{
    OLEDDraw_t lineToDraw;

    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "%s", screen.content.pwmScreen.pwmSignal.id);
    lineToDraw.row = 1;
    if (strlen(lineToDraw.str) > MAX_ID_LEN) lineToDraw.str[MAX_ID_LEN] = 0; // Prevent overflow of long IDs
    lineToDraw.col = OLED_CHAR_WIDTH - 1 - strlen(lineToDraw.str);
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);   

    // Adds whitespace when changing from long IDs to shorter IDs
    int whiteSpaceLen = MAX_ID_LEN - strlen(lineToDraw.str);
    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "                ");
    lineToDraw.str[whiteSpaceLen - 1] = 0;
    lineToDraw.col = OLED_CHAR_WIDTH - MAX_ID_LEN;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);   

    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "%3d Hz", screen.content.pwmScreen.pwmSignal.frequency);
    lineToDraw.row = 2;
    lineToDraw.col = OLED_CHAR_WIDTH - 1 - strlen(lineToDraw.str);
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0); 

    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), " %2d%%", screen.content.pwmScreen.pwmSignal.duty);
    lineToDraw.row = 3;
    lineToDraw.col = OLED_CHAR_WIDTH - 1 - strlen(lineToDraw.str);
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0);    
}


/**
 * @brief  Task to draw to the OLED display
 * 
 * @return None
 */
static void 
OLEDDrawTask(void* args)
{
    (void)args;

    OLEDDraw_t lineToDraw;
    while (true) 
    {
        if (xQueueReceive(OLEDDrawQueue, &lineToDraw, portMAX_DELAY) == pdPASS)
        {
            OLEDDraw(lineToDraw.str, lineToDraw.col, lineToDraw.row);
        }
    }   
}

/**
 * @brief Writes to the OLED library
 * 
 * @param str - The string to be drawn
 * @param col - Column to start string
 * @param row - Row to write string on
 */
static void
OLEDDraw(char str[], int col, int row)
{
    if (xSemaphoreTake(OLEDDrawMutex, (TickType_t) 10) == pdTRUE)
    {
        OLEDStringDraw (str, col, row);

        xSemaphoreGive(OLEDDrawMutex);
    }

}

/**
 * @brief Clears the OLED screen
 *  
 * @return None
 */
static void
clearScreen (void)
{
    OLEDDraw_t lineToDraw;

    lineToDraw.col = 0;
	
    usnprintf (lineToDraw.str, sizeof(lineToDraw.str), "                ");
    lineToDraw.row = 0;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0); 
    lineToDraw.row = 1;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0); 
    lineToDraw.row = 2;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0); 
    lineToDraw.row = 3;
    xQueueSendToBack(OLEDDrawQueue, &lineToDraw, 0); 
}


/**
 * @brief Converts the ABS state to a string for printing
 * 
 * @param state - ABS state
 * @return char* - String
 */
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