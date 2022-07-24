#include <stdint.h>
#include <stdbool.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "driverlib/pwm.h"

#include "libs/lib_buttons/ap_buttons.h"
#include "libs/lib_pwm/ap_pwm.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include "abs_control.h"

//*****************************************************************************
// Global variables
//*****************************************************************************
enum absStates absState = ABS_OFF;
extern TaskHandle_t pulseABSHandle;


/**
 * @brief Sets the ABS state
 * @param state The state to set the ABS to

 * @return No return
 */
void setABSState (enum absStates state)
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

void toggleABSState (void)
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

enum absStates getABSState (void)
{
    return absState;
}

void updateABS (void* args)
{
    (void)args; 
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

    while (true)
    {
        // Wait until a task has notified it to run
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

        switch (absState)
        {
            case ABS_ON:
                vTaskResume(pulseABSHandle);
                break;
            case ABS_OFF:
                vTaskSuspend(pulseABSHandle);
                setPWM(500, 30);
                break;
        }


        vTaskDelay(xDelay);
    }
}

void pulseABS (void* args)
{
    (void)args;

    static bool pulseOn = true;

    const TickType_t xDelay = 200 / portTICK_PERIOD_MS; 

    while (true)
    {
        if (pulseOn)
        {
            setPWM (500, 30);
            pulseOn = false;
        }
        else
        {
            setPWM (0, 0);
            pulseOn = true;
        }

        vTaskDelay(xDelay);
    }   
}