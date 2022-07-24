#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

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
// freeRTOS handles
//*****************************************************************************
TaskHandle_t updateOLEDHandle;
TaskHandle_t pulseABSHandle;
TaskHandle_t blinkHandle;
TaskHandle_t updateABSHandle;
TaskHandle_t updateStatusButtonHandle;
extern enum absStates absState;


void blink(void* args) {
    (void)args; // unused

    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

    while (true) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));

        vTaskDelay(xDelay);
        // configASSERT(wake_time < 1000);  // Runs vAssertCalled() if false
    }
}

void updateStatusButton (void* args)
{
    (void)args;
    const TickType_t xDelay = 50 / portTICK_PERIOD_MS;

    while (true) 
    {
        updateButtons();

        if (checkButton(UP) == PUSHED)
        {
            toggleABSState();

            // Tell the abs controller to update its status
            xTaskNotifyGiveIndexed( updateABSHandle, 0 );

            // Tell the OLED update task to write a new line with new abs state
            xTaskNotifyGiveIndexed( updateOLEDHandle, 0 );
        }

        vTaskDelay(xDelay);
    }
}

void updateOLED(void* args)
{
    (void)args;
    //const TickType_t xDelay = 50 / portTICK_PERIOD_MS;

    while(true)
    {
        // Wait until a task has notified it to run, when a new message is to be written
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        char string[17]; // Display fits 16 characters wide.
        OLEDStringDraw ("                ", 0, 0); //Clear line

        if (absState)
        {
            OLEDStringDraw ("ABS PWM", 0, 0);
        } else
        {
            OLEDStringDraw ("Continuous PWM", 0, 0);       
        }
        /*
        usnprintf (string, sizeof(string), "ABS state: %d", absState);
        OLEDStringDraw (string, 0, 0);*/
    }
}

int main(void) {
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM

    initialisePWM ();
    initButtons ();
    OLEDInitialise ();
    OLEDStringDraw ("Continuous PWM", 0, 0);

    // Initialisation is complete, so turn on the output.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

    xTaskCreate(&blink, "blink", 256, NULL, 0, &blinkHandle);
    xTaskCreate(&updateStatusButton, "updateStatusButton", 256, NULL, 0, &updateStatusButtonHandle);
    xTaskCreate(&updateABS, "updateABS", 256, NULL, 0, &updateABSHandle);
    xTaskCreate(&pulseABS, "pulseABS", 256, NULL, 0, &pulseABSHandle);
    xTaskCreate(&updateOLED, "updateOLED", 256, NULL, 0, &updateOLEDHandle);

    vTaskSuspend(pulseABSHandle);
    setPWM(500, 30);

    vTaskStartScheduler();

    return 0;
}


// This is an error handling function called when FreeRTOS asserts.
// This should be used for debugging purposes
void vAssertCalled( const char * pcFile, unsigned long ulLine ) {
    (void)pcFile; // unused
    (void)ulLine; // unused
    while (true) ;
}
