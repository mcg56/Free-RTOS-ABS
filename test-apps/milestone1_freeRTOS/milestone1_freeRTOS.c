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
// freeRTOS handles
//*****************************************************************************
TaskHandle_t pulseABSHandle;
TaskHandle_t blinkHandle;
TaskHandle_t updateABSHandle;
TaskHandle_t updateStatusButtonHandle;

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
        }

        vTaskDelay(xDelay);
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

    // Initialisation is complete, so turn on the output.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

    xTaskCreate(&blink, "blink", 256, NULL, 0, &blinkHandle);
    xTaskCreate(&updateStatusButton, "updateStatusButton", 256, NULL, 0, &updateStatusButtonHandle);
    xTaskCreate(&updateABS, "updateABS", 256, NULL, 0, &updateABSHandle);
    xTaskCreate(&pulseABS, "pulseABS", 256, NULL, 0, &pulseABSHandle);

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
