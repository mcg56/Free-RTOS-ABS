#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include "driverlib/systick.h"


#include <FreeRTOS.h>
#include <task.h>

#include "libs/lib_buttons/ap_buttons.h"
#include "libs/lib_pwm/ap_pwm.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include "libs/lib_adc/ap_adc.h"
#include "libs/lib_system/ap_system.h"
#include "stdlib.h"
#include "utils/ustdlib.h"
#include "driverlib/pwm.h"

TaskHandle_t updateOLEDHandle;
TaskHandle_t blinkHandle;
TaskHandle_t updateStatusButtonHandle;
TaskHandle_t updateADCHandle;

int32_t steering_value;


void blink(void* args) {
    (void)args; // unused

    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

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

            // Tell the OLED update task to write a new line with new abs state
            xTaskNotifyGiveIndexed( updateOLEDHandle, 0 );
        }

        vTaskDelay(xDelay);
    }
}

/**
 * @brief Update the OLED screen. Currently only displays the ABS state
 * @param args
 * @return No return
 */
void updateOLED(void* args)
{
    (void)args;
    const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
    uint16_t meanVal;
    while(true)
    {
        // Wait until a task has notified it to run, when a new message is to be written
        //ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));

        char string[17]; // Display fits 16 characters wide.
        OLEDStringDraw ("                ", 0, 0); //Clear line
        
        OLEDStringDraw ("ADC From Pot", 0, 0);

        meanVal = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
        
        // Form a new string for the line.  The maximum width specified for the
        //  number field ensures it is displayed right justified.
        usnprintf (string, sizeof(string), "Mean ADC = %4d", meanVal);
        // Update line on display.
        OLEDStringDraw (string, 0, 1);
        vTaskDelay(xDelay);
    }
}

/**
 * @brief Update the OLED screen. Currently only displays the ABS state
 * @param args
 * @return No return
 */
void updateADC(void* args)
{
    (void)args;
    const TickType_t xDelay = 5 / portTICK_PERIOD_MS;
    uint16_t i;
	
    while(true)
    {
       
        // Wait until a task has notified it to run, when a new message is to be written
        //ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        //
		// Background task: calculate the (approximate) mean of the values in the
		// circular buffer and display it, together with the sample number.
		sum = 0;
        updateSteering();
		for (i = 0; i < BUF_SIZE; i++)
			sum = sum + readCircBuf (&g_inBuffer);
		
        
		vTaskDelay(xDelay);
    }
}

int main(void) {

    initClock ();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
     // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM

    
	
    initialisePWM ();
    initButtons ();
    initDisplay ();
    initADC ();
	initCircBuf (&g_inBuffer, BUF_SIZE);


    OLEDStringDraw ("Initialising       ", 0, 0); //Clear line


    //xTaskCreate(&blink, "blink", 256, NULL, 0, &blinkHandle);
    xTaskCreate(&updateStatusButton, "updateStatusButton", 256, NULL, 0, &updateStatusButtonHandle);
    xTaskCreate(&updateOLED, "updateOLED", 256, NULL, 0, &updateOLEDHandle);
    xTaskCreate(&updateADC, "updateADC", 256, NULL, 0, &updateADCHandle);


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

