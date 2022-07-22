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

enum absStates {ABS_OFF = 0, ABS_ON};

//*****************************************************************************
// Global variables
//*****************************************************************************
static enum absStates absState = ABS_OFF;

//*****************************************************************************
// Task handles
//*****************************************************************************
TaskHandle_t pulseABSHandle;

void
initDisplay (void)
{
  // intialise the Orbit OLED display
	OLEDInitialise ();
}


//*****************************************************************************
// Function to display the mean interval in usec
//*****************************************************************************
void
displayButtonState (char *butStr, char *stateStr, uint8_t numPushes, uint8_t charLine)
{
    char string[17]; // Display fits 16 characters wide.
	
    OLEDStringDraw ("                ", 0, charLine);
    usnprintf (string, sizeof(string), "%s - %s %2d", butStr, stateStr, numPushes);
    OLEDStringDraw (string, 0, charLine);
}


void blink(void* args) {
    (void)args; // unused

    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

    while (true) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));

        vTaskDelay(xDelay);
        // configASSERT(wake_time < 1000);  // Runs vAssertCalled() if false
    }
}

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

void updateStatus (void* args)
{
    (void)args;
    const TickType_t xDelay = 50 / portTICK_PERIOD_MS;

    while (true) 
    {
        updateButtons();

        if (checkButton(UP) == PUSHED)
        {
            displayButtonState ("UP   ", "PUSH", (int)absState, 0);
            toggleABSState();
        }

        // toggleABSState();

        vTaskDelay(xDelay);
        // configASSERT(wake_time < 1000);  // Runs vAssertCalled() if false
    }
}

void updateABS (void* args)
{
    (void)args; 

    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

    while (true)
    {
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

    static pulseOn = true;

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



int main(void) {
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM

    initialisePWM ();
    initDisplay ();
    initButtons ();

    // Initialisation is complete, so turn on the output.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

    xTaskCreate(&blink, "blink", 256, NULL, 0, NULL);
    xTaskCreate(&updateStatus, "updateStatus", 256, NULL, 0, NULL);
    xTaskCreate(&updateABS, "updateABS", 256, NULL, 0, NULL);
    xTaskCreate(&pulseABS, "pulseABS", 256, NULL, 0, &pulseABSHandle);

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
