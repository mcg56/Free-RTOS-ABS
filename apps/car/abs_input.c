#include <FreeRTOS.h>
#include <semphr.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "driverlib/timer.h"
#include <inc/hw_memmap.h>
#include <driverlib/gpio.h>
#include "SW-TM4C-2.2.0.295/inc/hw_timer.h"
#include "abs_input.h"
#include "libs/lib_pwm/pwm_input.h"
#include <driverlib/sysctl.h>
#include <inc/hw_types.h>
#include "car_state.h"

#define ABS_DELAY_TIMER_PERIPHERAL  SYSCTL_PERIPH_TIMER4
#define ABS_DELAY_TIMER_BASE        TIMER4_BASE
#define ABS_DELAY_TIMER             TIMER_A
#define ABS_DELAY_TIMER_CONFIG      TIMER_CFG_PERIODIC_UP

TaskHandle_t processABSInputSignalTaskHandle;

void
ABSDelayTimerInit()	
	{
	/* Configure Timer 1. 
	*/
	SysCtlPeripheralEnable(ABS_DELAY_TIMER_PERIPHERAL);

	TimerConfigure(ABS_DELAY_TIMER_BASE, ABS_DELAY_TIMER_CONFIG);
	TimerEnable(ABS_DELAY_TIMER_BASE, ABS_DELAY_TIMER);
}



void processABSInputSignalTask(void* args)
{
    (void)args; // unused
    //Local variables
    static bool ABSState = false;
    static uint8_t brakeOnCount = 0;
    PWMSignal_t pwmDetails;
    while(1)
    {
        TickType_t wake_time = xTaskGetTickCount();
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));
        uint8_t currentABSBrakeDuty;
        HWREG(ABS_DELAY_TIMER_BASE + TIMER_O_TAV) = 0; // Reset delay timer
        bool highEdgeFound = false;

        // for 1/500s read the input pin level, if there is no high then ABS is on
        // sysclock is 80 mHz
        while(TimerValueGet(ABS_DELAY_TIMER_BASE, ABS_DELAY_TIMER) < 80000000/500)
        {
            if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0)) // Found high edge, use brake duty accordingly
            {
                updatePWMInput(ABSPWM_ID);
                pwmDetails = getPWMInputSignal(ABSPWM_ID);
                brakeOnCount++;
                currentABSBrakeDuty = pwmDetails.duty;
                if (brakeOnCount >= 4) // 4 kinda arbitrary
                {
                    ABSState = false;
                }
                highEdgeFound = true;
                //OLEDStringDraw ("Off", 0, 3);
                break;
            }
        }
        
        // No high edge found in 1/500 s, ABS must be toggled on. 0 duty
        if (!highEdgeFound)
        {
            //OLEDStringDraw ("On ", 0, 3);
            ABSState = true;
            brakeOnCount = 0;
            currentABSBrakeDuty = 0;
            //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));
        }

        // Wait until we can take the mutex to be able to use car state shared resource
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        setABSBrakePressureDuty(currentABSBrakeDuty);
        setABSState(ABSState);
        xSemaphoreGive(carStateMutex);

        vTaskDelayUntil(&wake_time, 50);
    }
}
