#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "SW-TM4C-2.2.0.295/inc/hw_timer.h"
#include "libs/lib_buttons/buttons.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include "stdlib.h"
#include "utils/ustdlib.h"
#include "driverlib/pwm.h"
#include "wheels.h"
#include "libs/lib_pwm/pwm_input.h"
#include "libs/lib_pwm/pwm_output.h"
#include "libs/lib_system/system.h"
#include "libs/lib_uart/uart.h"
#include "user_interface.h"
#include "car_state.h"
#include "car_pwm.h"

// TO DO: move into own timer module
#define ABS_TIMER_PERIPH        SYSCTL_PERIPH_TIMER3
#define ABS_TIMER_BASE          TIMER3_BASE
#define ABS_TIMER               TIMER_A
#define ABS_TIMER_CONFIG        TIMER_CFG_A_PERIODIC
#define ABS_TIMER_INT_FLAG      TIMER_TIMA_TIMEOUT
#define ABS_TIMER_DEFAULT_RATE  20 // [Hz]

//Task handles


TaskHandle_t processBrakeSignalTaskHandle;

/**
 * @brief Creates instances of all queues
 * @return None
 */
void createQueues(void)
{
    updatePWMQueue = xQueueCreate(10, sizeof(pwmOutputUpdate_t));
}

/**
 * @brief Creates instances of all semaphores/mutexs
 * @return None
 */
void createSempahores(void)
{
    carStateMutex = xSemaphoreCreateMutex();
}














#define DELAY_PERIPHERAL    SYSCTL_PERIPH_TIMER4
#define DELAY_TIMER_BASE    TIMER4_BASE
#define DELAY_TIMER         TIMER_A
#define DELAY_TIMER_CONFIG  TIMER_CFG_PERIODIC_UP

void
DelayTimerInit()	
	{
	/* Configure Timer 1. 
	*/
	SysCtlPeripheralEnable(DELAY_PERIPHERAL);

	TimerConfigure(DELAY_TIMER_BASE, DELAY_TIMER_CONFIG);
	TimerEnable(DELAY_TIMER_BASE, DELAY_TIMER);
}



void processBrakeSignalTask(void* args)
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
        HWREG(DELAY_TIMER_BASE + TIMER_O_TAV) = 0; // Reset delay timer
        bool highEdgeFound = false;

        // for 1/500s read the input pin level, if there is no high then ABS is on
        // sysclock is 80 mHz
        while(TimerValueGet(DELAY_TIMER_BASE, DELAY_TIMER) < 80000000/500)
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



int main(void) {
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    OLEDInitialise ();

    // Setup red LED on PF1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    initButtons();
    initPWMInputManager (CAR_PWM_MIN_FREQ);

    initialiseUSB_UART ();
    initializeCarPWMOutputs();

    // Create and register input ABS pwm
    PWMSignal_t ABSPWM = {.id = ABSPWM_ID, .gpioPort = GPIO_PORTB_BASE, .gpioPin = GPIO_PIN_0};
    registerPWMSignal(ABSPWM);

    createQueues();
    createSempahores();

    DelayTimerInit();

    xTaskCreate(&readUserInputsTask, "read inputs", 150, NULL, 0, &readInputsHandle);
    xTaskCreate(&updateWheelInfoTask, "update wheel info", 256, NULL, 2, &updateWheelInfoHandle);
    xTaskCreate(&updateUARTTask, "update UART", 256, NULL, 0, &updateUARTHandle);
    xTaskCreate(&updatePWMOutputsTask, "update PWM", 256, NULL, 2, &updatePWMOutputsTaskHandle);
    xTaskCreate(&processBrakeSignalTask, "dummy", 256, NULL, 3, &processBrakeSignalTaskHandle);
    xTaskCreate(&decelerationTask, "decelerationTask", 256, NULL, 1, &decelerationTaskHandle);
    vTaskSuspend(decelerationTaskHandle);

    // Tell the wheel update task to run, which fills out the wheels speeds with starting info
    xTaskNotifyGiveIndexed(updateWheelInfoHandle, 0);

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

