/**********************************************************
abs_input.c

Module based around reading and processing the ABS input
signal from the ABS controller. Contains tasks and functions
to detect ABS using a timer and read the duty.

A.J Eason A. Musalov
Last modified:  18/08/22
***********************************************************/

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

//*************************************************************
// Constant Definitions
//*************************************************************
#define ABS_DELAY_TIMER_PERIPHERAL  SYSCTL_PERIPH_TIMER4
#define ABS_DELAY_TIMER_BASE        TIMER4_BASE
#define ABS_DELAY_TIMER             TIMER_A
#define ABS_DELAY_TIMER_CONFIG      TIMER_CFG_PERIODIC_UP

#define ABS_INPUT_TASK_STACK_SIZE   256
#define ABS_INPUT_TASK_PRIORITY     3
#define ABS_INPUT_PWM_HZ            500
#define ABS_OFF_REQUIRED_READS      4


//*************************************************************
// Private function prototypes
//*************************************************************

/**
 * @brief Initialises timer used for 1/500 s delay when detecting ABS
 * @return None
 */
void ABSDelayTimerInit();

/**
 * @brief Task to process the ABS input. Looks for a high edge on the input
 * PWM pin for s ms (1/500 Hz). If this is found, reads the incoming duty and
 * stores it in car state. If no edge is found, it knows ABS is on and is in
 * the brake off toggle so sets abs brake duty to zero in car state. Changes 
 * overall ABS state when 4 consecutive task runs return a duty. Runs at
 * 20 Hz so samples input PWM every 50 ms. Runs at highest task priority.
 * @return None
 */
void processABSInputSignalTask(void* args);

//*****************************************************************************
// Global variables
//*****************************************************************************
TaskHandle_t processABSInputSignalTaskHandle;


//*****************************************************************************
// Functions
//*****************************************************************************


void ABSDelayTimerInit(void)	
{
    // Enable peripheral and configure timer
	SysCtlPeripheralEnable(ABS_DELAY_TIMER_PERIPHERAL);
	TimerConfigure(ABS_DELAY_TIMER_BASE, ABS_DELAY_TIMER_CONFIG);
	TimerEnable(ABS_DELAY_TIMER_BASE, ABS_DELAY_TIMER);
}


void initABSInput(void)
{
    //Create timer and task for reading ABS brake input
    ABSDelayTimerInit();
    xTaskCreate(&processABSInputSignalTask, "ABS input task", ABS_INPUT_TASK_STACK_SIZE, NULL, ABS_INPUT_TASK_PRIORITY, &processABSInputSignalTaskHandle);
}


void processABSInputSignalTask(void* args)
{
    //Local variables
    static bool ABSState = false;
    static uint8_t brakeOnCount = 0;
    const float taskPeriodms = 50; // [ms]
    PWMSignal_t pwmDetails;
    (void)args; // Unused
    while(1)
    {
        TickType_t wake_time = xTaskGetTickCount(); // Get current time for task period delay
        
        uint8_t currentABSBrakeDuty;
        bool highEdgeFound = false;

        // for 1/500s read the input pin level, if there is no high then ABS is on
        // sysclock is 80 mHz
        HWREG(ABS_DELAY_TIMER_BASE + TIMER_O_TAV) = 0; // Reset delay timer
        while(TimerValueGet(ABS_DELAY_TIMER_BASE, ABS_DELAY_TIMER) < SysCtlClockGet()/ABS_INPUT_PWM_HZ)
        {
            if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0)) // Found high edge, find and use brake duty accordingly
            {
                highEdgeFound = true;
                // Read the duty and frequency
                updatePWMInput(ABSPWM_ID);
                pwmDetails = getPWMInputSignal(ABSPWM_ID);
                currentABSBrakeDuty = pwmDetails.duty;

                //Update count to determine if ABS is off yet
                brakeOnCount++; 
                if (brakeOnCount >= ABS_OFF_REQUIRED_READS)
                {
                    ABSState = false;
                }
                break;
            }
        }
        
        // No high edge found in 1/500 s, ABS must be toggled on. 0 duty
        if (!highEdgeFound)
        {
            ABSState = true;
            brakeOnCount = 0;
            currentABSBrakeDuty = 0;
        }

        // Wait until we can take the mutex to be able to use car state shared resource
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        // Change car state values
        setABSBrakePressureDuty(currentABSBrakeDuty);
        setABSState(ABSState);
        xSemaphoreGive(carStateMutex);

        //Delay so task runs at 20 Hz
        vTaskDelayUntil(&wake_time, taskPeriodms);
    }
}
