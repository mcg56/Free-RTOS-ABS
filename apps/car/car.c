/**********************************************************
car.c - Main controlling file for the car simulator

A.J Eason A. Musalov
Last modified:  18/08/22
 **********************************************************/

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
#include "libs/lib_buttons/buttons.h"
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
#include "abs_input.h"


int main(void) {
    // Set up system clock
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    // Initialise the modules of this program. This sets up peripherals, 
    // creates tasks, queues, semaphores etc ready for normal operation.
    initUserInterface();
    initCarPwm();
    initCarState();
    initABSInput();
    initWheels();    

    // Start scheduler
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

