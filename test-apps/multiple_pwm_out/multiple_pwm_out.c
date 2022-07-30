#include <stdint.h>
#include <stdbool.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>

#include <FreeRTOS.h>
#include <task.h>
#include "stdlib.h"
#include "utils/ustdlib.h"
#include "driverlib/pwm.h"
#include "ap_buttons.h"
#include "pwm_manager.h"
#include "ap_uart.h"

// LOTS OF THIS IS UNEEDED AND CAN BE DELETED
// PWM configuration
#define PWM_START_RATE_HZ  250
#define PWM_RATE_STEP_HZ   50
#define PWM_RATE_MIN_HZ    50
#define PWM_RATE_MAX_HZ    400
#define PWM_FIXED_DUTY     40
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_16
#define PWM_DIVIDER        16

// I WANT TO DELETE THE BELOW BLOCK SOON
//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE	     PWM1_BASE
#define PWM_MAIN_GEN         PWM_GEN_0
#define PWM_MAIN_OUTNUM      PWM_OUT_1
#define PWM_MAIN_OUTBIT      PWM_OUT_1_BIT
#define PWM_MAIN_PERIPH_PWM	 SYSCTL_PERIPH_PWM1
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOD
#define PWM_MAIN_GPIO_BASE   GPIO_PORTD_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PD1_M1PWM1
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_1
#define WHEEL_FIXED_DUTY     50

#define PWM_MAIN_BASE2	     PWM0_BASE
#define PWM_MAIN_GEN2        PWM_GEN_3
#define PWM_MAIN_OUTNUM2      PWM_OUT_6
#define PWM_MAIN_OUTBIT2      PWM_OUT_6_BIT
#define PWM_MAIN_PERIPH_PWM2	 SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO2 SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE2   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG2 GPIO_PC4_M0PWM6
#define PWM_MAIN_GPIO_PIN2    GPIO_PIN_4
#define WHEEL_FIXED_DUTY2     50


/*********************************************************
 * initialisePWM
 * M0PWM7 (J4-05, PC5) is used
 *********************************************************/
void
initialisePWM1 (void)
{
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);  	// turn on main pwm peripheral
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);	// turn on GPIO for PWM mode

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);			// config for GPIO_PC5_M0PWM7  (J4 pin 5)
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);  // use GPIO Pin 5

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    
    // Set the initial PWM parameters
    setPWM1 (PWM_START_RATE_HZ, PWM_FIXED_DUTY);	// start rate is 250 Hz, 67% duty cycle

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
}

/*********************************************************
 * initialisePWM
 * M0PWM7 (J4-05, PC5) is used
 *********************************************************/
void
initialisePWM2 (void)
{
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM2);  	// turn on main pwm peripheral
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO2);	// turn on GPIO for PWM mode

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG2);			// config for GPIO_PC5_M0PWM7  (J4 pin 5)
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE2, PWM_MAIN_GPIO_PIN2);  // use GPIO Pin 5

    PWMGenConfigure(PWM_MAIN_BASE2, PWM_MAIN_GEN2,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    
    // Set the initial PWM parameters
    setPWM2 (PWM_START_RATE_HZ, PWM_FIXED_DUTY);	// start rate is 250 Hz, 67% duty cycle

    PWMGenEnable(PWM_MAIN_BASE2, PWM_MAIN_GEN2);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE2, PWM_MAIN_OUTBIT2, false);
}

/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setPWM1 (uint32_t ui32Freq, uint32_t ui32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / ui32Freq;

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
        ui32Period * ui32Duty / 100);
}

/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setPWM2 (uint32_t ui32Freq, uint32_t ui32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / ui32Freq;

    PWMGenPeriodSet(PWM_MAIN_BASE2, PWM_MAIN_GEN2, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE2, PWM_MAIN_OUTNUM2,
        ui32Period * ui32Duty / 100);
}

int main(void) {

    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM

    initButtons();
    initPWMManager ();
    initialisePWM1();
    initialisePWM2();
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_MAIN_BASE2, PWM_MAIN_OUTBIT2, true);
    initialiseUSB_UART ();

    PWMSignal_t testPWM = {.id = "testPWM", .gpioPin = GPIO_PIN_0};
    trackPWMSignal(testPWM);


    setPWM1(100, 50);
    setPWM2(110, 30);

    char str[100];
    PWMSignal_t signal;
    UARTSend("\n\rWaiting for press...\r\n");
    while (true)
    {
        updateButtons();

        if (checkButton(LEFT) == PUSHED)
        {
            updateAllPWMInfo();
            
            // Details of first PWM
            signal = getPWMInputSignals("testPWM");
            sprintf(str, "Frequency = %ld Hz\r\n", signal.frequency);
            UARTSend(str);
            sprintf(str, "Duty : %ld\r\n", signal.duty);
            UARTSend(str);
        }
    }

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

