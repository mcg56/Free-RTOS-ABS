/**********************************************************
car_pwm.c

Module concerned with the car PWM outputs. Contains structs
of the PWM signal hardware details and initialisation 
functions.

A.J Eason A. Musalov
Last modified:  19/08/22
***********************************************************/

#include "car_pwm.h"
#include "libs/lib_pwm/pwm_output.h"
#include "libs/lib_pwm/pwm_input.h"

//*************************************************************
// Private Constant Definitions
//*************************************************************
// Left front wheel output PWM hardware details
#define PWM_LF_BASE	        PWM0_BASE
#define PWM_LF_GEN          PWM_GEN_1
#define PWM_LF_OUTNUM       PWM_OUT_3
#define PWM_LF_OUTBIT       PWM_OUT_3_BIT
#define PWM_LF_PERIPH_PWM	SYSCTL_PERIPH_PWM0
#define PWM_LF_PERIPH_GPIO  SYSCTL_PERIPH_GPIOB
#define PWM_LF_GPIO_BASE    GPIO_PORTB_BASE
#define PWM_LF_GPIO_CONFIG  GPIO_PB5_M0PWM3
#define PWM_LF_GPIO_PIN     GPIO_PIN_5

// Left rear wheel output PWM hardware details
#define PWM_LR_BASE	        PWM0_BASE
#define PWM_LR_GEN          PWM_GEN_2
#define PWM_LR_OUTNUM       PWM_OUT_4
#define PWM_LR_OUTBIT       PWM_OUT_4_BIT
#define PWM_LR_PERIPH_PWM	SYSCTL_PERIPH_PWM0
#define PWM_LR_PERIPH_GPIO  SYSCTL_PERIPH_GPIOE
#define PWM_LR_GPIO_BASE    GPIO_PORTE_BASE
#define PWM_LR_GPIO_CONFIG  GPIO_PE4_M0PWM4
#define PWM_LR_GPIO_PIN     GPIO_PIN_4

//Right front wheel output PWM hardware details
#define PWM_RF_BASE	        PWM0_BASE
#define PWM_RF_GEN          PWM_GEN_3
#define PWM_RF_OUTNUM       PWM_OUT_7
#define PWM_RF_OUTBIT       PWM_OUT_7_BIT
#define PWM_RF_PERIPH_PWM	SYSCTL_PERIPH_PWM0
#define PWM_RF_PERIPH_GPIO  SYSCTL_PERIPH_GPIOC
#define PWM_RF_GPIO_BASE    GPIO_PORTC_BASE
#define PWM_RF_GPIO_CONFIG  GPIO_PC5_M0PWM7
#define PWM_RF_GPIO_PIN     GPIO_PIN_5

//Right rear wheel output PWM hardware details
#define PWM_RR_BASE	        PWM1_BASE
#define PWM_RR_GEN          PWM_GEN_1
#define PWM_RR_OUTNUM       PWM_OUT_2
#define PWM_RR_OUTBIT       PWM_OUT_2_BIT
#define PWM_RR_PERIPH_PWM	SYSCTL_PERIPH_PWM1
#define PWM_RR_PERIPH_GPIO  SYSCTL_PERIPH_GPIOA
#define PWM_RR_GPIO_BASE    GPIO_PORTA_BASE
#define PWM_RR_GPIO_CONFIG  GPIO_PA6_M1PWM2
#define PWM_RR_GPIO_PIN     GPIO_PIN_6

//Brake level output PWM hardware details
#define PWM_BRAKE_BASE	        PWM1_BASE
#define PWM_BRAKE_GEN           PWM_GEN_3
#define PWM_BRAKE_OUTNUM        PWM_OUT_6
#define PWM_BRAKE_OUTBIT        PWM_OUT_6_BIT
#define PWM_BRAKE_PERIPH_PWM	SYSCTL_PERIPH_PWM1
#define PWM_BRAKE_PERIPH_GPIO   SYSCTL_PERIPH_GPIOF
#define PWM_BRAKE_GPIO_BASE     GPIO_PORTF_BASE
#define PWM_BRAKE_GPIO_CONFIG   GPIO_PF2_M1PWM6
#define PWM_BRAKE_GPIO_PIN      GPIO_PIN_2
#define PWM_BRAKE_START_DUTY    5

//Steering wheel output PWM hardware details
#define PWM_STEER_BASE	        PWM1_BASE
#define PWM_STEER_GEN           PWM_GEN_3
#define PWM_STEER_OUTNUM        PWM_OUT_7
#define PWM_STEER_OUTBIT        PWM_OUT_7_BIT
#define PWM_STEER_PERIPH_PWM	SYSCTL_PERIPH_PWM1
#define PWM_STEER_PERIPH_GPIO   SYSCTL_PERIPH_GPIOF
#define PWM_STEER_GPIO_BASE     GPIO_PORTF_BASE
#define PWM_STEER_GPIO_CONFIG   GPIO_PF3_M1PWM7
#define PWM_STEER_GPIO_PIN      GPIO_PIN_3
#define PWM_STEERING_START_DUTY 50

//*****************************************************************************
// Global variables
//*****************************************************************************
PWMOutputHardwareDetails_t PWMHardwareDetailsLF = {PWM_LF_BASE, PWM_LF_GEN, PWM_LF_OUTNUM, PWM_LF_OUTBIT, PWM_LF_PERIPH_PWM, PWM_LF_PERIPH_GPIO, PWM_LF_GPIO_BASE, PWM_LF_GPIO_CONFIG, PWM_LF_GPIO_PIN};
PWMOutputHardwareDetails_t PWMHardwareDetailsLR = {PWM_LR_BASE, PWM_LR_GEN, PWM_LR_OUTNUM, PWM_LR_OUTBIT, PWM_LR_PERIPH_PWM, PWM_LR_PERIPH_GPIO, PWM_LR_GPIO_BASE, PWM_LR_GPIO_CONFIG, PWM_LR_GPIO_PIN};
PWMOutputHardwareDetails_t PWMHardwareDetailsRF = {PWM_RF_BASE, PWM_RF_GEN, PWM_RF_OUTNUM, PWM_RF_OUTBIT, PWM_RF_PERIPH_PWM, PWM_RF_PERIPH_GPIO, PWM_RF_GPIO_BASE, PWM_RF_GPIO_CONFIG, PWM_RF_GPIO_PIN};
PWMOutputHardwareDetails_t PWMHardwareDetailsRR = {PWM_RR_BASE, PWM_RR_GEN, PWM_RR_OUTNUM, PWM_RR_OUTBIT, PWM_RR_PERIPH_PWM, PWM_RR_PERIPH_GPIO, PWM_RR_GPIO_BASE, PWM_RR_GPIO_CONFIG, PWM_RR_GPIO_PIN};
PWMOutputHardwareDetails_t PWMHardwareDetailsBrake = {PWM_BRAKE_BASE, PWM_BRAKE_GEN, PWM_BRAKE_OUTNUM, PWM_BRAKE_OUTBIT, PWM_BRAKE_PERIPH_PWM, PWM_BRAKE_PERIPH_GPIO, PWM_BRAKE_GPIO_BASE, PWM_BRAKE_GPIO_CONFIG, PWM_BRAKE_GPIO_PIN};
PWMOutputHardwareDetails_t PWMHardwareDetailsSteering = {PWM_STEER_BASE, PWM_STEER_GEN, PWM_STEER_OUTNUM, PWM_STEER_OUTBIT, PWM_STEER_PERIPH_PWM, PWM_STEER_PERIPH_GPIO, PWM_STEER_GPIO_BASE, PWM_STEER_GPIO_CONFIG, PWM_STEER_GPIO_PIN};

//*****************************************************************************
// Functions
//*****************************************************************************

void initCarPwm(void)
{
    createPWMQueue();

    // Init PWM inputs and outputs
    initPWMInputManager (CAR_PWM_MIN_FREQ);
    initializeCarPWMOutputs();

    // Create and register input ABS pwm
    PWMSignal_t ABSPWM = {.id = ABSPWM_ID, .gpioPort = GPIO_PORTB_BASE, .gpioPin = GPIO_PIN_0};
    registerPWMSignal(ABSPWM);

    // Create PWM outut task
    xTaskCreate(&updatePWMOutputsTask, "update PWM", 256, NULL, 2, &updatePWMOutputsTaskHandle);

}

void initializeCarPWMOutputs(void)
{
    initializePWMGeneral(PWMHardwareDetailsLF, PWM_WHEEL_START_HZ, PWM_WHEEL_FIXED_DUTY);
    initializePWMGeneral(PWMHardwareDetailsLR, PWM_WHEEL_START_HZ, PWM_WHEEL_FIXED_DUTY);
    initializePWMGeneral(PWMHardwareDetailsRF, PWM_WHEEL_START_HZ, PWM_WHEEL_FIXED_DUTY);
    initializePWMGeneral(PWMHardwareDetailsRR, PWM_WHEEL_START_HZ, PWM_WHEEL_FIXED_DUTY);
    initializePWMGeneral(PWMHardwareDetailsBrake, PWM_BRAKE_FIXED_HZ, PWM_BRAKE_START_DUTY); // Need to check the nominal freq/duty of this
    initializePWMGeneral(PWMHardwareDetailsSteering, PWM_STEERING_FIXED_HZ, PWM_STEERING_START_DUTY); // Need to check the nominal freq/duty of this
    
    // Initialisation is complete, so turn on the pwm output.
    PWMOutputState(PWMHardwareDetailsLF.base, PWMHardwareDetailsLF.outbit, true);
    PWMOutputState(PWMHardwareDetailsLR.base, PWMHardwareDetailsLR.outbit, true);
    PWMOutputState(PWMHardwareDetailsRF.base, PWMHardwareDetailsRF.outbit, true);
    PWMOutputState(PWMHardwareDetailsRR.base, PWMHardwareDetailsRR.outbit, true);
    PWMOutputState(PWMHardwareDetailsBrake.base, PWMHardwareDetailsBrake.outbit, true);
    PWMOutputState(PWMHardwareDetailsSteering.base, PWMHardwareDetailsSteering.outbit, true);
}