/*
 * pwm.h
 */

#ifndef PWM_OUTPUT_H_
#define PWM_OUTPUT_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <stdbool.h>


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
#define PWM_MAIN_BASE	     PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM	 SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5


#define PWM_WHEEL_FIXED_DUTY    50
#define PWM_WHEEL_START_HZ      0

// Left front wheel output PWM hardware details M0PWM0 (gen 0)
#define PWM_LF_BASE	        PWM0_BASE
#define PWM_LF_GEN          PWM_GEN_0
#define PWM_LF_OUTNUM       PWM_OUT_0
#define PWM_LF_OUTBIT       PWM_OUT_0_BIT
#define PWM_LF_PERIPH_PWM	SYSCTL_PERIPH_PWM0
#define PWM_LF_PERIPH_GPIO  SYSCTL_PERIPH_GPIOB
#define PWM_LF_GPIO_BASE    GPIO_PORTB_BASE
#define PWM_LF_GPIO_CONFIG  GPIO_PB6_M0PWM0
#define PWM_LF_GPIO_PIN     GPIO_PIN_6

// Left rear wheel output PWM hardware details M0PWM2 (gen 1)
#define PWM_LR_BASE	        PWM0_BASE
#define PWM_LR_GEN          PWM_GEN_1
#define PWM_LR_OUTNUM       PWM_OUT_2
#define PWM_LR_OUTBIT       PWM_OUT_2_BIT
#define PWM_LR_PERIPH_PWM	SYSCTL_PERIPH_PWM0
#define PWM_LR_PERIPH_GPIO  SYSCTL_PERIPH_GPIOB
#define PWM_LR_GPIO_BASE    GPIO_PORTB_BASE
#define PWM_LR_GPIO_CONFIG  GPIO_PB4_M0PWM2
#define PWM_LR_GPIO_PIN     GPIO_PIN_4

//Right front wheel output PWM hardware details M0PWM4 (gen 2)
#define PWM_RF_BASE	        PWM0_BASE
#define PWM_RF_GEN          PWM_GEN_2
#define PWM_RF_OUTNUM       PWM_OUT_4
#define PWM_RF_OUTBIT       PWM_OUT_4_BIT
#define PWM_RF_PERIPH_PWM	SYSCTL_PERIPH_PWM0
#define PWM_RF_PERIPH_GPIO  SYSCTL_PERIPH_GPIOE
#define PWM_RF_GPIO_BASE    GPIO_PORTE_BASE
#define PWM_RF_GPIO_CONFIG  GPIO_PE4_M0PWM4
#define PWM_RF_GPIO_PIN     GPIO_PIN_4

//Right rear wheel output PWM hardware details M0PWM6 (gen 3)
#define PWM_RR_BASE	        PWM0_BASE
#define PWM_RR_GEN          PWM_GEN_3
#define PWM_RR_OUTNUM       PWM_OUT_6
#define PWM_RR_OUTBIT       PWM_OUT_6_BIT
#define PWM_RR_PERIPH_PWM	SYSCTL_PERIPH_PWM0
#define PWM_RR_PERIPH_GPIO  SYSCTL_PERIPH_GPIOC
#define PWM_RR_GPIO_BASE    GPIO_PORTC_BASE
#define PWM_RR_GPIO_CONFIG  GPIO_PC4_M0PWM6
#define PWM_RR_GPIO_PIN     GPIO_PIN_4

//Brake level output PWM hardware details M1PWM1 (gen 0)
#define PWM_BRAKE_BASE	        PWM1_BASE
#define PWM_BRAKE_GEN           PWM_GEN_0
#define PWM_BRAKE_OUTNUM        PWM_OUT_1
#define PWM_BRAKE_OUTBIT        PWM_OUT_1_BIT
#define PWM_BRAKE_PERIPH_PWM	SYSCTL_PERIPH_PWM1
#define PWM_BRAKE_PERIPH_GPIO   SYSCTL_PERIPH_GPIOD
#define PWM_BRAKE_GPIO_BASE     GPIO_PORTD_BASE
#define PWM_BRAKE_GPIO_CONFIG   GPIO_PD1_M1PWM1
#define PWM_BRAKE_GPIO_PIN      GPIO_PIN_1
#define PWM_BRAKE_FIXED_HZ      100
#define PWM_BRAKE_START_DUTY    0

//Steering wheel output PWM hardware details M1PWM2 (gen 1)
#define PWM_STEER_BASE	        PWM1_BASE
#define PWM_STEER_GEN           PWM_GEN_1
#define PWM_STEER_OUTNUM        PWM_OUT_2
#define PWM_STEER_OUTBIT        PWM_OUT_2_BIT
#define PWM_STEER_PERIPH_PWM	SYSCTL_PERIPH_PWM1
#define PWM_STEER_PERIPH_GPIO   SYSCTL_PERIPH_GPIOA
#define PWM_STEER_GPIO_BASE     GPIO_PORTA_BASE
#define PWM_STEER_GPIO_CONFIG   GPIO_PA6_M1PWM2
#define PWM_STEER_GPIO_PIN      GPIO_PIN_6
#define PWM_STEERING_FIXED_HZ   200
#define PWM_STEERING_START_DUTY 50


/************************************************************************************
***************************** PUBLIC STRUCT DEFINITIONS *****************************
************************************************************************************/


/**
 * @brief Contains hadware information about specific PWM output
 * @param base Base of PWM module (PWM0_BASE or PWM1_BASE)
 * @param gen PWM generator (can ge gen 1,2,3 or 4)
 * @param outnum Output number of PWM (pwm 1-7)
 * @param outbit Bit-wise ID for pwm output number
 * @param periphPWM Peripheral of PWM (again 0 or 1)
 * @param periphGPIO GPIO peripheral (e.g A, B, C, D...)
 * @param gpioBase Base of GPIO peripheral (A base, B base etc)
 * @param gpioConfig Sets alternate configuration of GPIO pin to PWM
 * @param gpioPin Pin of the port used for PWM output
 */
typedef struct{
    uint32_t base;
    uint32_t gen;
    uint32_t outnum;
    uint32_t outbit;
    uint32_t periphPWM;
    uint32_t periphGPIO;
    uint32_t gpioBase;
    uint32_t gpioConfig;
    uint32_t gpioPin;
} PWMHardwareDetails;

/**
 * @brief PWM signal struct, used for each PWM signal to define it's parameters
 * @param duty Desired duty cycle (%)
 * @param freq Desired frequency (Hz)
 * @param base Base of PWM signal
 * @param gen PWM generator
 * @param outnum Output number of PWM
 */
typedef struct{
uint32_t duty;
uint32_t freq;
uint32_t base;
uint32_t gen;
uint32_t outnum;
}pwmSignal;


/************************************************************************************
**********************************GLOBAL VARIABLES**********************************
************************************************************************************/
extern QueueHandle_t updatePWMQueue;
extern PWMHardwareDetails PWMHardwareDetailsLF;
extern PWMHardwareDetails PWMHardwareDetailsLR;
extern PWMHardwareDetails PWMHardwareDetailsRF;
extern PWMHardwareDetails PWMHardwareDetailsRR;
extern PWMHardwareDetails PWMHardwareDetailsSteering;
extern PWMHardwareDetails PWMHardwareDetailsBrake;


/************************************************************************************
*****************************PUBLIC FUNCTION PROTOTYPES*****************************
************************************************************************************/

/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void setPWM (uint32_t ui32Freq, uint32_t ui32Duty);

/**
 * @brief Set the frequency and duty cycle of any PWM signal
 * @param ui32Freq Desired PWM frequency (Hz)
 * @param ui32Duty Desired PWM duty cycle (%)
 * @param base Base of PWM signal
 * @param gen PWM generator
 * @param outnum PWM output num
 * @return No return
 */
void setPWMGeneral(uint32_t ui32Freq, uint32_t ui32Duty, uint32_t base, uint32_t gen, uint32_t outnum);

/*********************************************************
 * initialisePWM
 * M0PWM7 (J4-05, PC5) is used
 *********************************************************/
void initialisePWM (void);


/**
 * @brief Task that changes PWM output
 * @param args Unused
 * @return None
 */
void updatePWMOutputsTask(void* args);

/**
 * @brief Function to initialise then turn on all car output PWM signals
 * off the output.
 * @return None
 */
void initializeCarPWM(void);


#endif