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
} PWMOutputHardwareDetails_t;

/**
 * @brief Struct to pass desired output pwm to updatePWMOutputsTask via a queue
 * @param duty Desired duty cycle (%)
 * @param freq Desired frequency (Hz)
 * @param pwmOutput PWM output to change
 */
typedef struct{
uint32_t duty;
uint32_t freq;
PWMOutputHardwareDetails_t pwmOutput;
}pwmOutputUpdate_t;


/************************************************************************************
**********************************GLOBAL VARIABLES**********************************
************************************************************************************/
extern QueueHandle_t updatePWMQueue;
extern TaskHandle_t updatePWMOutputsTaskHandle;

/************************************************************************************
*****************************PUBLIC FUNCTION PROTOTYPES*****************************
************************************************************************************/
/**
 * @brief Function to initialise a given PWM. Sets a start Hz and duty but turns
 * off the output.
 * @param pwmOutput Hardware details of PWM to initialise
 * @param startHz Starting Hz
 * @param startDuty Starting duty
 * @return None
 */
void initializePWMGeneral(PWMOutputHardwareDetails_t PWM, uint32_t startHz, uint32_t startDuty);

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



#endif