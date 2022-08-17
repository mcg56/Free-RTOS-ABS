#include "car_pwm.h"
#include "libs/lib_pwm/pwm_output.h"
#include "libs/lib_pwm/pwm_input.h"

PWMOutputHardwareDetails_t PWMHardwareDetailsLF = {PWM_LF_BASE, PWM_LF_GEN, PWM_LF_OUTNUM, PWM_LF_OUTBIT, PWM_LF_PERIPH_PWM, PWM_LF_PERIPH_GPIO, PWM_LF_GPIO_BASE, PWM_LF_GPIO_CONFIG, PWM_LF_GPIO_PIN};
PWMOutputHardwareDetails_t PWMHardwareDetailsLR = {PWM_LR_BASE, PWM_LR_GEN, PWM_LR_OUTNUM, PWM_LR_OUTBIT, PWM_LR_PERIPH_PWM, PWM_LR_PERIPH_GPIO, PWM_LR_GPIO_BASE, PWM_LR_GPIO_CONFIG, PWM_LR_GPIO_PIN};
PWMOutputHardwareDetails_t PWMHardwareDetailsRF = {PWM_RF_BASE, PWM_RF_GEN, PWM_RF_OUTNUM, PWM_RF_OUTBIT, PWM_RF_PERIPH_PWM, PWM_RF_PERIPH_GPIO, PWM_RF_GPIO_BASE, PWM_RF_GPIO_CONFIG, PWM_RF_GPIO_PIN};
PWMOutputHardwareDetails_t PWMHardwareDetailsRR = {PWM_RR_BASE, PWM_RR_GEN, PWM_RR_OUTNUM, PWM_RR_OUTBIT, PWM_RR_PERIPH_PWM, PWM_RR_PERIPH_GPIO, PWM_RR_GPIO_BASE, PWM_RR_GPIO_CONFIG, PWM_RR_GPIO_PIN};
PWMOutputHardwareDetails_t PWMHardwareDetailsBrake = {PWM_BRAKE_BASE, PWM_BRAKE_GEN, PWM_BRAKE_OUTNUM, PWM_BRAKE_OUTBIT, PWM_BRAKE_PERIPH_PWM, PWM_BRAKE_PERIPH_GPIO, PWM_BRAKE_GPIO_BASE, PWM_BRAKE_GPIO_CONFIG, PWM_BRAKE_GPIO_PIN};
PWMOutputHardwareDetails_t PWMHardwareDetailsSteering = {PWM_STEER_BASE, PWM_STEER_GEN, PWM_STEER_OUTNUM, PWM_STEER_OUTBIT, PWM_STEER_PERIPH_PWM, PWM_STEER_PERIPH_GPIO, PWM_STEER_GPIO_BASE, PWM_STEER_GPIO_CONFIG, PWM_STEER_GPIO_PIN};


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