/** @file   user_interface.c
    @author A.J Eason A. Musalov
    @date   22/08/22
    @brief  Module which primary function is to maintain the UART terminal
            user interface. The module contains the USB_UART initialisation
            and UART display functions.
*/

#include "user_interface.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "libs/lib_uart/uart.h"
#include "driverlib/uart.h"
#include "car_state.h"
#include "car_pwm.h"
#include "libs/lib_buttons/buttons.h"
#include "libs/lib_pwm/pwm_output.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"

//*************************************************************
// Private Constant Definitions
//*************************************************************

#define UART_TASK_DELAY_MS          500
#define USER_INPUT_TASK_DELAY_MS    100
#define CAR_SPEED_INCREMENT         2
#define CAR_SPEED_DECREMENT         2
#define MAX_SPEED                   120
#define MIN_SPEED                   0
#define DUTY_CYCLE_MIN              5
#define STEERING_DUTY_INCREMENT     5
#define DUTY_CYCLE_MAX              95
#define BRAKE_PEDAL_INCREMENT       5
#define ROAD_CONDITION_INCREMENT    1
#define DRY_CONDITION               0
#define WET_CONDITION               1
#define ICY_CONDITION               2
#define PEDAL_STATE_ON              1
#define PEDAL_STATE_OFF             0
#define USER_INPUT_TASK_PRIORITY    1
#define USER_INPUT_TASK_STACK_SIZE  150
#define UART_TASK_PRIORITY          0
#define UART_TASK_STACK_SIZE        256


//*****************************************************************************
// Global variables
//*****************************************************************************

TaskHandle_t processUserInputsTaskHandle;
TaskHandle_t updateUARTTaskHandle;

//*************************************************************
// Private function prototypes
//*************************************************************

/**
 * @brief Converts a double or float to a string.
 * @param x         Floating point number
 * @param ndigit    Numer of digits of precision
 * @param buf       Char buffer to write to
 * @return Pointer to char buffer
 */
char *gcvt(double x, int ndigit, char *buf);

/**
 * @brief Task to process the user input from the UART and  buttons to control
 * the car simulator.
 * @param args Unused
 * @return No return
 */
void processUserInputsTask(void* args);

/**
 * @brief Task to update the car information on the UART display.
 * @param args Unused
 * @return No return
 */
void updateUARTTask(void* args);

//*****************************************************************************
// Functions
//*****************************************************************************

void initUserInterface(void)
{
    // Setup red LED on PF1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    // Initialise peripherals
    OLEDInitialise ();
    initButtons();
    initialiseUSB_UART ();

    // Create Tasks
    xTaskCreate(&processUserInputsTask, "Process inputs", USER_INPUT_TASK_STACK_SIZE, NULL, USER_INPUT_TASK_PRIORITY, &processUserInputsTaskHandle);
    xTaskCreate(&updateUARTTask, "update UART", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, &updateUARTTaskHandle);

}

void printInitialInformation(void)
{
    char LFbuff[6];
    char LRbuff[6]; 
    char RFbuff[6]; 
    char RRbuff[6];
    char floatBuff[6];
    // Static text
    vt100_print_text();

    // Steering angle
    gcvt (getSteeringAngle(), 4, floatBuff);
    vt100_print_steering_angle(getSteeringDuty(), floatBuff);

    // Car speed
    gcvt (getCarSpeed(), 4, floatBuff);
    vt100_print_car_speed(floatBuff);

    // Wheel information
    Wheel LF = getleftFront();
    Wheel LR = getleftRear();
    Wheel RF = getRightFront();
    Wheel RR = getRightRear();
    // Wheel speed line
    //Convert floats to strings
    gcvt (LF.speed, 4, LFbuff);
    gcvt (LR.speed, 4, LRbuff);
    gcvt (RF.speed, 4, RFbuff);
    gcvt (RR.speed, 4, RRbuff);
    vt100_print_wheel_speed(LFbuff, LRbuff, RFbuff, RRbuff);

    //Wheel PRR line
    //First, convert floats to strings
    gcvt (LF.pulseHz, 4, LFbuff);
    gcvt (LR.pulseHz, 4, LRbuff);
    gcvt (RF.pulseHz, 4, RFbuff);
    gcvt (RR.pulseHz, 4, RRbuff);
    vt100_print_prr(LFbuff, LRbuff, RFbuff, RRbuff);

    //Radius line
    //First, convert floats to strings
    gcvt (LF.turnRadius, 4, LFbuff);
    gcvt (LR.turnRadius, 4, LRbuff);
    gcvt (RF.turnRadius, 4, RFbuff);
    gcvt (RR.turnRadius, 4, RRbuff);
    vt100_print_radii(LFbuff, LRbuff, RFbuff, RRbuff);

    // Brake pressure, road condition and pedal state
    vt100_print_brake_pressure(getBrakePedalPressureDuty());
    vt100_print_condition(getRoadCondition());
    vt100_print_pedal(getPedalState());

    // Wheel slip information
    bool slipArray[4] = {LF.slipping, LR.slipping, RF.slipping, RR.slipping};
    bool absState = getABSState();
    vt100_print_slipage(slipArray, absState);
}

void vt100_print_text(void) {
    vt100_clear();
    vt100_set_yellow();
    UARTSend ("*** ABS SIM (12.07.22) ***");
    vt100_set_line_number(2);
    UARTSend ("Steering -> (a, d):");
    vt100_set_line_number(4);
    UARTSend ("Car speed (km/h) -> (s, w):");
    vt100_set_line_number(6);
    UARTSend ("Wheel speed (km/h):");
    vt100_set_line_number(8);
    UARTSend ("Wheel PRR (Hz):");
    vt100_set_line_number(10);
    UARTSend ("Wheel radii (m)");
    vt100_set_line_number(12);
    UARTSend ("Brake pedal pressure -> ([, ]):");
    vt100_set_line_number(14);
    UARTSend ("Brake pedal push/release -> (b):");
    vt100_set_line_number(16);
    UARTSend ("Road Condition -> (r):");
    vt100_set_line_number(18);
    UARTSend ("Wheel Slip -> (LF LR RF RR):");
    vt100_set_white();
}

void vt100_print_radii(char LF[6],char LR[6],char RF[6],char RR[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(11);
    sprintf (ANSIString, "Lf: %5s, Lr: %5s, Rf: %5s, Rr: %5s\r\n\n", LF, LR, RF, RR);
    UARTSend (ANSIString);
}

void vt100_print_prr(char LF[6],char LR[6],char RF[6],char RR[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(9);
    sprintf (ANSIString, "Lf: %5s, Lr: %5s, Rf: %5s, Rr: %5s\r\n\n", LF, LR, RF, RR);
    UARTSend (ANSIString);
}

void vt100_print_condition(Condition condition) {
    vt100_set_line_number(17);
    if (condition == 0){
        UARTSend ("DRY");
    } else if (condition == 1){
        UARTSend ("WET");
    } else {
        UARTSend ("ICY");
    }
}

void updateUARTTask(void* args)
{
    (void)args;
    const float taskPeriodms = UART_TASK_DELAY_MS;
    // Save previous writes to check if they need to be updated
    uint8_t prevSteeringDuty;
    uint8_t prevSpeed;
    float prevLFSpeed;
    float prevLFRadius;
    uint8_t prevBrakeDuty;
    uint8_t prevRoadCondition;
    bool prevPedalState;
    bool prevSlipArray[4];
    bool prevABSState;

    char LFbuff[6];
    char LRbuff[6]; 
    char RFbuff[6]; 
    char RRbuff[6];
    char floatBuff[6];

    printInitialInformation();
    
    while(true)
    {
        TickType_t wake_time = xTaskGetTickCount();
        // Wait until we can take the mutex to be able to use car state shared resource
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        // We have obtained the mutex, now get all car state variables

        //Get latest car information
        float speed = getCarSpeed();
        uint8_t steeringDuty = getSteeringDuty();
        float alpha = getSteeringAngle();
        uint8_t roadCondition = getRoadCondition();
        bool pedalState = getPedalState();
        uint8_t brakeDuty = getBrakePedalPressureDuty();
        bool absState = getABSState();
        Wheel LF = getleftFront();
        Wheel LR = getleftRear();
        Wheel RF = getRightFront();
        Wheel RR = getRightRear();
        xSemaphoreGive(carStateMutex);
        // Returned semaphore before printing the UART information

        // Steering line
        if (steeringDuty != prevSteeringDuty) // Only write line if there was a change
        {      
            gcvt (alpha, 4, floatBuff);
            vt100_print_steering_angle(steeringDuty, floatBuff);
            prevSteeringDuty = steeringDuty;
        }
        
        // Car speed line
        // Only write line if there was a change.
        // Need >= as = comparison for very small speeds returns incorrect boolean
        if (fabs(speed - prevSpeed) >= 0.0f) 
        {
            gcvt (speed, 4, floatBuff);
            vt100_print_car_speed(floatBuff);
            prevSpeed = speed;
        }
        
        // Wheel information lines
        // Wheel speed and PRR lines
        // If one wheel changed speed, they will all have changed speed and PRR.
        if (LF.speed != prevLFSpeed)// Only write line if there was a change
        {
            // Wheel speed line
            //Convert floats to strings
            gcvt (LF.speed, 4, LFbuff);
            gcvt (LR.speed, 4, LRbuff);
            gcvt (RF.speed, 4, RFbuff);
            gcvt (RR.speed, 4, RRbuff);

            vt100_print_wheel_speed(LFbuff, LRbuff, RFbuff, RRbuff);

            //Wheel PRR line
            //First, convert floats to strings
            gcvt (LF.pulseHz, 4, LFbuff);
            gcvt (LR.pulseHz, 4, LRbuff);
            gcvt (RF.pulseHz, 4, RFbuff);
            gcvt (RR.pulseHz, 4, RRbuff);

            vt100_print_prr(LFbuff, LRbuff, RFbuff, RRbuff);
            prevLFSpeed = LF.speed;
        }

        // Wheel turn radii line
        // If one radius changed, they will all have changed
        if (LF.turnRadius != prevLFRadius) // Only write line if there was a change
        {
            //First, convert floats to strings
            gcvt (LF.turnRadius, 4, LFbuff);
            gcvt (LR.turnRadius, 4, LRbuff);
            gcvt (RF.turnRadius, 4, RFbuff);
            gcvt (RR.turnRadius, 4, RRbuff);

            vt100_print_radii(LFbuff, LRbuff, RFbuff, RRbuff);
            prevLFRadius = LF.turnRadius;
        }
        
        if (brakeDuty != prevBrakeDuty)
        {
            vt100_print_brake_pressure(brakeDuty);
            prevBrakeDuty = brakeDuty;
        }

        if (roadCondition != prevRoadCondition)
        {
            vt100_print_condition(roadCondition);
            prevRoadCondition = roadCondition;
        }
        
        if (pedalState != prevPedalState)
        {   
            vt100_print_pedal(pedalState);
            prevPedalState = pedalState;
        }
        
        // If a wheel has started slipping, update the UART
        bool slipArray[4] = {LF.slipping, LR.slipping, RF.slipping, RR.slipping};
        if((prevSlipArray[0] != slipArray[0]) || (prevSlipArray[1] != slipArray[1]) || (prevSlipArray[2] != slipArray[2]) || (prevSlipArray[3] != slipArray[3]) || (absState != prevABSState))
        {
            vt100_print_slipage(slipArray, absState);
            for (int i=0;i < 4; i++)
            {
                prevSlipArray[i] = slipArray[i];
            }
            prevABSState = absState;
        }

        vTaskDelayUntil(&wake_time, taskPeriodms);
    }
}

void processUserInputsTask(void* args)
{
    (void)args; // unused
    const float taskPeriodms = USER_INPUT_TASK_DELAY_MS; //ms
    while (true) 
    {
        TickType_t wake_time = xTaskGetTickCount();
        updateButtons();

        // Convert to lowercase in case of CAPSLOCK
        int32_t c = tolower(UARTCharGetNonBlocking(UART_USB_BASE));

        // Wait until we can take the mutex to be able to use car state shared resource
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        // We have obtained the mutex, now can run the task

        
        // Update values accodingly.
        bool change = false;

        // Car speed user input
        if (checkButton(UP) == PUSHED || c == 'w')
        {
            float currentSpeed = getCarSpeed(); 
            setCarSpeed(fmin(currentSpeed + CAR_SPEED_INCREMENT, MAX_SPEED));
            change = true;            
        }
        if (checkButton(DOWN) == PUSHED || c == 's')
        {
            float currentSpeed = getCarSpeed();
            setCarSpeed(fmax(currentSpeed - CAR_SPEED_INCREMENT, MIN_SPEED));
            change = true;
        }
        // Car steering user input
        if (checkButton(LEFT) == PUSHED || c == 'a')
        {
            uint8_t currentSteeringWheelDuty = getSteeringDuty();
            if (currentSteeringWheelDuty > DUTY_CYCLE_MIN) {
                setSteeringDuty(currentSteeringWheelDuty - STEERING_DUTY_INCREMENT);
            }
            // Notify PWM task to update steering PWM to new value
            pwmOutputUpdate_t steeringPWM = {getSteeringDuty(), PWM_STEERING_FIXED_HZ, PWMHardwareDetailsSteering};
            xQueueSendToBack(updatePWMQueue, &steeringPWM, 0);
            change = true;
        }
        if (checkButton(RIGHT) == PUSHED|| c == 'd')
        {
            uint8_t currentSteeringWheelDuty = getSteeringDuty();
            if (currentSteeringWheelDuty < DUTY_CYCLE_MAX) {
                setSteeringDuty(currentSteeringWheelDuty + STEERING_DUTY_INCREMENT);
            }
            // Notify PWM task to update steering PWM to new value
            pwmOutputUpdate_t steeringPWM = {getSteeringDuty(), PWM_STEERING_FIXED_HZ, PWMHardwareDetailsSteering};
            xQueueSendToBack(updatePWMQueue, &steeringPWM, 0);
            change = true;
        }

        // Car brake pressure duty cycle
        if (c == '[')
        {
            uint8_t currentPedalBrakeDuty = getBrakePedalPressureDuty();
            if (currentPedalBrakeDuty > DUTY_CYCLE_MIN) {
                setBrakePedalPressureDuty(currentPedalBrakeDuty - BRAKE_PEDAL_INCREMENT);
            }
            if(getPedalState()) // Brake pedal pressed, need to update brake PWM to abs controller
            {
                // Notify PWM task to update brake pwm as pedal is activated
                pwmOutputUpdate_t brakePWM = {getBrakePedalPressureDuty(), PWM_BRAKE_FIXED_HZ, PWMHardwareDetailsBrake};
                xQueueSendToBack(updatePWMQueue, &brakePWM, 0);
            }
            change = true;
        }
        if (c == ']')
        {
            uint8_t currentPedalBrakeDuty = getBrakePedalPressureDuty();
            if (currentPedalBrakeDuty < DUTY_CYCLE_MAX) {
                setBrakePedalPressureDuty(currentPedalBrakeDuty + BRAKE_PEDAL_INCREMENT);
            }
            if(getPedalState()) // Brake pedal pressed, need to update brake PWM to abs controller
            {
                // Notify PWM task to update brake pwm as pedal is activated
                pwmOutputUpdate_t brakePWM = {getBrakePedalPressureDuty(), PWM_BRAKE_FIXED_HZ, PWMHardwareDetailsBrake};
                xQueueSendToBack(updatePWMQueue, &brakePWM, 0);
            }
            change = true;
        }

        // Road condition user input
        if (c == 'r')
        {   
            if  ((getRoadCondition() + ROAD_CONDITION_INCREMENT) > ICY_CONDITION) setRoadCondition(DRY_CONDITION);
                else (setRoadCondition(getRoadCondition() + ROAD_CONDITION_INCREMENT));
            change = true;
        }

        // Brake pedal toggle user input
        if (c == 'b')
        {
            bool pedalState = getPedalState();
            if  (pedalState == 0)
            {
                setPedalState(PEDAL_STATE_ON);
                // Start decleration task
                vTaskResume(decelerationTaskHandle);

                // Notify PWM task to update brake pwm as pedal is activated
                pwmOutputUpdate_t brakePWM = {getBrakePedalPressureDuty(), PWM_BRAKE_FIXED_HZ, PWMHardwareDetailsBrake};
                xQueueSendToBack(updatePWMQueue, &brakePWM, 0);
            } else {
                // Stop deceleration Task
                vTaskSuspend(decelerationTaskHandle);

                // Set brake pwm to 0% duty
                pwmOutputUpdate_t brakePWM = {0, PWM_BRAKE_FIXED_HZ, PWMHardwareDetailsBrake};
                xQueueSendToBack(updatePWMQueue, &brakePWM, 0);

                setPedalState(PEDAL_STATE_OFF);
            }
            change = true;
        }

        // Give the mutex back
        xSemaphoreGive(carStateMutex);
        
        //Check if any buttons changed
        if (change){
            // Tell the wheel update task to run
            xTaskNotifyGiveIndexed(updateWheelInfoHandle, 0);
        }
        vTaskDelayUntil(&wake_time, taskPeriodms);
    }
}