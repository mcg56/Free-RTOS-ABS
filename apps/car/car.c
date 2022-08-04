#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

#include <FreeRTOS.h>
#include <task.h>
#include "libs/lib_buttons/ap_buttons.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include "stdlib.h"
#include "utils/ustdlib.h"
#include "driverlib/pwm.h"
#include "wheels.h"
#include <queue.h>
#include "libs/lib_pwm/ap_pwm_input.h"
#include "libs/lib_pwm/ap_pwm_output.h"
#include "libs/lib_system/ap_system.h"
#include "libs/lib_uart/ap_uart.h"
#include "ui.h"
#include "driverlib/uart.h"
#include "car_state.h"


//Task handles
TaskHandle_t updateWheelInfoHandle;
TaskHandle_t readButtonsHandle;
TaskHandle_t blinkHandle;
TaskHandle_t updateOLEDHandle;
TaskHandle_t updateUARTHandle;
TaskHandle_t updatePWMOutputsTaskHandle;
TaskHandle_t updateAllPWMInputsHandle;
TaskHandle_t updateDecelHandle;


//TO DO: Move to ui.h
/**
 * @brief Struture for storing input data and passing between tasks 
 * through queues
 * @param steeringWheelDuty Car steering wheel duty (%)
 * @param speed             Car speed
 * @param condition         Road Condition
 * @param pedal             Brake pedal toggle
 * @param brakePressure     Brake pressure (%)
 */
typedef struct {
    uint8_t speed; //m
    uint8_t steeringWheelDuty; //km/h
    uint8_t condition; 
    bool pedal; 
    uint8_t brakePressure;
} InputData;




QueueHandle_t updateDecelQueue = NULL;


/**
 * @brief Creates instances of all queues
 * @return None
 */
void createQueues(void)
{
    OLEDDisplayQueue = xQueueCreate(5, sizeof(DisplayInfo));
    UARTDisplayQueue = xQueueCreate(5, sizeof(DisplayInfo));
    updatePWMQueue = xQueueCreate(10, sizeof(pwmSignal));
    updateDecelQueue = xQueueCreate(5, sizeof(InputData));
}


/**
 * @brief Task that blinks LED
 * @param args Unused
 * @return None
 */
void blink(void* args) {
    (void)args; // unused

    TickType_t wake_time = xTaskGetTickCount();
    

    while (true) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));
        vTaskDelayUntil(&wake_time, 100);

        

        // configASSERT(wake_time < 1000);  // Runs vAssertCalBled() if false
    }
}


/**
 * @brief Task that updates the OLED with current car information.
 * Mostly used just for debugging at the moment
 * @param args Unused
 * @return None
 */
void updateOLEDTask(void* args)
{
    (void)args; // unused
    while(true)
    {
        // Wait until a new message is to be written, as its added to queue
        DisplayInfo updatedDisplayInfo;
        portBASE_TYPE status = xQueueReceive(OLEDDisplayQueue, &updatedDisplayInfo, 100);
        if (status == pdPASS)
        {
            char string[17]; // Display fits 16 characters wide.
            OLEDStringDraw ("                ", 0, 0); //Clear line
            OLEDStringDraw ("                ", 0, 1); //Clear line

            char LFfloatStr[6]; // Display fits 16 characters wide.
            gcvt (updatedDisplayInfo.LF.speed, 4, &LFfloatStr);
            char str2[17];
            //usnprintf (string, sizeof(string), "D: %02d | v: %03d", updatedDisplayInfo.steeringWheelDuty, (int)updatedDisplayInfo.speed);
            sprintf(string, "D: %02d | v: %03d", updatedDisplayInfo.steeringWheelDuty, (int)updatedDisplayInfo.speed);
            OLEDStringDraw (string, 0, 0);
            sprintf(str2, "Lf %s", LFfloatStr);
            OLEDStringDraw (str2, 0, 1);
            //usnprintf (string, sizeof(string), "Lf %.2f Lr %.2f", updatedDisplayInfo.LF.speed, updatedDisplayInfo.LR.speed);
            //
            //usnprintf (string, sizeof(string), "Rf %.2f Rr %.2f", updatedDisplayInfo.RF.speed, updatedDisplayInfo.RR.speed);
            //OLEDStringDraw (string, 0, 2);
        } else continue;

    }
}


/**
 * @brief Update the UART terminal with data about the car.
 * @param args Unused
 * @return No return
 */
void updateUARTTask(void* args)
{
    (void)args;
    TickType_t wake_time = xTaskGetTickCount();

    
    // // Create empty ine array to clear lines
    // char empytLine[MAX_STR_LEN + 1];
    // for(int i = 0; i < MAX_STR_LEN; i++)
    // {
    //     empytLine[i] = ' ';
    // }
    // empytLine[sizeof(empytLine)/sizeof(empytLine[0]) - 1] = '\r'; // Put last index as carridge return

    vt100_print_text();
    vt100_set_white();

    while(true)
    {
        // Wait until a new message is to be written, as its added to queue
        DisplayInfo updatedDisplayInfo;
        portBASE_TYPE status = xQueueReceive(UARTDisplayQueue, &updatedDisplayInfo, 100);
        if (status == pdPASS)
        {

            //Steering line
            
            char alphaStr[6];
            
            gcvt (updatedDisplayInfo.alpha, 4, &alphaStr);
            vt100_print_steering_angle(updatedDisplayInfo.steeringWheelDuty, alphaStr);


            // Car speed line
            vt100_print_car_speed(updatedDisplayInfo.speed);
        
            
            char LFbuff[6];
            char LRbuff[6]; 
            char RFbuff[6]; 
            char RRbuff[6];
            // Wheel speed line
            //First, convert floats to strings
            gcvt (updatedDisplayInfo.LF.speed, 4, &LFbuff);
            gcvt (updatedDisplayInfo.LR.speed, 4, &LRbuff);
            gcvt (updatedDisplayInfo.RF.speed, 4, &RFbuff);
            gcvt (updatedDisplayInfo.RR.speed, 4, &RRbuff);

            vt100_print_wheel_speed(LFbuff, LRbuff, RFbuff, RRbuff);

            // Wheel turn radii line
            //First, convert floats to strings
            gcvt (updatedDisplayInfo.LF.turnRadius, 4, &LFbuff);
            gcvt (updatedDisplayInfo.LR.turnRadius, 4, &LRbuff);
            gcvt (updatedDisplayInfo.RF.turnRadius, 4, &RFbuff);
            gcvt (updatedDisplayInfo.RR.turnRadius, 4, &RRbuff);

            vt100_print_radii(LFbuff, LRbuff, RFbuff, RRbuff);
            
            //Wheel PRR line
            //First, convert floats to strings
            gcvt (updatedDisplayInfo.LF.pulseHz, 4, &LFbuff);
            gcvt (updatedDisplayInfo.LR.pulseHz, 4, &LRbuff);
            gcvt (updatedDisplayInfo.RF.pulseHz, 4, &RFbuff);
            gcvt (updatedDisplayInfo.RR.pulseHz, 4, &RRbuff);

            vt100_print_prr(LFbuff, LRbuff, RFbuff, RRbuff);

            vt100_print_brake_pressure(updatedDisplayInfo.brakePressure);

            vt100_print_condition(updatedDisplayInfo.condition);

            vt100_print_pedal(updatedDisplayInfo.pedal);

        }else continue;
  
    }
}


<<<<<<< HEAD
                leftRear.speed = updatedInput.speed;
                leftRear.turnRadius = 0;

                rightFront.speed = updatedInput.speed;
                rightFront.turnRadius = 0;

                rightRear.speed = updatedInput.speed;
                rightRear.turnRadius = 0;
                
            } 
            else if (alpha < 0.0) //Turning left
            {
                calculateWheelRadii(&leftRear, &leftFront, &rightRear, &rightFront, alpha);
                calculateWheelSpeedsFromRadii(&leftFront, &leftRear, &rightFront, &rightRear, updatedInput.speed);
            }
            else if (alpha > 0.0) //Turning right
            {
                calculateWheelRadii(&rightRear, &rightFront, &leftRear, &leftFront, alpha);
                calculateWheelSpeedsFromRadii(&leftFront, &leftRear, &rightFront, &rightRear, updatedInput.speed);
            }
            calculateWheelPwmFreq(&leftFront, &leftRear, &rightFront, &rightRear);
            detectWheelSlip(&leftFront, &leftRear, &rightFront, &rightRear, slipArray, updatedInput.condition, updatedInput.pedal,updatedInput.brakePressure);
            vt100_print_slipage(slipArray);

            // TO DO Lock which ever wheel is slipping
            // Wheel info updated, signal display tasks to run via queues
            
            DisplayInfo updatedDisplayInfo = {leftFront, leftRear, rightFront, rightRear, updatedInput.speed, updatedInput.steeringWheelDuty, alpha, updatedInput.condition, updatedInput.pedal, updatedInput.brakePressure};
            xQueueSendToBack(UARTDisplayQueue, &updatedDisplayInfo, 0);
            xQueueSendToBack(OLEDDisplayQueue, &updatedDisplayInfo, 0);
            
            xQueueSendToBack(updateDecelQueue, &updatedInput, 0);

            /* Sending wheel pwms 1 at a time may cause issues as it updates them one at a time so abs
            controller might think its slipping whne it just hasnt updated all wheels yet*/
            pwmSignal leftFrontPWM = {PWM_WHEEL_FIXED_DUTY, (uint32_t)leftFront.pulseHz, PWMHardwareDetailsLF.base, PWMHardwareDetailsLF.gen, PWMHardwareDetailsLF.outnum};
            xQueueSendToBack(updatePWMQueue, &leftFrontPWM, 0);

            pwmSignal leftRearPWM = {PWM_WHEEL_FIXED_DUTY, (uint32_t)leftRear.pulseHz, PWMHardwareDetailsLR.base, PWMHardwareDetailsLR.gen, PWMHardwareDetailsLR.outnum};
            xQueueSendToBack(updatePWMQueue, &leftRearPWM, 0);

            pwmSignal rightFrontPWM = {PWM_WHEEL_FIXED_DUTY, (uint32_t)rightFront.pulseHz, PWMHardwareDetailsRF.base, PWMHardwareDetailsRF.gen, PWMHardwareDetailsRF.outnum};
            xQueueSendToBack(updatePWMQueue, &rightFrontPWM, 0);

            pwmSignal rightRearPWM = {PWM_WHEEL_FIXED_DUTY, (uint32_t)rightRear.pulseHz, PWMHardwareDetailsRR.base, PWMHardwareDetailsRR.gen, PWMHardwareDetailsRR.outnum};
            xQueueSendToBack(updatePWMQueue, &rightRearPWM, 0);

        }else continue;
    }
}
=======
>>>>>>> 66fff860e169e3d4a4460ff5410a52f71ccce11a

// TO DO: change to read uart task? and move to ui.c
/**
 * @brief Reads the buttons and changes inputs accordingly
 * @param args Unused
 * @return No return
 */
void readButtonsTask(void* args)
{
    (void)args; // unused
    const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
    while (true) 
    {
        updateButtons();
        
        //char string[17]; // Display fits 16 characters wide.
        
        int32_t c = UARTCharGetNonBlocking(UART_USB_BASE);

        // Update values accodingly. No checks for value max/min limits yet
        bool change = false;

        // Can only change values if brake is off
        if (getPedalState() == 0) {
            if (checkButton(UP) == PUSHED || c == 'w')
            {
                uint8_t currentSpeed = getCarSpeed();
                setCarSpeed(currentSpeed + 5);
                change = true;            
            }
            if (checkButton(DOWN) == PUSHED || c == 'q')
            {
                uint8_t currentSpeed = getCarSpeed();
                if (currentSpeed != 0) {
                    setCarSpeed(currentSpeed - 5);
                }
                change = true;
            }
            if (checkButton(LEFT) == PUSHED || c == '1')
            {
                uint8_t currentSteeringWheelDuty = getSteeringDuty();
                if (currentSteeringWheelDuty > 5) {
                    setSteeringDuty(currentSteeringWheelDuty - 5);
                }
                change = true;
            }
            if (checkButton(RIGHT) == PUSHED|| c == '2')
            {
                uint8_t currentSteeringWheelDuty = getSteeringDuty();
                if (currentSteeringWheelDuty < 95) {
                    setSteeringDuty(currentSteeringWheelDuty + 5);
                }
                change = true;
            }
            if (c == '[')
            {
                uint8_t currentBrakeDuty = getBrakePressureDuty();
                if (currentBrakeDuty > 5) {
                    setBrakePressureDuty(currentBrakeDuty - 5);
                }
                change = true;
            }
            if (c == ']')
            {
                uint8_t currentBrakeDuty = getBrakePressureDuty();
                if (currentBrakeDuty < 95) {
                    setBrakePressureDuty(currentBrakeDuty + 5);
                }
                change = true;
            }
            if (c == 'r')
            {
                uint8_t currentRoadCondition = getRoadCondition(); 
                if  (currentRoadCondition <= 2) setRoadCondition(currentRoadCondition + 1);
                else setRoadCondition(0);
                change = true;
            }
        }

        if (c == 'b')
        {
            bool pedalState = getPedalState();
            if  (pedalState == 0)
            {
                setPedalState(1);
                // Start decleration task
                vTaskResume(updateDecelHandle);
            } else {
                // Stop deceleration Task
                vTaskSuspend(updateDecelHandle);
                // Update the queue in the button task to match the latest speed changes
                setPedalState(0);
            }
            change = true;
        }
        
        //Check if any buttons changed
        if (change){
            // Tell the wheel update task to run
            xTaskNotifyGiveIndexed(updateWheelInfoHandle, 0);

            uint8_t currentSteeringWheelDuty = getSteeringDuty();
            pwmSignal steeringPWM = {currentSteeringWheelDuty, PWM_STEERING_FIXED_HZ, PWMHardwareDetailsSteering.base, PWMHardwareDetailsSteering.gen, PWMHardwareDetailsSteering.outnum};
            xQueueSendToBack(updatePWMQueue, &steeringPWM, 0);
        }


        taskYIELD(); // Not sure if this is needed or not
        vTaskDelay(xDelay);
    }
}

void processABSPWMInputTask(void* args)
{
    (void)args;
    const TickType_t xDelay = 330 / portTICK_PERIOD_MS;
    PWMSignal_t pwmDetails;

    while (true) 
    {
        bool ABSOn = false;
        for (int i = 0; i<10; i++)
        {
            if(updatePWMInput(ABSPWM_ID)) // Timeout occured, ABS must be on
            {
                ABSOn = true;
            } else{
                pwmDetails = getPWMInputSignal(ABSPWM_ID);
            }

        }
        if (ABSOn)
        {
            // Toggle blue
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2));
        }
        char string[17]; // Display fits 16 characters wide.
        sprintf(string, "D: %02ld F: %03ld", pwmDetails.duty, pwmDetails.frequency);
        OLEDStringDraw (string, 0, 3);

        vTaskDelay(xDelay);
    }   
}

void updateDecel (void* args)
{
    (void)args;

    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS; // Need to couple to thingy //Need to reduce in smaller increment dt*max decel*brakepress

    
    while (true)
    {
            // Get the current input data
            InputData currentInput;
            portBASE_TYPE status = xQueueReceive(updateDecelQueue, &currentInput, 100);

            // Modify the speed dependant on brake pressure
            currentInput.speed = currentInput.speed - currentInput.brakePressure/5;
            if (currentInput.speed <= 0 || currentInput.speed >= 200) {
                    currentInput.speed = 0;
            }

            setCarSpeed((uint8_t)newSpeed);
            // Tell the wheel update task to run
            xTaskNotifyGiveIndexed(updateWheelInfoHandle, 0);
            vTaskDelay(xDelay);
    }   
}


int main(void) {
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    OLEDInitialise ();

    // Setup red LED on PF1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    // Setup Blue LED on PF2
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    initButtons();
    initPWMInputManager (CAR_PWM_MIN_FREQ);

    initialiseUSB_UART ();
    initializeCarPWMOutputs();


    PWMSignal_t ABSPWM = {.id = ABSPWM_ID, .gpioPin = GPIO_PIN_0};
    registerPWMSignal(ABSPWM);

    createQueues();
    //xTaskCreate(&blink, "blink", 150, NULL, 0, &blinkHandle);
    xTaskCreate(&readButtonsTask, "read buttons", 150, NULL, 0, &readButtonsHandle);
    xTaskCreate(&updateWheelInfoTask, "update wheel info", 256, NULL, 0, &updateWheelInfoHandle);
    //xTaskCreate(&updateOLEDTask, "update OLED", 256, NULL, 0, &updateOLEDHandle);
    xTaskCreate(&updateUARTTask, "update UART", 256, NULL, 0, &updateUARTHandle);
    xTaskCreate(&updatePWMOutputsTask, "update PWM", 256, NULL, 0, &updatePWMOutputsTaskHandle);
    xTaskCreate(&processABSPWMInputTask, "Update abs pwm input", 256, NULL, 0, NULL);
    xTaskCreate(&updateDecel, "updateDecel", 256, NULL, 0, &updateDecelHandle);
    vTaskSuspend(updateDecelHandle);

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

