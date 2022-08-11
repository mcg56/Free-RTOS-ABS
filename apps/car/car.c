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
TaskHandle_t readInputsHandle;
TaskHandle_t updateUARTHandle;
TaskHandle_t updatePWMOutputsTaskHandle;
TaskHandle_t updateAllPWMInputsHandle;
TaskHandle_t decelerationTaskHandle;

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



/**
 * @brief Update the UART terminal with data about the car.
 * @param args Unused
 * @return No return
 */
void updateUARTTask(void* args)
{
    (void)args;
    const TickType_t xDelay = 333 / portTICK_PERIOD_MS;

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

    vt100_print_text();

    // Print statting information first time task is run
    gcvt (getSteeringAngle(), 4, &floatBuff);
    vt100_print_steering_angle(getSteeringDuty(), floatBuff);

    gcvt (getCarSpeed(), 4, &floatBuff);
    vt100_print_car_speed(floatBuff);
    Wheel LF = getleftFront();
    Wheel LR = getleftRear();
    Wheel RF = getRightFront();
    Wheel RR = getRightRear();
    // Wheel speed line
    //Convert floats to strings
    gcvt (LF.speed, 4, &LFbuff);
    gcvt (LR.speed, 4, &LRbuff);
    gcvt (RF.speed, 4, &RFbuff);
    gcvt (RR.speed, 4, &RRbuff);

    vt100_print_wheel_speed(LFbuff, LRbuff, RFbuff, RRbuff);

    //Wheel PRR line
    //First, convert floats to strings
    gcvt (LF.pulseHz, 4, &LFbuff);
    gcvt (LR.pulseHz, 4, &LRbuff);
    gcvt (RF.pulseHz, 4, &RFbuff);
    gcvt (RR.pulseHz, 4, &RRbuff);

    vt100_print_prr(LFbuff, LRbuff, RFbuff, RRbuff);

    //Radius line
    //First, convert floats to strings
    gcvt (LF.turnRadius, 4, &LFbuff);
    gcvt (LR.turnRadius, 4, &LRbuff);
    gcvt (RF.turnRadius, 4, &RFbuff);
    gcvt (RR.turnRadius, 4, &RRbuff);

    vt100_print_radii(LFbuff, LRbuff, RFbuff, RRbuff);
    vt100_print_brake_pressure(getBrakePedalPressureDuty());
    vt100_print_condition(getRoadCondition());
    vt100_print_pedal(getPedalState());

    bool slipArray[4] = {LF.slipping, LR.slipping, RF.slipping, RR.slipping};
    bool absState = getABSState();
    vt100_print_slipage(slipArray, absState);

    
    while(true)
    {
        // Wait until we can take the mutex to be able to use car state shared resource
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        // We have obtained the mutex, now can run the task

        //Steering line
        uint8_t steeringDuty = getSteeringDuty();
        if (steeringDuty != prevSteeringDuty) // Only write line if there was a change
        {      
            gcvt (getSteeringAngle(), 4, &floatBuff);
            vt100_print_steering_angle(getSteeringDuty(), floatBuff);
            prevSteeringDuty = steeringDuty;
        }
        
        // Car speed line
        float speed = getCarSpeed();
        if (speed != prevSpeed) // Only write line if there was a change
        {
            gcvt (getCarSpeed(), 4, &floatBuff);
            vt100_print_car_speed(floatBuff);
            prevSpeed = speed;
        }
        
        // Wheel information lines
        // First get the wheel structs
        Wheel LF = getleftFront();
        Wheel LR = getleftRear();
        Wheel RF = getRightFront();
        Wheel RR = getRightRear();
        
        // Wheel speed and PRR lines
        // If one wheel changed speed, they will all have changed speed and PRR.
        if (LF.speed != prevLFSpeed)// Only write line if there was a change
        {
            // Wheel speed line
            //Convert floats to strings
            gcvt (LF.speed, 4, &LFbuff);
            gcvt (LR.speed, 4, &LRbuff);
            gcvt (RF.speed, 4, &RFbuff);
            gcvt (RR.speed, 4, &RRbuff);

            vt100_print_wheel_speed(LFbuff, LRbuff, RFbuff, RRbuff);

            //Wheel PRR line
            //First, convert floats to strings
            gcvt (LF.pulseHz, 4, &LFbuff);
            gcvt (LR.pulseHz, 4, &LRbuff);
            gcvt (RF.pulseHz, 4, &RFbuff);
            gcvt (RR.pulseHz, 4, &RRbuff);

            vt100_print_prr(LFbuff, LRbuff, RFbuff, RRbuff);
            prevLFSpeed = LF.speed;
        }

        // Wheel turn radii line
        // If one radius changed, they will all have changed
        if (LF.turnRadius != prevLFRadius) // Only write line if there was a change
        {
            //First, convert floats to strings
            gcvt (LF.turnRadius, 4, &LFbuff);
            gcvt (LR.turnRadius, 4, &LRbuff);
            gcvt (RF.turnRadius, 4, &RFbuff);
            gcvt (RR.turnRadius, 4, &RRbuff);

            vt100_print_radii(LFbuff, LRbuff, RFbuff, RRbuff);
            prevLFRadius = LF.turnRadius;
        }
        
        uint8_t brakeDuty = getBrakePedalPressureDuty();
        if (brakeDuty != prevBrakeDuty)
        {
            vt100_print_brake_pressure(brakeDuty);
            prevBrakeDuty = brakeDuty;
        }


        uint8_t roadCondition  = getRoadCondition();
        if (roadCondition != prevRoadCondition)
        {
            vt100_print_condition(roadCondition);
            prevRoadCondition = roadCondition;
        }
        

        bool pedalState = getPedalState();
        if (pedalState != prevPedalState)
        {   
            vt100_print_pedal(pedalState);
            prevPedalState = pedalState;
        }
        
        bool slipArray[4] = {LF.slipping, LR.slipping, RF.slipping, RR.slipping};
        bool absState = getABSState();
        if((prevSlipArray[0] != slipArray[0]) || (prevSlipArray[1] != slipArray[1]) || (prevSlipArray[2] != slipArray[2]) || (prevSlipArray[3] != slipArray[3]) || (absState != prevABSState))
        {
            vt100_print_slipage(slipArray, absState);
            for (int i=0;i < 4; i++)
            {
                prevSlipArray[i] = slipArray[i];
            }
            prevABSState = absState;
        }

        // Give the mutex back
        xSemaphoreGive(carStateMutex);

        vTaskDelay(xDelay);
    }
}



// TO DO: change to read uart task? and move to ui.c
/**
 * @brief Reads the buttons and changes inputs accordingly
 * @param args Unused
 * @return No return
 */
void readInputsTask(void* args)
{
    (void)args; // unused
    const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
    while (true) 
    {
        updateButtons();
        // Wait until we can take the mutex to be able to use car state shared resource
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        // We have obtained the mutex, now can run the task

        int32_t c = tolower(UARTCharGetNonBlocking(UART_USB_BASE));
        // Update values accodingly.
        bool change = false;

        if (checkButton(UP) == PUSHED || c == 'w')
        {
            float currentSpeed = getCarSpeed();
            setCarSpeed(currentSpeed + 5);
            change = true;            
        }
        if (checkButton(DOWN) == PUSHED || c == 's')
        {
            float currentSpeed = getCarSpeed();
            if (currentSpeed != 0) {
                setCarSpeed(currentSpeed - 5);
            }
            change = true;
        }
        if (checkButton(LEFT) == PUSHED || c == 'a')
        {
            uint8_t currentSteeringWheelDuty = getSteeringDuty();
            if (currentSteeringWheelDuty > 5) {
                setSteeringDuty(currentSteeringWheelDuty - 5);
            }
            // Notify PWM task to update steering PWM to new value
            pwmOutputUpdate_t steeringPWM = {getSteeringDuty(), PWM_STEERING_FIXED_HZ, pwmSteering};
            xQueueSendToBack(updatePWMQueue, &steeringPWM, 0);
            change = true;
        }
        if (checkButton(RIGHT) == PUSHED|| c == 'd')
        {
            uint8_t currentSteeringWheelDuty = getSteeringDuty();
            if (currentSteeringWheelDuty < 95) {
                setSteeringDuty(currentSteeringWheelDuty + 5);
            }
            // Notify PWM task to update steering PWM to new value
            pwmOutputUpdate_t steeringPWM = {getSteeringDuty(), PWM_STEERING_FIXED_HZ, pwmSteering};
            xQueueSendToBack(updatePWMQueue, &steeringPWM, 0);
            change = true;
        }
        if (c == '[')
        {
            uint8_t currentPedalBrakeDuty = getBrakePedalPressureDuty();
            if (currentPedalBrakeDuty > 5) {
                setBrakePedalPressureDuty(currentPedalBrakeDuty - 5);
            }
            if(getPedalState()) // Brake pedal pressed, need to update brake PWM to abs controller
            {
                // Notify PWM task to update brake pwm as pedal is activated
                pwmOutputUpdate_t brakePWM = {getBrakePedalPressureDuty(), PWM_BRAKE_FIXED_HZ, pwmBrake};
                xQueueSendToBack(updatePWMQueue, &brakePWM, 0);
            }
            change = true;
        }
        if (c == ']')
        {
            uint8_t currentPedalBrakeDuty = getBrakePedalPressureDuty();
            if (currentPedalBrakeDuty < 95) {
                setBrakePedalPressureDuty(currentPedalBrakeDuty + 5);
            }
            if(getPedalState()) // Brake pedal pressed, need to update brake PWM to abs controller
            {
                // Notify PWM task to update brake pwm as pedal is activated
                pwmOutputUpdate_t brakePWM = {getBrakePedalPressureDuty(), PWM_BRAKE_FIXED_HZ, pwmBrake};
                xQueueSendToBack(updatePWMQueue, &brakePWM, 0);
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
        

        if (c == 'b')
        {
            bool pedalState = getPedalState();
            if  (pedalState == 0)
            {
                setPedalState(1);
                // Start decleration task
                vTaskResume(decelerationTaskHandle);

                // Notify PWM task to update brake pwm as pedal is activated
                pwmOutputUpdate_t brakePWM = {getBrakePedalPressureDuty(), PWM_BRAKE_FIXED_HZ, pwmBrake};
                xQueueSendToBack(updatePWMQueue, &brakePWM, 0);
            } else {
                // Stop deceleration Task
                vTaskSuspend(decelerationTaskHandle);

                // Set brake pwm to 5% duty (0 pressure)
                pwmOutputUpdate_t brakePWM = {5, PWM_BRAKE_FIXED_HZ, pwmBrake};
                xQueueSendToBack(updatePWMQueue, &brakePWM, 0);

                setPedalState(0);
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
        // TO DO: maybe make this reading of pwm a critcal section so wont cause unwanted timeouts
        for (int i = 0; i<15; i++) // Could be 10, testing with 15 for safety
        {
            if(updatePWMInput(ABSPWM_ID)) // Timeout occured, ABS might be on
            {
                ABSOn = true;
                break;
            } else{
                pwmDetails = getPWMInputSignal(ABSPWM_ID);
            }
        }
        /*
        if (ABSOn)
        {
            OLEDStringDraw("ABS on ", 0, 1);
        }else{
            OLEDStringDraw("ABS off", 0, 1);
        }*/


        // NEW
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        setABSBrakePressureDuty(pwmDetails.duty);
        setABSState(ABSOn);
        // Give the mutex back
        xSemaphoreGive(carStateMutex);

        /*char string[17]; // Display fits 16 characters wide.
        sprintf(string, "D: %02ld F: %03ld", pwmDetails.duty, pwmDetails.frequency);
        OLEDStringDraw (string, 0, 3);*/


        /*char ANSIString[MAX_STR_LEN + 1]; // For uart message
        vt100_set_line_number(21);
        vt100_set_white();
        sprintf(ANSIString, "Duty: %lu  Freq %lu", pwmDetails.duty, pwmDetails.frequency);
        UARTSend (ANSIString);*/

        vTaskDelay(xDelay);
    }   
}

void decelerationTask (void* args)
{
    (void)args;
    const float maxDecel = 5; // m/s^2
    const float taskPeriodms = 50; //ms
    TickType_t wake_time = xTaskGetTickCount();     
    
    while (true)
    {
            // Wait until we can take the mutex to be able to use car state shared resource
            //while(xSemaphoreTake( carStateMutex, ( TickType_t ) 10 ) != pdTRUE) continue;
            xSemaphoreTake(carStateMutex, portMAX_DELAY);
            // We have obtained the mutex, now can run the task

            float currentSpeed = getCarSpeed();
            // TO DO: Change to getABSBrakePressureDuty when using with ABS controller 
            //(Doesnt make a difference to output but shows we actually use the ABS duty not just our own)
            uint8_t currentABSBrakeDuty = getBrakePedalPressureDuty(); // = getABSBrakePressureDuty();
            
            // Modify the speed dependant on brake pressure
            float newSpeed = currentSpeed - (float)currentABSBrakeDuty*maxDecel*taskPeriodms/1000.0/100.0;
            if (newSpeed <= 0) {
                    newSpeed = 0;
            }

            setCarSpeed(newSpeed);

            // Give the mutex back
            xSemaphoreGive(carStateMutex);

            // Tell the wheel update task to run
            xTaskNotifyGiveIndexed(updateWheelInfoHandle, 0);
            vTaskDelayUntil(&wake_time, taskPeriodms);
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


    PWMSignal_t ABSPWM = {.id = ABSPWM_ID, .gpioPort = GPIO_PORTB_BASE, .gpioPin = GPIO_PIN_0};
    registerPWMSignal(ABSPWM);

    createQueues();
    createSempahores();

    xTaskCreate(&readInputsTask, "read inputs", 150, NULL, 0, &readInputsHandle);
    xTaskCreate(&updateWheelInfoTask, "update wheel info", 256, NULL, 0, &updateWheelInfoHandle);
    xTaskCreate(&updateUARTTask, "update UART", 256, NULL, 0, &updateUARTHandle);
    xTaskCreate(&updatePWMOutputsTask, "update PWM", 256, NULL, 0, &updatePWMOutputsTaskHandle);
    xTaskCreate(&processABSPWMInputTask, "Update abs pwm input", 256, NULL, 0, NULL);
    xTaskCreate(&decelerationTask, "decelerationTask", 256, NULL, 0, &decelerationTaskHandle);
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

