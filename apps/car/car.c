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
TaskHandle_t readInputsHandle;
TaskHandle_t blinkHandle;
TaskHandle_t updateUARTHandle;
TaskHandle_t updatePWMOutputsTaskHandle;
TaskHandle_t updateAllPWMInputsHandle;
TaskHandle_t updateDecelHandle;
TaskHandle_t toggleABSTaskHandle; // NEW

/**
 * @brief Creates instances of all queues
 * @return None
 */
void createQueues(void)
{
    OLEDDisplayQueue = xQueueCreate(5, sizeof(DisplayInfo));
    UARTDisplayQueue = xQueueCreate(5, sizeof(DisplayInfo));
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
 * @brief Update the UART terminal with data about the car.
 * @param args Unused
 * @return No return
 */
void updateUARTTask(void* args)
{
    (void)args;
   
    // // Create empty ine array to clear lines
    // char empytLine[MAX_STR_LEN + 1];
    // for(int i = 0; i < MAX_STR_LEN; i++)
    // {
    //     empytLine[i] = ' ';
    // }
    // empytLine[sizeof(empytLine)/sizeof(empytLine[0]) - 1] = '\r'; // Put last index as carridge return
    const TickType_t xDelay = 10 / portTICK_PERIOD_MS;

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
        
        //char string[17]; // Display fits 16 characters wide.
        
        

        // Wait until we can take the mutex to be able to use car state shared resource
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        // We have obtained the mutex, now can run the task

        int32_t c = UARTCharGetNonBlocking(UART_USB_BASE);
        // Update values accodingly.
        bool change = false;

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
            // Notify PWM task to update steering PWM to new value
            pwmOutputUpdate_t steeringPWM = {getSteeringDuty(), PWM_STEERING_FIXED_HZ, pwmSteering};
            xQueueSendToBack(updatePWMQueue, &steeringPWM, 0);
            change = true;
        }
        if (checkButton(RIGHT) == PUSHED|| c == '2')
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
                vTaskResume(updateDecelHandle);

                // Notify PWM task to update brake pwm as pedal is activated
                pwmOutputUpdate_t brakePWM = {getBrakePedalPressureDuty(), PWM_BRAKE_FIXED_HZ, pwmBrake};
                xQueueSendToBack(updatePWMQueue, &brakePWM, 0);
            } else {
                // Stop deceleration Task
                vTaskSuspend(updateDecelHandle);

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
        if (ABSOn)
        {
            OLEDStringDraw("ABS on ", 0, 1);
        }else{
            OLEDStringDraw("ABS off", 0, 1);
        }


        // NEW
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        setABSBrakePressureDuty(pwmDetails.duty);
        setABSState(ABSOn);
        // Give the mutex back
        xSemaphoreGive(carStateMutex);

        char string[17]; // Display fits 16 characters wide.
        sprintf(string, "D: %02ld F: %03ld", pwmDetails.duty, pwmDetails.frequency);
        OLEDStringDraw (string, 0, 3);


        /*char ANSIString[MAX_STR_LEN + 1]; // For uart message
        vt100_set_line_number(21);
        vt100_set_white();
        sprintf(ANSIString, "Duty: %lu  Freq %lu", pwmDetails.duty, pwmDetails.frequency);
        UARTSend (ANSIString);*/

        vTaskDelay(xDelay);
    }   
}

void updateDecel (void* args)
{
    (void)args;

    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS; // Need to match this to the ABS Duty
    
    while (true)
    {
            // Wait until we can take the mutex to be able to use car state shared resource
            //while(xSemaphoreTake( carStateMutex, ( TickType_t ) 10 ) != pdTRUE) continue;
            xSemaphoreTake(carStateMutex, portMAX_DELAY);
            // We have obtained the mutex, now can run the task

            uint8_t currentSpeed = getCarSpeed();
            // TO DO: Change to getABSBrakePressureDuty when using with ABS controller 
            //(Doesnt make a difference to output but shows we actually use the ABS duty not just our own)
            uint8_t currentABSBrakeDuty = getBrakePedalPressureDuty(); // = getABSBrakePressureDuty();
            
            // Modify the speed dependant on brake pressure
            int newSpeed = currentSpeed - currentABSBrakeDuty/10;
            if (newSpeed <= 0) {
                    newSpeed = 0;
            }

            setCarSpeed((uint8_t)newSpeed);

            // Give the mutex back
            xSemaphoreGive(carStateMutex);

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

    initButtons();
    initPWMInputManager (CAR_PWM_MIN_FREQ);

    initialiseUSB_UART ();
    initializeCarPWMOutputs();


    PWMSignal_t ABSPWM = {.id = ABSPWM_ID, .gpioPort = GPIO_PORTB_BASE, .gpioPin = GPIO_PIN_0};
    registerPWMSignal(ABSPWM);

    createQueues();
    createSempahores();

    //xTaskCreate(&blink, "blink", 150, NULL, 0, &blinkHandle);
    xTaskCreate(&readInputsTask, "read inputs", 150, NULL, 0, &readInputsHandle);
    xTaskCreate(&updateWheelInfoTask, "update wheel info", 256, NULL, 0, &updateWheelInfoHandle);
    xTaskCreate(&updateUARTTask, "update UART", 256, NULL, 0, &updateUARTHandle);
    xTaskCreate(&updatePWMOutputsTask, "update PWM", 256, NULL, 0, &updatePWMOutputsTaskHandle);
    xTaskCreate(&processABSPWMInputTask, "Update abs pwm input", 256, NULL, 0, NULL);
    xTaskCreate(&updateDecel, "updateDecel", 256, NULL, 0, &updateDecelHandle);
    xTaskCreate(toggleABSTask, "Toggle ABS", 50, NULL, 0, &toggleABSTaskHandle);
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

