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
#include <timers.h>
#include <queue.h>
#include <semphr.h>

#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "SW-TM4C-2.2.0.295/inc/hw_timer.h"
#include "libs/lib_buttons/buttons.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include "stdlib.h"
#include "utils/ustdlib.h"
#include "driverlib/pwm.h"
#include "wheels.h"
#include "libs/lib_pwm/pwm_input.h"
#include "libs/lib_pwm/pwm_output.h"
#include "libs/lib_system/system.h"
#include "libs/lib_uart/uart.h"
#include "ui.h"
#include "car_state.h"
#include "car_pwm.h"

// TO DO: move into own timer module
#define ABS_TIMER_PERIPH        SYSCTL_PERIPH_TIMER3
#define ABS_TIMER_BASE          TIMER3_BASE
#define ABS_TIMER               TIMER_A
#define ABS_TIMER_CONFIG        TIMER_CFG_A_PERIODIC
#define ABS_TIMER_INT_FLAG      TIMER_TIMA_TIMEOUT
#define ABS_TIMER_DEFAULT_RATE  20 // [Hz]

#define STATUS_LED_PERIPH   SYSCTL_PERIPH_GPIOF
#define STATUS_LED_BASE     GPIO_PORTF_BASE
#define STATUS_LED_PIN      GPIO_PIN_1

//Task handles
TaskHandle_t updateWheelInfoHandle;
TaskHandle_t readInputsHandle;
TaskHandle_t updateUARTHandle;
TaskHandle_t updatePWMOutputsTaskHandle;
TaskHandle_t updateAllPWMInputsHandle;
TaskHandle_t decelerationTaskHandle;

TimerHandle_t xTimer_ReadABS;
TimerHandle_t xTimer_Delay;



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







// TO DO: change to read uart task? and move to ui.c
/**
 * @brief Reads the buttons and changes inputs accordingly
 * @param args Unused
 * @return No return
 */
void readInputsTask(void* args)
{
    (void)args; // unused
    const TickType_t xDelay = 333 / portTICK_PERIOD_MS;
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
            pwmOutputUpdate_t steeringPWM = {getSteeringDuty(), PWM_STEERING_FIXED_HZ, PWMHardwareDetailsSteering};
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
            pwmOutputUpdate_t steeringPWM = {getSteeringDuty(), PWM_STEERING_FIXED_HZ, PWMHardwareDetailsSteering};
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
                pwmOutputUpdate_t brakePWM = {getBrakePedalPressureDuty(), PWM_BRAKE_FIXED_HZ, PWMHardwareDetailsBrake};
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
                pwmOutputUpdate_t brakePWM = {getBrakePedalPressureDuty(), PWM_BRAKE_FIXED_HZ, PWMHardwareDetailsBrake};
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
                //TimerEnable(ABS_TIMER_BASE, ABS_TIMER);

                // Notify PWM task to update brake pwm as pedal is activated
                pwmOutputUpdate_t brakePWM = {getBrakePedalPressureDuty(), PWM_BRAKE_FIXED_HZ, PWMHardwareDetailsBrake};
                xQueueSendToBack(updatePWMQueue, &brakePWM, 0);
            } else {
                // Stop deceleration Task
                vTaskSuspend(decelerationTaskHandle);
                //TimerDisable(ABS_TIMER_BASE, ABS_TIMER);

                // Set brake pwm to 0% duty
                pwmOutputUpdate_t brakePWM = {0, PWM_BRAKE_FIXED_HZ, PWMHardwareDetailsBrake};
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
    const TickType_t xDelay = 250 / portTICK_PERIOD_MS;
    PWMSignal_t pwmDetails;
    uint8_t changeABSStateCount = 0;
    bool ABSOn = false;
    int NORMAL_BRAKE_HZ = 500;
    int ABS_PULE_HZ = 50;

    while (true) 
    {
        bool timeoutOccurred = false;
        // Read a whole period of the 50 Hz ABS pulse to ensure the long low signal will be read at some point
        for (int i = 0; i<(2*(NORMAL_BRAKE_HZ/ABS_PULE_HZ)); i++)

        {
            if(updatePWMInput(ABSPWM_ID)) // Timeout occured, ABS might be on
            {
                timeoutOccurred = true;
                break;

                
            } else{
                pwmDetails = getPWMInputSignal(ABSPWM_ID);
            }
        }

        // If ABS input is different to the current state (timeout while ABS off or no timeout while on)
        if (ABSOn ^ timeoutOccurred)
        {
            //changeABSStateCount++; 
            changeABSStateCount++; // Increment count
            if (changeABSStateCount >= 2) // If N number of different inputs in a row, assume state has actually changed and change ABS state
            {
                ABSOn = !ABSOn;
                changeABSStateCount = 0;
            }
        } else{ // Reset count if same input condition shows again
            changeABSStateCount = 0;
        }

        
        // if (ABSOn)
        // {
        //     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

        // }else{
        //     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
        // }


        // NEW
        xSemaphoreTake(carStateMutex, portMAX_DELAY);
        setABSBrakePressureDuty(pwmDetails.duty);
        setABSState(ABSOn);
        // Give the mutex back
        xSemaphoreGive(carStateMutex);

        /*char string[17]; // Display fits 16 characters wide.
        sprintf(string, "D: %02ld F: %03ld", pwmDetails.duty, pwmDetails.frequency);
        OLEDStringDraw (string, 0, 3);*/

        /*
        char ANSIString[MAX_STR_LEN + 1]; // For uart message
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
    const float taskPeriodms = 100; //ms
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



void vTimerCallback( TimerHandle_t xTimer )
 {
uint32_t ulMaxExpiryCountBeforeStopping;
uint32_t ulCount;
uint32_t ulCount_delay;
bool highEdgeFound = false;
static bool ABSState = false;
static uint8_t brakeOnCount = 0;
static float maxDecel = 5; // m/s^2
uint8_t currentABSBrakeDuty;

if (xTimer == xTimer_ReadABS){
    ulMaxExpiryCountBeforeStopping = 5000;
}
if (xTimer == xTimer_Delay){
    ulMaxExpiryCountBeforeStopping = 5000;
}

    /* Optionally do something if the pxTimer parameter is NULL. */
    configASSERT( xTimer );

    /* The number of times this timer has expired is saved as the
    timer's ID.  Obtain the count. */
    ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
    

    
    /* Increment the count, then test to see if the timer has expired
    ulMaxExpiryCountBeforeStopping yet. */
    ulCount++;
    
    if (xTimer == xTimer_ReadABS)
    {
        ulCount_delay = 0;
        xTimerStop( xTimer_Delay, 0);
        vTimerSetTimerID( xTimer_Delay, ( void * ) ulCount_delay );
        xTimerStart( xTimer_Delay, 0);
        uint8_t brakeOnCount = 0;
        while (( uint32_t ) pvTimerGetTimerID( xTimer_Delay ) < 5) {
            if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0)) // Found high edge, use brake duty accordingly
            {
                brakeOnCount++;
                
                if (brakeOnCount >= 4) // 4 kinda arbitrary
                {
                    ABSState = false;
                }
                highEdgeFound = true;
                OLEDStringDraw ("Off", 0, 2);
                break;
            }
    
        }
    }

    if (ulCount % 20 == 0 && xTimer == xTimer_ReadABS)
    {
        char string[17]; // Display fits 16 characters wide.
        sprintf(string, "Count = %ld", ulCount);
        OLEDStringDraw (string, 0, 1);
    
       
    }
    if (ulCount % 1 == 0 && xTimer == xTimer_Delay)
    {
        char string[17]; // Display fits 16 charactersde.

        sprintf(string, "Count = %ld", ulCount);
        OLEDStringDraw (string, 0, 3);
        
    }

    /* If the timer has expired 10 times then stop it from running. */
    if( ulCount >= ulMaxExpiryCountBeforeStopping )
    {
        /* Do not use a block time if calling a timer API function
        from a timer callback function, as doing so could cause a
        deadlock! */
        xTimerStop( xTimer, 0 );

    }
    else
    {
       /* Store the incremented count back into the timer's ID field
       so it can be read back again the next time this software timer
       expires. */
       vTimerSetTimerID( xTimer, ( void * ) ulCount );
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

    // Create and register input ABS pwm
    PWMSignal_t ABSPWM = {.id = ABSPWM_ID, .gpioPort = GPIO_PORTB_BASE, .gpioPin = GPIO_PIN_0};
    registerPWMSignal(ABSPWM);

    createQueues();
    createSempahores();

    xTimer_ReadABS = xTimerCreate
                   ( /* Just a text name, not used by the RTOS
                     kernel. */
                     "ABS Timer",
                     /* The timer period in ticks, 50ms */
                     pdMS_TO_TICKS(50) ,
                     /* The timers will auto-reload themselves
                     when they expire. */
                     pdTRUE,
                     /* The ID is used to store a count of the
                     number of times the timer has expired, which
                     is initialised to 0. */
                     ( void * ) 0,
                     /* Each timer calls the same callback when
                     it expires. */
                     vTimerCallback
                   );

    xTimer_Delay = xTimerCreate
                   ( /* Just a text name, not used by the RTOS
                     kernel. */
                     "Delay Timer",
                     /* The timer period in ticks, 1ms */
                     pdMS_TO_TICKS(2) ,
                     /* The timers will auto-reload themselves
                     when they expire. */
                     pdTRUE,
                     /* The ID is used to store a count of the
                     number of times the timer has expired, which
                     is initialised to 0. */
                     ( void * ) 0,
                     /* Each timer calls the same callback when
                     it expires. */
                     vTimerCallback
                   );

    xTimerStart( xTimer_ReadABS, 0 );
    xTimerStart( xTimer_Delay, 0 );
    

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

