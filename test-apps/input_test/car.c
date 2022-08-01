/*To do:
    -Read potentiometer as steering wheel and create steering PWM output accordingly
    -Generate four wheel speed output PWM signals, that vary in speed based on steering
        wheel position
    -UART/USB user interface? text based VT100 examples?
*/

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
#include "driverlib/uart.h"
#include "wheels.h"
#include "ui.h"
#include <queue.h>
#include "libs/lib_pwm/ap_pwm.h"
#include "libs/lib_system/ap_system.h"
#include "libs/lib_uart/ap_uart.h"

//Task handles
TaskHandle_t updateWheelInfoHandle;
TaskHandle_t readButtonsHandle;
TaskHandle_t blinkHandle;
TaskHandle_t updateOLEDHandle;
TaskHandle_t updateUARTHandle;
TaskHandle_t updatePWMTaskHandle;

/**
 * @brief Struture for storing input data and passing between tasks 
 * through queues
 * @param steeringWheelDuty Car steering wheel duty (%)
 * @param alpha             Turn angle (degrees)
 */
typedef struct {
    uint8_t speed; //m
    uint8_t steeringWheelDuty; //km/h
} InputData;


/**
 * @brief Struture for storing display data and passing display information 
 * between tasks through queues
 * @param LF                Left front wheel struct
 * @param LR                Left rear wheel struct                
 * @param RF                Right front wheel struct
 * @param RR                Right rear wheel struct
 * @param speed             Car speed (km/h)
 * @param steeringWheelDuty Car steering wheel duty (%)
 * @param alpha             Turn angle (degrees)
 */
typedef struct {
    Wheel LF;
    Wheel LR;
    Wheel RF;
    Wheel RR;
    uint8_t speed; //m
    uint8_t steeringWheelDuty; //km/h
    float alpha;
} DisplayInfo;


QueueHandle_t inputDataQueue = NULL;
QueueHandle_t OLEDDisplayQueue = NULL;
QueueHandle_t UARTDisplayQueue = NULL;


/**
 * @brief Creates instances of all queues
 * @return None
 */
void createQueues(void)
{
    inputDataQueue = xQueueCreate(5, sizeof(InputData));
    OLEDDisplayQueue = xQueueCreate(5, sizeof(DisplayInfo));
    UARTDisplayQueue = xQueueCreate(5, sizeof(DisplayInfo));
    updatePWMQueue = xQueueCreate(10, sizeof(pwmSignal));
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

        vt100_print_brake_pressure();
        
        
        vTaskDelayUntil(&wake_time, 100);
    }
}
/**
 * @brief Task to update the UART terminal with data about the car.
 * @param args Unused
 * @return No return
 */
void updateWheelInfoTask(void* args)
{
    (void)args; // unused
    static Wheel leftFront  = {0, 0, 0};
    static Wheel leftRear   = {0, 0, 0};
    static Wheel rightFront = {0, 0, 0};
    static Wheel rightRear  = {0, 0, 0};
    while(true) {
        // Wait until driving inputs change, indicated by a new value on the queue
        InputData updatedInput;
        portBASE_TYPE status = xQueueReceive(inputDataQueue, &updatedInput, 100);
        if (status == pdPASS)
        {
            float alpha = calculateSteeringAngle((float)updatedInput.steeringWheelDuty);
            if (alpha == 0) // Driving straight
            {
                leftFront.speed = updatedInput.speed;
                leftFront.turnRadius = 0;

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
            // Wheel info updated, signal display tasks to run via queues

            DisplayInfo updatedDisplayInfo = {leftFront, leftRear, rightFront, rightRear, updatedInput.speed, updatedInput.steeringWheelDuty, alpha};
            xQueueSendToBack(UARTDisplayQueue, &updatedDisplayInfo, 0);
            xQueueSendToBack(OLEDDisplayQueue, &updatedDisplayInfo, 0);

            /* Sending wheel pwms 1 at a time may cause issues as it updates them one at a time so abs
            controller might think its slipping whne it just hasnt updated all wheels yet*/
            pwmSignal leftFrontPWM = {WHEEL_FIXED_DUTY, (uint32_t)leftFront.pulseHz, PWMHardwareDetailsLF.base, PWMHardwareDetailsLF.gen, PWMHardwareDetailsLF.outnum};
            xQueueSendToBack(updatePWMQueue, &leftFrontPWM, 0);
        }else continue;
    }
}

/**
 * @brief Reads the buttons and changes inputs accordingly
 * @param args Unused
 * @return No return
 */
void readButtonsTask(void* args)
{
    (void)args; // unused
    const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
    static InputData currentInput = {0, 50};
    while (true) 
    {
        updateButtons();
        
        char string[17]; // Display fits 16 characters wide.
        
        int32_t c = UARTCharGetNonBlocking(UART_USB_BASE);
        
        char letter = c;
        
        sprintf(string, "Key: %c", letter);
        OLEDStringDraw ("    ",0,3);
        OLEDStringDraw (string, 0, 3);


        // Update values accodingly. No checks for value max/min limits yet
        bool change = false;
        if (checkButton(UP) == PUSHED || c == 'w')
        {
            currentInput.speed += 5;
            change = true;            
        }
        if (checkButton(DOWN) == PUSHED || c == 'q')
        {
            currentInput.speed -= 5;
            change = true;
        }
        if (checkButton(LEFT) == PUSHED || c == '1')
        
        {
            currentInput.steeringWheelDuty -= 5;
            change = true;
        }
        if (checkButton(RIGHT) == PUSHED|| c == '2')
        
        {
            currentInput.steeringWheelDuty += 5;
            change = true;
        }
        //Check if any buttons changed
        if (change){
            InputData updatedInput = {currentInput.speed, currentInput.steeringWheelDuty};
            // Add to queue so wheel update task to run
            xQueueSendToBack(inputDataQueue, &updatedInput, 0);

            
        }

        

        
        

        taskYIELD(); // Not sure if this is needed or not
        vTaskDelay(xDelay);
    }
}

int main(void) {
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    initializeCarPWM();
    OLEDInitialise ();

    // Setup red LED on PF1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    initButtons();
    initialiseUSB_UART ();

    
    turnOnAllCarPWM();

    createQueues();
    xTaskCreate(&blink, "blink", 256, NULL, 0, &blinkHandle);
    xTaskCreate(&readButtonsTask, "read buttons", 256, NULL, 0, &readButtonsHandle);
    xTaskCreate(&updateOLEDTask, "update OLED", 256, NULL, 0, &updateOLEDHandle);
    xTaskCreate(&updateWheelInfoTask, "update wheel info", 256, NULL, 0, &updateWheelInfoHandle);
    xTaskCreate(&updateUARTTask, "update UART", 256, NULL, 0, &updateUARTHandle);
    xTaskCreate(&updatePWMTask, "update PWM", 256, NULL, 0, &updatePWMTaskHandle);

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

