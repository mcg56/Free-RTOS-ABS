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
#include "wheels.h"
#include <queue.h>
#include "libs/lib_pwm/ap_pwm.h"
#include "libs/lib_system/ap_system.h"
#include "libs/lib_uart/ap_uart.h"


typedef struct {
    uint8_t speed; //m
    uint8_t steeringWheelDuty; //km/h
} InputData;
InputData currentInput = {0, 50};

typedef struct {
    Wheel LF;
    Wheel LR;
    Wheel RF;
    Wheel RR;
    float speed; //m
    uint8_t steeringWheelDuty; //km/h
} OLEDDisplayInfo;


QueueHandle_t inputDataQueue = NULL;
QueueHandle_t OLEDDisplayQueue = NULL;

void createQueues(void)
{
    inputDataQueue = xQueueCreate(5, sizeof(InputData));
    OLEDDisplayQueue = xQueueCreate(5, sizeof(OLEDDisplayInfo));
}

TaskHandle_t updateWheelInfoHandle;
TaskHandle_t readButtonsHandle;
TaskHandle_t blinkHandle;
TaskHandle_t updateOLEDHandle;
TaskHandle_t updateUARTHandle;

extern Wheel leftFront;
extern Wheel leftRear;
extern Wheel rightFront;
extern Wheel rightRear;

float alpha = 0;

void blink(void* args) {
    (void)args; // unused

    TickType_t wake_time = xTaskGetTickCount();

    while (true) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));
        vTaskDelayUntil(&wake_time, 100);

        // configASSERT(wake_time < 1000);  // Runs vAssertCalBled() if false
    }
}

void updateOLED(void* args)
{
    (void)args; // unused
    while(true)
    {
        // Wait until a new message is to be written, as its added to queue
        OLEDDisplayInfo updatedDisplayInfo;
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
 * @brief Update the UART terminal.
 * @param args
 * @return No return
 */
void updateUART(void* args)
{
    (void)args;
    const TickType_t xDelay = 50 / portTICK_PERIOD_MS;

    char ANSIString[MAX_STR_LEN + 1]; // For uart message

    //First write static text in yellow
    sprintf (ANSIString, "%c%s", VT100_ESC, VT100_CLS);
    UARTSend (ANSIString);
    sprintf (ANSIString, "%c%s", VT100_ESC, VT100_HOME);
    UARTSend (ANSIString);
    sprintf (ANSIString, "%c%s", VT100_ESC, VT100_FG_YELLOW);
    UARTSend (ANSIString);
    UARTSend ("~~~~~~~Angus Anton Car sim outputs~~~~~~~\r\n\n");
    UARTSend ("Steering\r\n\n");
    UARTSend ("Car speed (km/h)\r\n\n");
    UARTSend ("Wheel turn radii (m)\r\n\n");
    UARTSend ("Wheel speed (km/h)\r\n\n");
    UARTSend ("Wheel PRR (Hz)\r\n\n");
    UARTSend ("Brake pressure\r\n\n");

    sprintf (ANSIString, "%c%s", VT100_ESC, VT100_FG_WHITE);
    UARTSend (ANSIString);
    while(true)
    {
        sprintf (ANSIString, "%c%s", VT100_ESC, VT100_HOME);
        UARTSend (ANSIString);

        //Steering line
        sprintf (ANSIString, "%c%s", VT100_ESC, VT100_THREE_DOWN);
        UARTSend (ANSIString);

        char alphaStr[6]; // Display fits 16 characters wide.
        gcvt (alpha, 4, &alphaStr);
        sprintf (ANSIString, "Duty: %d%%    Angle: %s degrees\r\n\n", currentInput.steeringWheelDuty, alphaStr);
        UARTSend (ANSIString);

        // Car speed line
        sprintf (ANSIString, "%d km/h\r\n\n", currentInput.speed);
        UARTSend (ANSIString);

        // Wheel turn radii line
        //First, convert floats to strings
        char LFbuff[6];
        char LRbuff[6]; 
        char RFbuff[6]; 
        char RRbuff[6];
        gcvt (leftFront.turnRadius, 4, &LFbuff);
        gcvt (leftRear.turnRadius, 4, &LRbuff);
        gcvt (rightFront.turnRadius, 4, &RFbuff);
        gcvt (rightRear.turnRadius, 4, &RRbuff);

        sprintf (ANSIString, "Lf: %s, Lr: %s, Rf: %s, Rr: %s\r\n\n", LFbuff, LRbuff, RFbuff, RRbuff);
        UARTSend (ANSIString);

        // Wheel speed line
        //First, convert floats to strings
        gcvt (leftFront.speed, 4, &LFbuff);
        gcvt (leftRear.speed, 4, &LRbuff);
        gcvt (rightFront.speed, 4, &RFbuff);
        gcvt (rightRear.speed, 4, &RRbuff);

        sprintf (ANSIString, "Lf: %s, Lr: %s, Rf: %s, Rr: %s\r\n\n", LFbuff, LRbuff, RFbuff, RRbuff);
        UARTSend (ANSIString);

        // meanVal = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
        // /* Form and send a status message to the console. Desired
        //     height/yaw is in the brackets and current is outside*/
        // sprintf (statusStr, "Raw ADC: %ld \r\nMeanVal: %d \r\n----------\r\n",
        //             sum, meanVal);
        
        // UARTSend (statusStr);
        



        vTaskDelay(xDelay);
    }
}

void updateWheelInfo(void* args)
{
    (void)args; // unused
    while(true) {
        // Wait until driving inputs change, indicated by a new value on the queue
        InputData updatedInput;
        portBASE_TYPE status = xQueueReceive(inputDataQueue, &updatedInput, 100);
        if (status == pdPASS)
        {
            alpha = calculateSteeringAngle((float)updatedInput.steeringWheelDuty);
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
                OLEDStringDraw ("Here", 0, 2);
                calculateWheelRadii(&leftRear, &leftFront, &rightRear, &rightFront, alpha);
                calculateWheelSpeedsFromRadii(&leftFront, &leftRear, &rightFront, &rightRear, updatedInput.speed);
            }
            else if (alpha > 0.0) //Turning right
            {
                calculateWheelRadii(&rightRear, &rightFront, &leftRear, &leftFront, alpha);
                calculateWheelSpeedsFromRadii(&leftFront, &leftRear, &rightFront, &rightRear, updatedInput.speed);
            }
            calculateWheelPwmFreq(&leftFront, &leftRear, &rightFront, &rightRear);
            // Wheel info updated, allow display to OLED task to run
            // Add to queue so wheel update task to run
            OLEDDisplayInfo updatedDisplayInfo = {leftFront, leftRear, rightFront, rightRear, updatedInput.speed, updatedInput.steeringWheelDuty};
            xQueueSendToBack(OLEDDisplayQueue, &updatedDisplayInfo, 0);
        }else continue;
    }
}


void readButtons(void* args)
{
    (void)args; // unused
    const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
    while (true) 
    {
        updateButtons();
        
        // Update values accodingly. No checks for value max/min limits yet
        bool change = false;
        if (checkButton(UP) == PUSHED)
        {
            currentInput.speed += 10;
            change = true;            
        }
        if (checkButton(DOWN) == PUSHED)
        {
            currentInput.speed -= 10;
            change = true;
        }
        if (checkButton(LEFT) == PUSHED)
        {
            currentInput.steeringWheelDuty -= 5;
            change = true;
        }
        if (checkButton(RIGHT) == PUSHED)
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
    initDisplay ();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
    
    initialisePWM ();
    initButtons();
    initialiseUSB_UART ();

    // Initialisation is complete, so turn on the output.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    createQueues();
    //xTaskCreate(&blink, "blink", 256, NULL, 0, &blinkHandle);
    xTaskCreate(&readButtons, "read buttons", 256, NULL, 0, &readButtonsHandle);
    xTaskCreate(&updateWheelInfo, "update wheel info", 256, NULL, 0, &updateWheelInfoHandle);
    xTaskCreate(&updateOLED, "update OLED", 256, NULL, 0, &updateOLEDHandle);
    xTaskCreate(&updateUART, "update UART", 256, NULL, 0, &updateUARTHandle);

    // Allow OLED task to run initially to give first values on screen
    //sxTaskNotifyGiveIndexed(updateOLEDHandle, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));
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

