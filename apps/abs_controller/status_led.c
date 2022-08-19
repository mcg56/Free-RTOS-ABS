/**********************************************************
 *
 * status_led.c - Main controlling file for Tiva display
 * 
 * T.R Peterson, M.C Gardyne
 * Last modified:  19.8.22
 **********************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "status_led.h"

//*************************************************************
// Constant Definitions
//*************************************************************
#define DEFAULT_BLINK_RATE  500                 // [ms]

#define STATUS_LED_PERIPH   SYSCTL_PERIPH_GPIOF
#define STATUS_LED_BASE     GPIO_PORTF_BASE
#define STATUS_LED_PIN      GPIO_PIN_1

//*************************************************************
// Function prototypes
//*************************************************************
static void blinkTask           (void* args);
static void setBlinkDelayTask   (void* args);  
static void toggleLED           (void);
static void setBlinkDelay       (TickType_t delay);
static void ledOn               (void);
static void ledOff              (void);

//*****************************************************************************
// freeRTOS handles
//*****************************************************************************
TaskHandle_t    blinkLEDHandle;
QueueHandle_t   blinkRateQueue;

//*****************************************************************************
// Static variables
//*****************************************************************************
static TickType_t blinkTaskDelay = DEFAULT_BLINK_RATE / portTICK_PERIOD_MS;

/**
 * @brief Initialise LED
 * 
 * @return None
 */
void
initStatusLED (void)
{
    SysCtlPeripheralEnable(STATUS_LED_PERIPH);
    GPIOPinTypeGPIOOutput(STATUS_LED_BASE, STATUS_LED_PIN);

    blinkRateQueue = xQueueCreate(5, sizeof(TickType_t));

    xTaskCreate(&blinkTask, "blinkLED", 256, NULL, 0, &blinkLEDHandle);
    xTaskCreate(&setBlinkDelayTask, "setBlinkDelay", 256, NULL, 0, NULL);
}

/**
 * @brief Set the Status LED State object
 * 
 * @return None
 */
void
setStatusLEDState (enum statusLEDState state)
{
    switch(state)
    {
        case BLINKING:
            vTaskResume(blinkLEDHandle);
            break;
        case FIXED_ON:
            vTaskSuspend(blinkLEDHandle);
            ledOn();
            break;
        case FIXED_OFF:
            vTaskSuspend(blinkLEDHandle);
            ledOff();
            break; 
    }
}

/**
 * @brief Task to blink the LED a certain rate
 * 
 * @return None
 */
static void 
blinkTask(void* args) 
{
    (void)args; // unused
    
    while (true) {
        toggleLED();

        vTaskDelay(blinkTaskDelay);
    }
}

/**
 * @brief Set the Blink Rate object
 * 
 * @param rateHz - Rate to blink the LED in Hertz
 * @return None
 */
void 
setStatusLEDBlinkRate (float rateHz)
{
    // setBlinkTaskDelay((1000 / rateHz) / portTICK_PERIOD_MS);
    TickType_t delay = ((1000 / rateHz) / portTICK_PERIOD_MS);
    
    xQueueSendToBack(blinkRateQueue, &delay, 0);
}

/**
 * @brief Task to set the LED blink task delay
 * 
 * @return None
 */
static void 
setBlinkDelayTask(void* args) 
{
    (void)args; // unused
    TickType_t delay;

    while (true) 
    {
        if (xQueueReceive(blinkRateQueue, &delay, portMAX_DELAY) == pdPASS)
        { 
            setBlinkDelay (delay);
        }
    }
}

/**
 * @brief Set the Blink Task Delay object
 * 
 * @param delay - Number of ticks to delay
 * @return None
 */
static void
setBlinkDelay (TickType_t delay)
{
    blinkTaskDelay = delay;
}

/**
 * @brief Toggles the LED on/off
 * 
 * @return None
 */
static void
toggleLED (void)
{
    GPIOPinWrite(STATUS_LED_BASE, STATUS_LED_PIN, ~GPIOPinRead(STATUS_LED_BASE, STATUS_LED_PIN));
}

/**
 * @brief Set LED on
 * 
 * @return None
 */
static void
ledOn (void)
{
    GPIOPinWrite(STATUS_LED_BASE, STATUS_LED_PIN, STATUS_LED_PIN);
}

/**
 * @brief Set LED off
 * 
 * @return None
 */
static void
ledOff (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, STATUS_LED_PIN, ~STATUS_LED_PIN);
}