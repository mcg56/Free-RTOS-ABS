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
#include <inc/hw_ints.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>


#include <FreeRTOS.h>

#include <task.h>
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

#define PULSE_SYSCTL_PERIPH SYSCTL_PERIPH_GPIOC
#define PULSE_GPIO_PORT_BASE GPIO_PORTC_BASE
#define PULSE_GPIO_INT_PIN GPIO_INT_PIN_4
#define PULSE_GPIO_PIN GPIO_PIN_4
#define PULSE_INT SYSCTL_PERIPH_GPIOC


#define TIMER_SYSCTL_PERIPH SYSCTL_PERIPH_TIMER0
#define TIMER_BASE TIMER0_BASE
#define TIMER_MODULE TIMER_A
#define TIMER_INT INT_TIMER0A
#define TIMER_INT_TYPE TIMER_TIMA_TIMEOUT

#define MILLIS_MIN_DELTA 0.01


//Task handles
TaskHandle_t updateWheelInfoHandle;
TaskHandle_t readInputsHandle;
TaskHandle_t updateUARTHandle;
TaskHandle_t updatePWMOutputsTaskHandle;
TaskHandle_t updateAllPWMInputsHandle;
TaskHandle_t decelerationTaskHandle;


QueueHandle_t pulse_time_q;
SemaphoreHandle_t timer_timeout_sem;
QueueHandle_t timer_val_q;

SemaphoreHandle_t timer_timeout_sem;


struct timer_msg {
    uint32_t time;
    bool rising;
};

/**
 * @brief Creates instances of all queues
 * @return None
 */
void createQueues(void)
{
    updatePWMQueue = xQueueCreate(10, sizeof(pwmOutputUpdate_t));
    pulse_time_q = xQueueCreate(10, sizeof(double));
    timer_val_q = xQueueCreate(10, sizeof(struct timer_msg));

}

/**
 * @brief Creates instances of all semaphores/mutexs
 * @return None
 */
void createSempahores(void)
{
    timer_timeout_sem = xQueueCreateCountingSemaphore(100, 0);
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
    
    while (true) 
    {
        double val;
        while (xQueueReceive(pulse_time_q, &val, portMAX_DELAY) == pdTRUE){
            char string[17]; // Display fits 16 characters wide.
            sprintf(string, "Count = %.4lf", val);
            OLEDStringDraw (string, 0, 1);
        }


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



static void timer_interrupt(void)
{
    TimerIntClear(TIMER_BASE, TIMER_INT_TYPE);

    /* We don't need to yield to another task since no tasks are blocked on this
       sempahore */
    xSemaphoreGiveFromISR(timer_timeout_sem, NULL);
}


// Setup periodic countdown timer with timeout interrupt
static void timer_init(void)
{
    SysCtlPeripheralEnable(TIMER_SYSCTL_PERIPH);
    while (!SysCtlPeripheralReady(TIMER_SYSCTL_PERIPH)) {}
    TimerConfigure(TIMER_BASE, TIMER_CFG_PERIODIC);

    // For a 1 second timer period use a load value equal to timer frequency
    TimerLoadSet(TIMER_BASE, TIMER_MODULE, SysCtlClockGet());

    TimerIntRegister(TIMER_BASE, TIMER_MODULE, timer_interrupt);

    /*
     * IMPORTANT: must set the interrupt to be greater than or equal to
     * configMAX_SYSCALL_INTERRUPT_PRIORITY, otherwise will cause an assertion
     * failure
     */
    IntPrioritySet(TIMER_INT, 1);

    TimerIntEnable(TIMER_BASE, TIMER_INT_TYPE);
    IntEnable(TIMER_INT);
    TimerEnable(TIMER_BASE, TIMER_MODULE);
}

static void gpio_interrupt(void)
{
    BaseType_t higher_pri_woken = pdFALSE;

    GPIOIntClear(PULSE_GPIO_PORT_BASE, PULSE_GPIO_INT_PIN);

    const struct timer_msg msg = {
        .time = TimerValueGet(TIMER_BASE, TIMER_MODULE),
        .rising = GPIOPinRead(PULSE_GPIO_PORT_BASE, PULSE_GPIO_PIN),
    };

    xQueueSendFromISR(timer_val_q, &msg, &higher_pri_woken);

    /* If higher_pri_woken is true, this causes FreeRTOS to switch context to the
    higher priority task that was woken by writing to the queue after the ISR
    finishes. Because the pulse timer task has a higher priority than the UART
    task, we want the pulse timer task to preempt the UART task when we write to
    the queue.*/
    portYIELD_FROM_ISR(higher_pri_woken);
}

// Setup PB0 as input with pull-down and rising/falling edge interrupts
static void gpio_setup(void)
{
    SysCtlPeripheralEnable(PULSE_SYSCTL_PERIPH);
    while (!SysCtlPeripheralReady(PULSE_SYSCTL_PERIPH)) {}

    GPIOPinTypeGPIOInput(PULSE_GPIO_PORT_BASE, PULSE_GPIO_PIN);
    GPIOPadConfigSet(PULSE_GPIO_PORT_BASE, PULSE_GPIO_PIN,
        GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPD);

    GPIOIntTypeSet(PULSE_GPIO_PORT_BASE, PULSE_GPIO_PIN, GPIO_BOTH_EDGES);
    GPIOIntRegister(PULSE_GPIO_PORT_BASE, gpio_interrupt);

    // See note in timer_init
    IntPrioritySet(PULSE_INT, 1);

    GPIOIntEnable(PULSE_GPIO_PORT_BASE, PULSE_GPIO_INT_PIN);
    IntEnable(PULSE_INT);
}

static inline double lfabs(const double v)
{
    return v < 0 ? -v : v;
}

static void pulse_timer_task(void *arg)
{
    (void) arg;

    uint32_t rising_time = 0;
    double prev_millis = -1;

    while (1) {
        struct timer_msg msg;
        if (xQueueReceive(timer_val_q, &msg, portMAX_DELAY) == pdTRUE) {
            /* Found out how many times the timer overflowed by taking from the
               semaphore (non-blocking) until we cannot take anymore.*/
            uint32_t overflows = 0;
            while (xSemaphoreTake(timer_timeout_sem, 0) == pdTRUE)
                overflows++;

            if (msg.rising) {
                rising_time = msg.time;
            } else {
                /* Assume that a rising edge message will have been received
                   before this falling edge (which should be the case)*/
                uint64_t ticks;

                if (overflows) {
                    ticks = rising_time + SysCtlClockGet() - msg.time;

                    // The first overflow is a partial cycle
                    overflows -= 1;
                } else {
                    // No overflow so rising time should be higher
                    ticks = rising_time - msg.time;
                }

                // All complete timer cycles correspond to 1000 ms
                const double millis = 1000.0 *
                    (overflows + ((double) ticks) / SysCtlClockGet());

                if (lfabs(millis - prev_millis) > MILLIS_MIN_DELTA) {
                    prev_millis = millis;
                    xQueueSend(pulse_time_q, &millis, portMAX_DELAY);
                }
            }
        }
    }
}



int main(void) {
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    OLEDInitialise ();
    IntMasterDisable();

   

    // Setup red LED on PF1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);


    gpio_setup();
    timer_init();
    initButtons();
    initPWMInputManager (CAR_PWM_MIN_FREQ);

    initialiseUSB_UART ();
    initializeCarPWMOutputs();

    // Create and register input ABS pwm
    PWMSignal_t ABSPWM = {.id = ABSPWM_ID, .gpioPort = GPIO_PORTB_BASE, .gpioPin = GPIO_PIN_0};
    registerPWMSignal(ABSPWM);

    createQueues();
    createSempahores();

    
    xTaskCreate(&readInputsTask, "read inputs", 150, NULL, 0, &readInputsHandle);
    xTaskCreate(&updateWheelInfoTask, "update wheel info", 256, NULL, 0, &updateWheelInfoHandle);
    xTaskCreate(&updateUARTTask, "update UART", 256, NULL, 0, &updateUARTHandle);
    xTaskCreate(&updatePWMOutputsTask, "update PWM", 256, NULL, 0, &updatePWMOutputsTaskHandle);
    xTaskCreate(&pulse_timer_task, "pulse-timer", 256, NULL, 1, NULL);
    xTaskCreate(&processABSPWMInputTask, "Update abs pwm input", 256, NULL, 0, NULL);
    xTaskCreate(&decelerationTask, "decelerationTask", 256, NULL, 0, &decelerationTaskHandle);
    vTaskSuspend(decelerationTaskHandle);

    // Tell the wheel update task to run, which fills out the wheels speeds with starting info
    xTaskNotifyGiveIndexed(updateWheelInfoHandle, 0);

    IntMasterEnable();
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

