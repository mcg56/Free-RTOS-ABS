#include "user_interface.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
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

TaskHandle_t readInputsHandle;
TaskHandle_t updateUARTHandle;

void vt100_set_yellow(void) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_FG_YELLOW);
    UARTSend (ANSIString);
}

void vt100_set_white(void) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    sprintf (ANSIString, "%c%s", VT100_ESC, VT100_FG_WHITE);
    UARTSend (ANSIString);
}

void vt100_clear(void) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_HOME);
    UARTSend (ANSIString);
    sprintf(ANSIString,"%c%s", VT100_ESC, VT100_CLS);
    UARTSend (ANSIString);
    //printf("%c%s", VT100_ESC, VT100_BOLD);
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_BG_BLACK);
    UARTSend (ANSIString);
    vt100_set_yellow();
}


void vt100_set_line_number(int line) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    char buf[6] = {0};
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_HOME);
    UARTSend (ANSIString);
    sprintf(buf, "[%dB", line);
    sprintf(ANSIString, "%c%s", VT100_ESC, buf);
    UARTSend (ANSIString);
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_CLR);
    UARTSend (ANSIString);
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

void vt100_print_steering_angle(uint8_t duty, char alphaStr[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(3);
    sprintf (ANSIString, "Duty: %2d%%    Angle: %5s degrees\r\n\n", duty, alphaStr);
    UARTSend (ANSIString);
}

void vt100_print_car_speed(char speedStr[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(5);
    sprintf (ANSIString, "%5s km/h\r\n\n", speedStr);
    UARTSend (ANSIString);
}

void vt100_print_wheel_speed(char LF[6],char LR[6],char RF[6],char RR[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(7);
    sprintf (ANSIString, "Lf: %5s, Lr: %5s, Rf: %5s, Rr: %5s\r\n\n", LF, LR, RF, RR);
    UARTSend (ANSIString);
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



void vt100_print_brake_pressure(uint8_t pressure) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(13);
    sprintf(ANSIString, "%d %%", pressure);
    UARTSend (ANSIString);
}


void vt100_print_pedal(bool pedal) {
    vt100_set_line_number(15);
    if (pedal == 1)
    {
        UARTSend ("ON");
    }
    else{
        UARTSend ("OFF");
    }
}

void vt100_print_condition(uint8_t condition) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(17);
    sprintf(ANSIString, "%s", get_condition(condition));
    UARTSend (ANSIString);
}

const char* get_condition(uint8_t condition){
    
    if (condition == 0)
    {
        return "DRY";
    }
    else if (condition == 1)
    {
        return "WET";
    }
    else{
        return "ICY";
    }
}

void vt100_print_slipage(bool slipArray[4], bool ABSstate) 
{
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(19);
    char buf[4];
    if (ABSstate)
    {
        strncpy(buf, "ON", 4);
    } 
    else
    {
        strncpy(buf, "OFF", 4);
    } 
    sprintf(ANSIString, "LF: %d LR: %d RF: %d RR: %d ABS: %s", slipArray[0], slipArray[1],slipArray[2], slipArray[3], buf);
    UARTSend (ANSIString);
}

void updateUARTTask(void* args)
{
    (void)args;
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

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
        // We have obtained the mutex, now get all car state variables
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

        //Steering line
        if (steeringDuty != prevSteeringDuty) // Only write line if there was a change
        {      
            gcvt (alpha, 4, &floatBuff);
            vt100_print_steering_angle(steeringDuty, floatBuff);
            prevSteeringDuty = steeringDuty;
        }
        
        // Car speed line
        if (speed != prevSpeed) // Only write line if there was a change
        {
            gcvt (speed, 4, &floatBuff);
            vt100_print_car_speed(floatBuff);
            prevSpeed = speed;
        }
        
        // Wheel information lines
        // First get the wheel structs
        
        
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

        vTaskDelay(xDelay);
    }
}


/**
 * @brief Reads the buttons and uart input and changes car state values accordingly
 * @param args Unused
 * @return No return
 */
void readUserInputsTask(void* args)
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
            float currentSpeed = getCarSpeed(); //TO DO limit to 120
            setCarSpeed(currentSpeed + 100);
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