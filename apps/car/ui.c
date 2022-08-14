#include "ui.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "libs/lib_uart/ap_uart.h"
#include "driverlib/uart.h"
#include "car_state.h"

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