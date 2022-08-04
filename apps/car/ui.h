#ifndef UI_H_
#define UI_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "utils/ustdlib.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "wheels.h"

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
 * @param condition         Road Condition
 * @param pedal             Brake pedal toggle
 * @param brakePressure     Brake pressure (%)
 */
typedef struct {
    Wheel LF;
    Wheel LR;
    Wheel RF;
    Wheel RR;
    uint8_t speed; //m
    uint8_t steeringWheelDuty; //km/h
    float alpha;
    uint8_t condition;
    bool pedal;
    uint8_t brakePressure; 
} DisplayInfo;

extern QueueHandle_t OLEDDisplayQueue;
extern QueueHandle_t UARTDisplayQueue;

void vt100_set_yellow(void);
void vt100_set_white(void);
void vt100_set_line_number(int line);
void vt100_print_text(void);
void vt100_print_steering_angle(uint8_t duty, char alphaStr[6]);
void vt100_print_car_speed(uint8_t speed);
void vt100_print_wheel_speed(char LF[6],char LR[6],char RF[6],char RR[6]);
void vt100_print_radii(char LF[6],char LR[6],char RF[6],char RR[6]);
void vt100_print_prr(char LF[6],char LR[6],char RF[6],char RR[6]);
void vt100_print_brake_pressure(uint8_t pressure);
void vt100_print_pedal(bool pedal);
void vt100_print_condition(uint8_t condition);
const char* get_condition(uint8_t condition);
void vt100_print_slipage(bool slipArray[4]);

#endif

