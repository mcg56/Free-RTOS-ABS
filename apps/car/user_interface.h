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

extern TaskHandle_t processUserInputsTaskHandle;
extern TaskHandle_t updateUARTTaskHandle;

void vt100_print_text(void);
void vt100_print_radii(char LF[6],char LR[6],char RF[6],char RR[6]);
void vt100_print_prr(char LF[6],char LR[6],char RF[6],char RR[6]);
void vt100_print_condition(uint8_t condition);
const char* get_condition(uint8_t condition);


void processUserInputsTask(void* args);

/**
 * @brief Update the UART terminal with data about the car.
 * @param args Unused
 * @return No return
 */
void updateUARTTask(void* args);

#endif

