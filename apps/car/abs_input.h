#ifndef ABS_INPUT_H_
#define ABS_INPUT_H_

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>


extern TaskHandle_t processABSInputSignalTaskHandle;

void initABSInput(void);
void processABSInputSignalTask(void* args);

#endif