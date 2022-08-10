#ifndef WHEELS_H_
#define WHEELS_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "utils/ustdlib.h"


/**
 * @brief Task to update the wheel information and signal to PWM generators to update the frequencies
 * @param args Unused
 * @return No return
 */
void updateWheelInfoTask(void* args);

void toggleABSTask(void* args);

#endif