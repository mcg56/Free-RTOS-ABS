#ifndef ALTITUDE_H
#define ALTITUDE_H

/*****************************************************************************
 altitude.h:
 File containing all the functions related to measuring and calculating the
 altitude of the helicopter using an ADC. Initialise the ADC that is used to
 calculate the helicopter altitude by taking samples and to trigger an
 interrupt once the sample is done. The interrupt is handled by ADCIntHandler
 which places the sample in the circular buffer.
 The ADC range corresponding to the volatge range is calculated along with the
 reference point for 0% height and these are used to calculate the current
 altitude.

 Authors:  Anton Musalov - Megan Belton - Angus Eason

 Acknowledgements:
 Code from labs ~ Author Phil Bones:
 Code from ADCdemo1.c:
  -initADC
  -ADCIntHandler

 ************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include <math.h>
#include "libs/lib_adc/ap_circBufT.h"

/******************************************************************************
 Constants
 *****************************************************************************/
#define BUF_SIZE 10
#define ADC_BIT_COUNT 12
#define BOARD_VOLTAGE 3.3
#define HEIGHT_VOLTAGE_RANGE 0.8
#define MAX_HEIGHT 100
#define MIN_HEIGHT 0
#define MIN_HEIGHT_TOLERANCE 1
#define HEIGHT_STEP 10
#define MAIN_LANDING_STEP 5

/******************************************************************************
 Global variables
 *****************************************************************************/
circBuf_t g_inBuffer;
int32_t sum;



/******************************************************************************
 Functions
 *****************************************************************************/

/*****************************************************************************
 initADC: Function to initialise the ADC peripheral.
 Initialises the ADC interrupt. Configures sequence 3 to do a single sample when
 the processor sends a signal and set the interrupt flag once the sample is done.
 Enables the interrupt to be ready for use.
 ****************************************************************************/
void initADC (void);

void
updateSteering(void);


#endif /*ALTITUDE_H*/
