#ifndef APADC_H
#define APADC_H

/*****************************************************************************
 ap_adc.c:
 File containing all the functions related to measuring and calculating the
 altitude of the helicopter using an ADC. Initialise the ADC that is used to
 calculate the helicopter altitude by taking samples. 
 The ADC range corresponding to the volatge range is calculated along with the
 reference point for 0% height and these are used to calculate the current
 altitude.

 Authors:  Anton Musalov - Megan Belton - Angus Eason
 Modifications: Anton Musalov
 Acknowledgements:
 Code from labs ~ Author Phil Bones:
 Code from ADCdemo1.c:
  -initADC


 ************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include <math.h>
#include "libs/lib_adc/circBufT.h"

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
 Initialises the ADC. Configures sequence 3 to do a single sample when
 updateSteering function is called
 ****************************************************************************/
void initADC (void);
/*****************************************************************************
 updateSteering: Function to trigger ADC reading of the potentiometer.
 The data is sent to a circular buffer where it is stored ready to be read 
 ****************************************************************************/
void updateSteering(void);


#endif /*APADC_H*/
