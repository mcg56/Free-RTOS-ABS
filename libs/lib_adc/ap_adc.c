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
#include "ap_adc.h"


/*****************************************************************************
 initADC: Function to initialise the ADC peripheral.
 Initialises the ADC interrupt. Configures sequence 3 to do a single sample when
 the processor sends a signal and set the interrupt flag once the sample is done.
 Enables the interrupt to be ready for use.
 ****************************************************************************/
void initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 9 (ADC_CTL_CH9) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    // //
    // // Register the interrupt handler
    // ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    // //
    // // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    // ADCIntEnable(ADC0_BASE, 3);
}




void
updateSteering(void)
{
    
    uint32_t ul_value;
    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3); 
    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ul_value);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ul_value);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
 
}

