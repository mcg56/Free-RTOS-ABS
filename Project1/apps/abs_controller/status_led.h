#ifndef STATUS_LED_H_
#define STATUS_LED_H_

/** @file   status_led.h
    @author T. Peterson, M. Gardyne
    @date   22/08/22
    @brief  Controls the ABS status light
*/

//*************************************************************
// Type Definitions
//*************************************************************

/**
 * @brief Status LED states
 */
enum statusLEDState {BLINKING = 0, FIXED_ON, FIXED_OFF};

//*************************************************************
// Function handles
//*************************************************************

/**
 * @brief Initialise LED
 * 
 * @return None
 */
extern void
initStatusLED (void);

/**
 * @brief Set the Status LED State object
 * 
 * @return None
 */
extern void
setStatusLEDState (enum statusLEDState state);

/**
 * @brief Set the Blink Rate object
 * 
 * @param rateHz - Rate to blink the LED in Hertz
 * @return None
 */
extern void 
setStatusLEDBlinkRate (float rateHz);

#endif