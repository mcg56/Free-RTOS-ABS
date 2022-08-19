#ifndef STATUS_LED_H_
#define STATUS_LED_H_

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