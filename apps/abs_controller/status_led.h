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
//TO DO: headers
extern void
initStatusLED (void);

void
setStatusLEDState (enum statusLEDState state);

extern void 
setStatusLEDBlinkRate (float rateHz);

#endif