/**********************************************************
 *
 * abs_controller.c - Main controlling file for the 
 *      ABS controller.
 *
 * T.R Peterson, M.C Gardyne
 * Last modified:  24.7.22
 **********************************************************/

/**
 * General plan
 * 
 * 1. In the background have all the PWM information updating
 * 2. Do some calculations based on the PWM states to determine if the car
 *    is slipping.
 * 3. Output a brake PWM according to calculation
 * 4. Update the screen to reflect the current state/information
 * 5. Update an LED
 * 
 */

/**
 * This will contain:
 * 
 * 1. The overall main and task creation stuff
 *
 */
