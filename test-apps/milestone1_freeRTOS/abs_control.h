/*
 * abs_control.h
 */

#ifndef ABS_CONTROL_H_
#define ABS_CONTROL_H_

enum absStates {ABS_OFF = 0, ABS_ON};

void
setABSState (enum absStates state);

void 
toggleABSState (void);

enum absStates 
getABSState (void);

void 
updateABS (void* args);

void 
pulseABS (void* args);

#endif