#ifndef BUFFER_H
#define BUFFER_H

/** @file   buffer.h
    @author T. Peterson
    @date   22/08/22
    @brief  Moving average filter
*/

#include <stdio.h>
#include <stdlib.h>

//*************************************************************
// Type Definitions
//*************************************************************

typedef struct 
{
    int8_t length;
    int8_t index;
    int16_t array[];
} buffer_t;

//*************************************************************
// Function handles
//*************************************************************

/**
 * @brief Create a dynamically allocated buffer
 * 
 * @return buffer - Pointer to the created buffer
 */
extern buffer_t
*create_buffer (int16_t length);

/**
 * @brief Add a value to a buffer
 * 
 * @return bool - Represents if the addition was successful
 */
extern bool 
add_to_buffer (buffer_t *buffer, int16_t value);

/**
 * @brief Averages the current buffer contents
 * 
 * @return int - Average buffer value
 */
extern int16_t
average_buffer (buffer_t *buffer);

#endif /* BUFFER_H  */
