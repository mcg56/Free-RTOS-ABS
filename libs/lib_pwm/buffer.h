/** @file   buffer.h
    @author T. Peterson, J. Aitken
    @date   11/05/22
    @brief  
*/

#ifndef BUFFER_H
#define BUFFER_H

#include <stdio.h>
#include <stdlib.h>

typedef struct 
{
    int8_t length;
    int8_t index;
    int16_t array[];
} buffer_t;

extern buffer_t
*create_buffer (int16_t length);

extern bool 
add_to_buffer (buffer_t *buffer, int16_t value);

extern int16_t
average_buffer (buffer_t *buffer);

#endif /* BUFFER_H  */
