/** @file   buffer.c
    @author T. Peterson
    @date   22/08/22
    @brief  Moving average filter
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "buffer.h"

/**
 * @brief Create a dynamically allocated buffer
 * 
 * @return buffer - Pointer to the created buffer
 */
extern buffer_t
*create_buffer (int16_t length)
{
    buffer_t *buffer = malloc(sizeof(buffer_t) + length * sizeof(int16_t));
    buffer->length = length;
    buffer->index = 0;
    for (int i = 0; i < length; i++) buffer->array[i] = -1;

    return buffer;
}

/**
 * @brief Add a value to a buffer
 * 
 * @return bool - Represents if the addition was successful
 */
extern bool 
add_to_buffer (buffer_t *buffer, int16_t value)
{
    buffer->array[buffer->index] = value;
    buffer->index++;

    if (buffer->index >= buffer->length)
        buffer->index = 0;

    return true;
}

/**
 * @brief Averages the current buffer contents
 * 
 * @return int - Average buffer value
 */
extern int16_t
average_buffer (buffer_t *buffer)
{
    int32_t sum = 0;
    int16_t len = 0;

    for (int i = 0; i < buffer->length; i++)
    {
        len++;
        if (buffer->array[i] == -1)
            break;

        sum += buffer->array[i];
    }

    return sum/len;
}
