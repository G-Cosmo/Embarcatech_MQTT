#ifndef JOYSTICK_QUEUE_H
#define JOYSTICK_QUEUE_H

#include "FreeRTOS.h"
#include "queue.h"

typedef struct
{
    uint16_t x_position; //nivel da agua
    uint16_t x_center;
    float x_percentual;

    uint16_t y_position; //volume de chuva
    uint16_t y_center;
    float y_percentual;

} joystick_data_t;

QueueHandle_t xQueueJoystickData;


#endif