#ifndef UWBTASK_H
#define UWBTASK_H

#include <Arduino.h>

// Configuración del task UWB
#define UWB_TASK_PRIORITY 2
#define UWB_TASK_STACK_SIZE 4096
#define UWB_TASK_CORE 0  // Core 0 dedicado a UWB

// Estructura para datos compartidos entre cores
struct UWBSharedData {
    float tag_x;
    float tag_y;
    bool position_valid;
    float distances[3];
    bool anchor_status[3];
    float frequency;
    unsigned long measurement_count;
    unsigned long last_update;
    bool data_ready;
};

// Funciones públicas
void UWBTask_setup();
void UWBTask_start();
UWBSharedData* UWBTask_getSharedData();

// Task function
void uwbTaskFunction(void* parameter);

#endif // UWBTASK_H
