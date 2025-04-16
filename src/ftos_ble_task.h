// September 24, 2024
// FreeRTOS tasks for Pico W BLE project
// Allows us to use ulTaskNotifyTake() and xTaskNotifyGive() to execute 
// vehicle tasks in a separate task to allow BTstack to continue working
// its own tasks.
//
// This FreeRTOS Task Handle is needed in service_implementation.h so we have to
// get it from main.c where it is declared and defined. This FreeRTOS Task Handle
// allows xTaskNotifyGive() to notify vBLEinput_HandlerTask() in main.c that an
// event has occured. Used in the gatt functions for Characteristic_b and 
// Characteristic_e.

#ifndef FTOSTASK_H
#define FTOSTASK_H

// Get xBLEinput_HandlerTask from main.c and send to service_implementation.h
//TaskHandle_t get_xBLEinput_HandlerTask(void);
TaskHandle_t get_xBTstack_HandlerTask(void);


#endif   // FTOSTASKS_H
