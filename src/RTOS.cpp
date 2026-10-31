#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "scheduler_support.hpp"
#include <algorithm>


namespace WebServer {
    void WSbegin();
    void WSupdate();
}


void Scheduler_StartTask(int period_ms, int priority, task_cb task) {
    if (num_tasks < MAXTASKS) {
        list_of_tasks_to_do[num_tasks].priority = priority;
        
        TickType_t period_ticks = pdMS_TO_TICKS(period_ms);
        list_of_tasks_to_do[num_tasks].period = period_ticks;
        // Schedule it to run after its first period, or immediately (current_tick)
        list_of_tasks_to_do[num_tasks].next_run_tick = xTaskGetTickCount() + period_ticks;
        list_of_tasks_to_do[num_tasks].suspended = false;
        list_of_tasks_to_do[num_tasks].callback = task;
        num_tasks++;

        std::sort(list_of_tasks_to_do, list_of_tasks_to_do + num_tasks, 
            [](const taskControlBlock& a, const taskControlBlock& b) {
                return a.priority < b.priority; // Ascending order: 0 comes first
            });
    }
}

void Scheduler_Init() {
    WebServer::WSbegin();//runs once, to start server

    Scheduler_StartTask(100, 0, []() { WebServer::WSupdate(); });   //get update from web server
    //scheduler_StartTask(period, 1, IMU Update);
    //scheduler_StartTask(period, 2, MotorMath);
    //scheduler_StartTask(period, 3, StoreMotorMath);
    //scheduler_StartTask(period, 4, ApplyMotorMath);

}


void scheduler() { 
    // Fetch current tick dynamically inside the loop
    TickType_t current_tick = xTaskGetTickCount();  //ESP32 internal timer tick

    for (int i = 0; i < num_tasks; i++) {
        if (list_of_tasks_to_do[i].suspended) {
            continue; // Skip suspended tasks
        }

        // Check if the target tick has arrived or passed
        if ((int32_t)(current_tick - list_of_tasks_to_do[i].next_run_tick) >= 0) {
            
            // Execute the task callback safely
            if (list_of_tasks_to_do[i].callback != nullptr) {
                list_of_tasks_to_do[i].callback();
            }

            // Set the next target run time (prevents drift using absolute offsets)
            list_of_tasks_to_do[i].next_run_tick = current_tick + list_of_tasks_to_do[i].period;
        }
    }

}