
typedef void (*task_cb)();  // Callback function prototype
void Scheduler_StartTask(int period_ms, task_cb task, int priority);    //start task function prototype

typedef struct {
    TickType_t period;          // Period in ms
    TickType_t next_run_tick;   // Absolute tick when it should run next, ticks fully initialized in RTOS.cpp
    bool suspended;
    int priority;
    task_cb callback;
} taskControlBlock;

#define MAXTASKS 10
extern taskControlBlock list_of_tasks_to_do[MAXTASKS]; 
extern int num_tasks = 0;
