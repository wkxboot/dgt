#ifndef  __WATCH_DOG_TASK_H__
#define  __WATCH_DOG_TASK_H__


extern osThreadId   watch_dog_task_hdl;
void watch_dog_task(void const * argument);


#define  WATCH_DOG_TASK_INTERVAL_VALUE               200












#endif