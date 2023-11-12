#ifndef _SYS_TASK_H
#define _SYS_TASK_H




typedef struct sys_task
{
    unsigned char enable;               // 使能标志
    unsigned int interval;                  // 间隔时间
    unsigned int tick_cnt;
    void (*task_handler)();
    struct sys_task *sys_task_next;
}sys_task_t;


/**
 * @brief 系统任务
 * 
 * @param task 任务
 * @param handler 任务轮询函数
 * @param interval 轮询间隔
 */
void sys_task_create(sys_task_t *task, void (*handler)(void), unsigned int interval);





/**
 * @brief 启动任务
 * 
 * @param task  任务句柄
 */
void sys_task_start(sys_task_t *task);


/**
 * @brief 停止任务
 * 
 * @param task  任务句柄
 */
void sys_task_stop(sys_task_t *task);


/**
 * @brief 系统任务轮询
 * 
 */
void sys_task_process(void);

#endif
