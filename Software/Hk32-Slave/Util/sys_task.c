#include "include.h"







sys_task_t *sys_task_head = NULL;



/**
 * @brief ϵͳ����
 * 
 * @param task ����
 * @param handler ������ѯ����
 * @param interval ��ѯ���
 */
void sys_task_create(sys_task_t *task, void (*handler)(void), unsigned int interval)
{
    sys_task_t *sys_task_tail = NULL;
    memset(task, 0, sizeof(sys_task_t));
    task->enable = 0;
    task->interval = interval;
    task->tick_cnt = get_sys_tick();
    task->task_handler = handler;
    task->sys_task_next = NULL;
    if (sys_task_head == NULL)
    {
      sys_task_head = task;
			return ;
    }
    sys_task_tail = sys_task_head;
    while (sys_task_tail->sys_task_next != NULL)
    {
       sys_task_tail = sys_task_tail->sys_task_next;
    }
    sys_task_tail->sys_task_next = task;
}





/**
 * @brief ��������
 * 
 * @param task  ������
 */
void sys_task_start(sys_task_t *task)
{
    task->enable = 1;
}



/**
 * @brief ֹͣ����
 * 
 * @param task  ������
 */
void sys_task_stop(sys_task_t *task)
{
    task->enable = 0;
}


/**
 * @brief ϵͳ������ѯ
 * 
 */
void sys_task_process()
{
    sys_task_t  *task = NULL;
    for (task = sys_task_head; task != NULL; task = task->sys_task_next) 
    {
        if (task->enable && is_timeout(task->tick_cnt, task->interval))
        {
            task->task_handler();                  // ����
            task->tick_cnt =  get_sys_tick();	   
        }
    }
}
