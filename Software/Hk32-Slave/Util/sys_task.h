#ifndef _SYS_TASK_H
#define _SYS_TASK_H




typedef struct sys_task
{
    unsigned char enable;               // ʹ�ܱ�־
    unsigned int interval;                  // ���ʱ��
    unsigned int tick_cnt;
    void (*task_handler)();
    struct sys_task *sys_task_next;
}sys_task_t;


/**
 * @brief ϵͳ����
 * 
 * @param task ����
 * @param handler ������ѯ����
 * @param interval ��ѯ���
 */
void sys_task_create(sys_task_t *task, void (*handler)(void), unsigned int interval);





/**
 * @brief ��������
 * 
 * @param task  ������
 */
void sys_task_start(sys_task_t *task);


/**
 * @brief ֹͣ����
 * 
 * @param task  ������
 */
void sys_task_stop(sys_task_t *task);


/**
 * @brief ϵͳ������ѯ
 * 
 */
void sys_task_process(void);

#endif
