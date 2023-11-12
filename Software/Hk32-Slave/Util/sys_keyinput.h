#ifndef _SYS_KEYINPUT_H
#define _SYS_KEYINPUT_H

typedef struct
{
    unsigned char long_press_max_leav;              // ��󳤰��ȼ�
    unsigned int long_press_time_1;                 // �ȼ�1ʱ��
    unsigned int long_press_time_2;                 // �ȼ�2ʱ��
}key_long_press_t;

typedef struct keyinput_dev
{
    unsigned char (*get_gpio_val)();            // ��ȡIO״̬
    key_long_press_t  long_press;
    void (*key_press_process)(unsigned char);                               // ��������    ��������
    void (*key_long_press_process)(unsigned char, unsigned char);           // ��������    �ﵽ�ȼ�  �������� 
    void (*key_reaease_process)(unsigned char, unsigned char);              // ��������   �Ƿ�Ϊ�����ͷ�
    unsigned int    key_timer;
    unsigned char   key_cnt;
    unsigned int    hight_cnt;
    unsigned char   low_cnt;
    unsigned char   key_press_flag;
    unsigned char   long_press_flag;
    struct keyinput_dev *keyinput_dev_next;
    
}keyinput_dev_t;





/**
 * @brief ���������豸
 * 
 * @param keynput �����豸
 */
void key_dev_create(keyinput_dev_t *keynput);


/**
 * @brief ��������
 * 
 */
void keyinput_process(void);

#endif

