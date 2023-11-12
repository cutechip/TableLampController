#ifndef _SYS_KEYINPUT_H
#define _SYS_KEYINPUT_H

typedef struct
{
    unsigned char long_press_max_leav;              // 最大长按等级
    unsigned int long_press_time_1;                 // 等级1时间
    unsigned int long_press_time_2;                 // 等级2时间
}key_long_press_t;

typedef struct keyinput_dev
{
    unsigned char (*get_gpio_val)();            // 获取IO状态
    key_long_press_t  long_press;
    void (*key_press_process)(unsigned char);                               // 按键触发    按键次数
    void (*key_long_press_process)(unsigned char, unsigned char);           // 长按触发    达到等级  按键次数 
    void (*key_reaease_process)(unsigned char, unsigned char);              // 按键次数   是否为长按释放
    unsigned int    key_timer;
    unsigned char   key_cnt;
    unsigned int    hight_cnt;
    unsigned char   low_cnt;
    unsigned char   key_press_flag;
    unsigned char   long_press_flag;
    struct keyinput_dev *keyinput_dev_next;
    
}keyinput_dev_t;





/**
 * @brief 创建按键设备
 * 
 * @param keynput 按键设备
 */
void key_dev_create(keyinput_dev_t *keynput);


/**
 * @brief 按键处理
 * 
 */
void keyinput_process(void);

#endif

