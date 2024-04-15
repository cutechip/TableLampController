#include "ui.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include "humiture.h"
#include <rotate.h>
#include <menu_list.h>
#include <menu.h>
#include <menu_handle.h>
#include <ESPWiFiManager.h>
#include <wifi.h>
#include <ntp.h>
#include <ControlCenter.h>
#include <sys_config.h>
#include <NetData.h>
#include <slave.h>
#include <bat.h>


U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 10, /* data=*/ 9, /* reset=*/ U8X8_PIN_NONE);   // ESP32 Thing, pure SW emulated I2C

typedef enum {
    HOME_MAIN,
    HOME_WORD,
    HOME_MSG
}home_page_t;

typedef enum {
    PAGE_INIT,
    PAGE_HOME,
    PAGE_MENU,
    PAGE_NET_SETTING,
    PAGE_PDUSB_SETTING,
    PAGE_SYSTEM_SETTING,
    PAGE_PORT_SETTING,
    PAGE_ENGLISH_WORD
}page_t;


typedef enum {
    MENU_WIFI,      // WIFI
    MENU_PDUSB,     // PD
    MENU_SETTING,   // 系统设置
    MENU_OUTCTRL,   // 端口输出
    MENU_WORD,      // 词汇
    MENU_MAX        
}menu_t;

uint8_t rotate_type;
page_t current_page;
uint8_t menu_page = MENU_WIFI;
home_page_t home_show_mode;
uint32_t show_time_tick = 0;    // 页面停留时间计数
uint16_t show_tick = 0;
uint32_t ui_tick;
uint8_t seting_net_flage = 0;              // 配网标志
uint32_t menu_timeout_tick;
uint16_t change_over_tick = 10;          // 主页面切换计时
uint16_t home_refresh_tick = 1000;          // 主页面切换计时

word_t target_word;         // 目标显示的词汇
uint16_t word_len, chinese_len;        // 占用屏幕长度
bool wordIsRoll, chineseIsRool;
int16_t wordStartLocation = 0;                  // 英文起始坐标
int16_t chineseStartLocation = 0;               // 汉字起始坐标

bool shortmsgIsRool;
uint16_t shortmsg_len;        // 占用屏幕长度
char  shortmsg[128];
int16_t shortmsgStartLocation = 0;                  // 英文起始坐标

void ui_init()
{
    u8g2.begin();
    u8g2.enableUTF8Print();
    u8g2.setFlipMode(1);   
}

/**
 * @brief 字体占用长度
 * 
 * @param str 
 * @param word_size 
 * @return uint16_t 
 */
uint16_t count_word_len(const char *str, uint8_t ascii_size, uint8_t chinese_size)
{
    uint16_t len = strlen(str);
    uint16_t chinese_cnt = 0;
    uint16_t ascii_cnt = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        if (*(str + i) < 128){
            ascii_cnt++;
        }
        else
        {
            chinese_cnt++;
        }
    }
    chinese_cnt /= 3;
    Serial1.printf("\r\nascii_cnt:%d, chinese_cnt:%d\r\n", ascii_cnt, chinese_cnt);

    return (ascii_size * ascii_cnt) +  (chinese_size * chinese_cnt);
}

/**
 * @brief 设置短消息
 * 
 * @param msg 
 */
void set_short_message(const char *msg)
{
    memset(shortmsg, 0, sizeof(shortmsg));
    strcpy(shortmsg, msg);
    home_show_mode = HOME_MSG;

    shortmsg_len = count_word_len(shortmsg, 8, 16); 
    if (shortmsg_len > 128)  
    {
        // 需要滚动显示
        shortmsgIsRool = true;
        shortmsgStartLocation = 10;
    } else {
        // 需要居中显示
        shortmsgIsRool = false;
        shortmsgStartLocation = (128 - shortmsg_len) / 2;
    } 
    // 设置刷新速度
    home_refresh_tick = 200;        
}

uint8_t boot_animation()
{
    static uint16_t i= 0;
    char display_buff[10];
    i += 8;
	u8g2.clearBuffer();
    sprintf(display_buff,"%d%%",(int)(i/80.0*100));

    u8g2.setFont(u8g2_font_wqy16_t_gb2312b);
    u8g2.drawUTF8(30,20,"@萌了个芯");		//字符显示

    u8g2.setFont(u8g2_font_ncenB08_tf);
    u8g2.drawStr(100, 49, display_buff);				//当前进度显示

    u8g2.drawRBox(16, 40, i, 10, 4);              //圆角填充框矩形框
    u8g2.drawRFrame(16, 40, 80, 10, 4);         //圆角矩形
    u8g2.sendBuffer();
    if (i >= 80) return 1;
    else return 0;
}


const char * get_week(uint8_t week)
{
    switch (week)
    {
    case 0:
        return "星期日";
    break;
    case 1:
        return "星期一";
    break;
    case 2:
        return "星期二";
    break;
    case 3:
        return "星期三";
    break;
    case 4:
        return "星期四";
    break;
    case 5:
        return "星期五";
    break;
        
    case 6:
        return "星期六";
    break;
    }
    return "未知";
}

const char * get_weather_by_code(uint8_t code)
{
    if (code == 0 || code == 1 || code == 2 || code == 3 ) return "晴天";
    if (code == 4 || code == 5 || code == 6 || code == 7  || code == 8) return "多云";
    if (code == 9) return "阴天";
    if (code == 10 || code == 11 || code == 12) return "阵雨";
    if (code == 13) return "小雨";
    if (code == 14) return "中雨";
    if (code == 16 || code == 17 || code == 18) return "暴雨";
    if (code == 19) return "冻雨";
    if (code == 32  || code == 33 || code == 34)  return "大风";
    return "未知";
}

uint8_t get_weather_icon_by_code(uint8_t code)
{
    if (code == 0 || code == 1 || code == 2 || code == 3 ) return 0x33;
    if (code == 4 || code == 5 || code == 6 || code == 7  || code == 8) return 0x34;
    if (code == 9) return 0x35;
    if (code == 10 || code == 11 || code == 12) return 0x36;
    if (code == 13)  return 0x37;
    if (code == 14)  return 0x38;
    if (code == 16 || code == 17 || code == 18) return 0x38;
    if (code == 19)  return 0x38;
    if (code == 32  || code == 33 || code == 34)  return 0x3c;
    return 0x30;
}



/**
 * @brief 显示当前时间
 * 
 */
void show_home()
{
    char diaplay_buf[36];
    static uint8_t flag = 0; 
    uint8_t code_day = get_weather();
    u8g2.setFont(u8g2_font_siji_t_6x10);
    u8g2.clearBuffer();
    if (get_wifi_connect_status() == WIFICONFIG_CONNECT_SUCCESS) u8g2.drawGlyph(1, 12, 0xe21a);
    u8g2.drawGlyph(116, 12, 0xe242 + (get_bat_level() * 2));
    u8g2.setFont(u8g2_font_freedoomr25_tn);
    time_t timep = getNtpTime();
    struct tm *p;
    p = gmtime(&timep);
    sprintf(diaplay_buf, "%02d%c%02d", p->tm_hour, flag ? ':' : ' ' , p->tm_min);    
    u8g2.drawStr(2, 48, (const char *)diaplay_buf);
    u8g2.drawHLine(1, 16, 127);
    u8g2.drawHLine(1, 51, 95);
    u8g2.drawHLine(96, 63, 30);
    u8g2.drawVLine(96, 16, 50);
    u8g2.drawVLine(127, 16, 50);
    u8g2.drawVLine(1, 16, 50);
    u8g2.setFont(u8g2_font_unifont_t_weather);
    u8g2.drawGlyph(106, 40, get_weather_icon_by_code(code_day));
    u8g2.setFont(u8g2_font_wqy13_t_gb2312b);
    u8g2.drawUTF8(100, 60, get_weather_by_code(code_day));
    u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
    memset(diaplay_buf, 0, sizeof(diaplay_buf));
    sprintf(diaplay_buf, "温度：%.0f℃ %0.0f%c", get_temperature(), get_humidity(), '%');
    u8g2.drawUTF8(5, 63, diaplay_buf);
    u8g2.drawUTF8(50, 13, get_week(p->tm_wday));
    u8g2.sendBuffer();
    flag = !flag;
    
}








void menu_page_click(uint8_t menu_id)
{
    wifi_connect_status_t wifi_status;
    sys_config_t config = get_sys_config();
    Serial1.printf("menu_id:%d", menu_id);
    if (menu_id == MENU_WIFI)
    {
        current_page = PAGE_NET_SETTING;

        // 有可能之前就已经正在配网 
        wifi_status = get_wifi_connect_status();

        if (wifi_status == WIFICONFIG_INIT_SUCCESS || wifi_status == WIFICONFIG_RECV_WIFI_INFO) 
        {
            seting_net_flage = 1;
        }else{
            seting_net_flage = 0;
        }
        return ;
    }
    if (menu_id == MENU_PDUSB)
    {
        current_page = PAGE_PDUSB_SETTING;
        menu_list_config("快充输出", u8g2);
        menu_list_add_bool_item("打开输出", config.usbOut, usb_pdout_enable);
        return ;
    }
    if (menu_id == MENU_SETTING)
    {
        current_page = PAGE_SYSTEM_SETTING;
        
        menu_list_config("系统设置", u8g2);
        menu_list_add_bool_item("雷达感应", config.radar, radar_detection);
        menu_list_add_uint_item("延时关灯", 0, 30, 5, "S", config.delayOff, delayed_shutdown);
        return ;
    }
    if (menu_id == MENU_OUTCTRL)
    {
        current_page = PAGE_PORT_SETTING;
        menu_list_config("端口输出", u8g2);
        menu_list_add_bool_item("端口1", config.port1.sw, port1_enable);
        menu_list_add_uint_item("亮度1", 1, 100, 6, NULL, config.port1.brightness, port1_brightness);
        menu_list_add_uint_item("色温1", 1, 100, 6, NULL,  config.port1.warm, port1_warm);

        menu_list_add_bool_item("端口2", config.port2.sw, port2_enable);
        menu_list_add_uint_item("亮度2", 1, 100, 6, NULL, config.port2.brightness, port2_brightness);
        menu_list_add_uint_item("色温2", 1, 100, 6, NULL,  config.port2.warm, port2_warm);

        menu_list_add_bool_item("端口3", config.port3.sw, port3_enable);
        menu_list_add_uint_item("亮度3", 1, 100, 6, NULL, config.port3.brightness, port3_brightness);
        menu_list_add_uint_item("色温3", 1, 100, 6, NULL,  config.port2.warm, port3_warm);

        menu_list_add_bool_item("端口4", config.port3.sw, port4_enable);
        menu_list_add_uint_item("亮度4", 1, 100, 6, NULL, config.port2.brightness, port4_brightness);
        menu_list_add_uint_item("色温4", 1, 100, 6, NULL,  config.port2.warm, port4_warm);
        return ;
    }

    if (menu_id == MENU_WORD)
    {
        current_page = PAGE_ENGLISH_WORD;
        
        menu_list_config("随机单词", u8g2);
        menu_list_add_bool_item("打开", config.word, random_words_enable);
        // menu_list_add_uint_item("展示时间", 1, 10, 1, "分钟", get_random_words(1), random_words_showtime);
    }
}



/**
 * @brief 显示WIFI网络配置页面
 * 
 */
void show_wifi_config()
{
    wifi_connect_status_t status = get_wifi_connect_status();
    u8g2.setFont(u8g2_font_wqy13_t_gb2312b);
    u8g2.clearBuffer();
    if (seting_net_flage) 
    {
        // 如果之前就已经有wifi连接 那么需要断开当前wifi 不要显示出来
        if (status == WIFICONFIG_DISCONNECTED) return ;
        switch(status)
        {
            case WIFICONFIG_CONNECT_SUCCESS:
                u8g2.drawUTF8(0, 16 + 16 + 16,"3.连接成功");
            case WIFICONFIG_RECV_WIFI_INFO:
                u8g2.drawUTF8(0, 16 + 16,"2.接收到WIFI信息");
            case WIFICONFIG_INIT_SUCCESS:
                u8g2.drawUTF8(0, 16,"1.已生成配网AP");
            break;
            case WIFICONFIG_CONNECT_TIMEOUT:
                u8g2.drawUTF8(0, 32,"连接超时,请重试..");
            break;
            default:
            break;
        }
        u8g2.sendBuffer();
        return ;
    }
    u8g2.drawUTF8(10, 32,"短按进入配网");
    u8g2.sendBuffer();
}


/**
 * @brief 
 * 
 */
void display_message()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_wqy16_t_gb2312b);
    u8g2.drawUTF8(38, 16, "《消息》");
    u8g2.drawUTF8(shortmsgStartLocation, 50, shortmsg);
    u8g2.sendBuffer();
    if (shortmsgIsRool)
    {
        shortmsgStartLocation -= 10;
        if (shortmsgStartLocation <= -shortmsg_len) shortmsgStartLocation = 10;
    }
}






/**
 * @brief 显示单词
 * 
 */
void display_word()
{
    // static int8_t x1 = 10, x2 = 10;
    

    // Serial1.printf("\r\nword:%d, %d\r\n", strlen(word.word), strlen(word.chinese));
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_inb16_mf);
    u8g2.drawUTF8(wordStartLocation, 20, target_word.word);
    u8g2.setFont(u8g2_font_wqy16_t_gb2312b);
    u8g2.drawUTF8(chineseStartLocation, 54, target_word.chinese);
    u8g2.sendBuffer();
    if (wordIsRoll)
    {
        wordStartLocation -= 5;
        if (wordStartLocation <= -word_len) wordStartLocation = 10;
    }
    
    if (chineseIsRool)
    {
        chineseStartLocation -= 10;
        if (chineseStartLocation <= -chinese_len) chineseStartLocation = 10;
    }
}




/**
 * @brief 加载单词显示数据
 * 
 */
bool load_word_data()
{
    sys_config_t config = get_sys_config();
    if (!config.word)   return false;
    if (!get_net_word()) return false;

    // 设置当前显示页面的参数 显示时间 以及刷屏速度
    change_over_tick = 30000;  
    home_refresh_tick = 200;  
    home_show_mode = HOME_WORD;

    // 开始计算需要显示的位置
    target_word = get_random_word();
    Serial1.printf("word:%s", target_word.word);
    word_len = count_word_len(target_word.word, 13, 13);
    chinese_len = count_word_len(target_word.chinese, 8, 16); 
    
    if (word_len > 128)  
    {
        // 需要滚动显示
        wordIsRoll = true;
        wordStartLocation = 10;
    } else {
        // 需要居中显示
        wordIsRoll = false;
        wordStartLocation = (128 - word_len) / 2;
    }  

    if (chinese_len > 128) 
    {
        // 需要滚动显示
        chineseIsRool = true;
        chineseStartLocation = 10;
    } else {
        chineseIsRool = false;
            // 需要居中显示
        chineseStartLocation = (128 - chinese_len) / 2;
    }  

    return true;
}



/**
 * @brief 切换页面处理
 * 
 */
void change_over_process()
{
    // 屏幕显示消息
    if (millis() - show_time_tick > change_over_tick)
    {
        show_time_tick = millis();
        if (home_show_mode == HOME_MAIN) 
        {
            // 词汇未打开或初始化失败 直接进入显示消息页面
            if (load_word_data()) return ;
        }

        if (home_show_mode == HOME_WORD) 
        {
            // 设置主页面显示的参数
            home_show_mode = HOME_MAIN;
            change_over_tick = 10000; 
            home_refresh_tick = 1000;  
            return ;       
        }
    }
}


/**
 * @brief 编码器读按键处理
 * 
 */
void rotate_process()
{
    get_rotate(&rotate_type);
    if (rotate_type != 0) Serial1.printf("%d", rotate_type);
    switch (current_page)
    {
        case PAGE_HOME:
            if(rotate_type == ROTATE_RELEASED) 
            {
                // 退出主页
                if (home_show_mode != HOME_MAIN)
                {
                    // 切换回主页面
                    home_refresh_tick = 1000;
                    home_show_mode = HOME_MAIN;
                    change_over_tick = 10000;
                    show_time_tick = millis();
                   break;
                }
                // 显示菜单
                current_page = PAGE_MENU;
                menu_timeout_tick = millis();
                menu_add_item(MENU_WIFI, "网络配置", 0x00f7, menu_page_click);
                menu_add_item(MENU_PDUSB, "快充输出", 0x00ee, menu_page_click);
                menu_add_item(MENU_SETTING, "系统设置", 0x0081, menu_page_click);
                menu_add_item(MENU_OUTCTRL, "端口输出", 0x00a1, menu_page_click);
                menu_add_item(MENU_WORD, "随机单词", 0x00e2, menu_page_click);
                menu_config(u8g2);
                menu_refresh();
            }
            else if (rotate_type == ROTATE_PRESS_LEFT)
            {
                // 快速关灯
                master_switch(false);
            }
            else if (rotate_type == ROTATE_PRESS_RIGHT)
            {
                // 快速开灯
                master_switch(true);
            }
        break;
        case PAGE_MENU:
            if(rotate_type == ROTATE_LEFT) 
            {
                menu_timeout_tick = millis();
                menu_cmd(MENU_LEFT_CLICK);
            }
            if(rotate_type == ROTATE_RIGHT) 
            {
                menu_timeout_tick = millis();
                menu_cmd(MENU_RIGHT_CLICK);
            }
            if(rotate_type == ROTATE_LONG_PRESSED) 
            {
                current_page = PAGE_HOME;
            }
            if (rotate_type == ROTATE_RELEASED)
            {
                menu_cmd(MENU_ENTER_CLICK);
            }
        break;
        case PAGE_NET_SETTING:
            if (rotate_type == ROTATE_RELEASED)
            {
                // 再次短按进入配网
                if (!seting_net_flage) 
                {
                    seting_net_flage = 1;
                    auto_connect_wifi();
                } 
            }
            if (rotate_type == ROTATE_LONG_PRESSED)
            {
                // 退出页面
                current_page = PAGE_HOME;
            }
        break;
        case PAGE_PDUSB_SETTING:
        case PAGE_PORT_SETTING:
        case PAGE_SYSTEM_SETTING:
        case PAGE_ENGLISH_WORD:
            if(rotate_type == ROTATE_LEFT) 
            {
                menu_list_cmd(MENU_LIST_LEFT_CLICK);
            }
            if(rotate_type == ROTATE_RIGHT) 
            {
                menu_list_cmd(MENU_LIST_RIGHT_CLICK);
            }
            if(rotate_type == ROTATE_LONG_PRESSED) 
            {
                menu_list_clean();
                sys_congig_commit();
                current_page = PAGE_MENU;
                menu_timeout_tick = millis();
                menu_refresh();
            }
            if (rotate_type == ROTATE_RELEASED)
            {
                menu_list_cmd(MENU_LIST_ENTER_CLICK);
            }
        break;
        default:
        break;

    }
    rotate_type = 0;
}



/**
 * @brief 主页定时切换页面
 * 
 */
void home_page_process()
{
     // 刷新页面
    if ((millis() - ui_tick) > home_refresh_tick)
    {
        ui_tick = millis();
        if (home_show_mode == HOME_MAIN)
        {
            show_home();
        }
        else if (home_show_mode == HOME_WORD)
        {
            display_word();
        }
        else if (home_show_mode == HOME_MSG)
        {
            display_message();
        }
    }
}

/**
 * @brief UI显示
 * 
 */
void ui_show_task()
{


    rotate_process();
    
    switch (current_page)
    {
    case PAGE_INIT:
        if (boot_animation()) current_page = PAGE_HOME;
        break;
    case PAGE_HOME:
        change_over_process();
        home_page_process();
        // 轮询切换页面
        break;
    case PAGE_MENU:
        if (millis() - menu_timeout_tick > 8000)
        {
            current_page = PAGE_HOME;
        }
        menu_process();
        break;
    case PAGE_NET_SETTING:
        
        if ((millis() - ui_tick) > 500)
        {
            ui_tick = millis();
            show_wifi_config();
        }
    break;
    case PAGE_PDUSB_SETTING:
    case PAGE_PORT_SETTING:
    case PAGE_SYSTEM_SETTING:
    case PAGE_ENGLISH_WORD:
        menu_list_process();
        break;
    default:
        break;
    }
}