#include <Arduino.h>
#include <U8g2lib.h>
#include <menu.h>

#define MAX_MENU   10

typedef struct 
{
    uint8_t menu_id;
    const char *item_title;
    uint16_t icon;
    void (*click_handle)(uint8_t);
}menu_t;


static U8G2 u8g2;
menu_t menu[MAX_MENU];
uint8_t menu_len = 0;
uint8_t current_index = 0;
uint8_t refresh_flag = 1;



/**
 * @brief 配置U8G2
 * 
 * @param u8g2_obj 
 */
void menu_config(U8G2 &u8g2_obj)
{
    u8g2 = u8g2_obj;
}

/**
 * @brief 添加菜单
 * 
 * @param item_title 
 * @param icon 
 * @param click_handle 
 */
void menu_add_item(uint8_t menu_id, const char *item_title, uint16_t icon, void (*click_handle)(uint8_t))
{
    if (menu_len >= MAX_MENU) return ;
    menu[menu_len].menu_id = menu_id;
    menu[menu_len].item_title = item_title;
    menu[menu_len].icon = icon;
    menu[menu_len].click_handle = click_handle;
    menu_len++;
}


/**
 * @brief 菜单命令
 * 
 * @param cmd 
 */
void menu_cmd(uint8_t cmd)
{
    if (cmd == MENU_LEFT_CLICK)
    {
        
        if (current_index == 0) current_index = menu_len - 1;
        else current_index--;
        refresh_flag = 1;
    }
    else if (cmd == MENU_RIGHT_CLICK)
    {
        if (current_index >= (menu_len - 1)) current_index = 0;
        current_index++;
        refresh_flag = 1;
    } else {
        menu[current_index].click_handle(menu[current_index].menu_id);
    }
    
}


/**
 * @brief 刷新菜单
 * 
 */
void menu_refresh()
{
    refresh_flag = 1;
}

void menu_process()
{
    uint8_t last_index, next_index;
    if (!refresh_flag) return ;
    if (current_index == menu_len - 1)
    {
        // 最后一个了
        last_index = current_index - 1;
        next_index = 0;
    }
    else if (current_index == 0) 
    {
        // 第一个
        last_index = menu_len - 1;
        next_index = current_index + 1;
    } else {
        last_index = current_index - 1;
        next_index = current_index + 1;
    }
    if (refresh_flag)
    {
        refresh_flag = 0;
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_open_iconic_all_4x_t);
        u8g2.drawGlyph(50, 33, menu[current_index].icon);
        u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
        u8g2.drawGlyph(1, 32, menu[last_index].icon);
        u8g2.drawGlyph(110, 32, menu[next_index].icon);
        u8g2.setFont(u8g2_font_wqy13_t_gb2312b);
        u8g2.drawUTF8(38, 57, menu[current_index].item_title);	
        u8g2.sendBuffer();	  
    }
}