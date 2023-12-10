#ifndef _MENU_H
#define _MENU_H


#define MENU_LEFT_CLICK         0
#define MENU_RIGHT_CLICK        1
#define MENU_ENTER_CLICK        2


void menu_config(U8G2 &u8g2_obj);
void menu_add_item(uint8_t menu_id, const char *item_title, uint16_t icon, void (*click_handle)(uint8_t));
void menu_cmd(uint8_t cmd);
void menu_process();
void menu_refresh();


#endif