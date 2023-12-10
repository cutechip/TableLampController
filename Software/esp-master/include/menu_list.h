#ifndef _MENU_LIST_H
#define _MENU_LIST_H


#define MENU_LIST_LEFT_CLICK         0
#define MENU_LIST_RIGHT_CLICK        1
#define MENU_LIST_ENTER_CLICK        2

typedef enum {
    MENU_ITEM_BOOL,
    MENU_ITEM_INT
}menu_type_t;


void menu_list_config(const char *title, U8G2 &u8g2_obj);
void menu_list_add_bool_item(const char *item_title, uint8_t value, void (*value_change_handle)(uint8_t));
void menu_list_add_uint_item(const char *item_title, uint8_t min, uint8_t max, uint8_t setp, const char *unit, uint8_t value, void (*value_change_handle)(uint8_t));
void menu_list_process(void);
void menu_list_cmd(uint8_t cmd);
void menu_list_clean(void);

#endif