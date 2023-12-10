#include <Arduino.h>
#include <U8g2lib.h>
#include <menu_list.h>

#define MAX_MENU_NUM    20
#define PAGE_SHOW_ITEM_NUM      3

typedef struct 
{
    const char *item_title;
    menu_type_t menu_type;
    uint8_t value;              // Ĭ��ֵ
    void (*value_change_handle)(uint8_t);       // �ص�����
    uint8_t uint_min;
    uint8_t uint_max;
    uint8_t setp;
    const char *unit;
}menu_list_t;




menu_list_t menu_list[MAX_MENU_NUM];
static uint8_t menu_total = 0;                  // �˵�����
static uint8_t current_index = 0;               // ��ǰѡ������
static uint8_t menu_min_index = 0;              // ��Ҫ��ʾ����ʼ����
static uint8_t need_refresh_flag = 1;           // ����ҳ���־
static uint8_t item_checked_flag = 0;           // ��ǰ�˵�ѡ�б�־
char display_title[32];                         // �˵�����

static U8G2 u8g2;                               // u8g2������


/**
 * @brief ���ñ���
 * 
 * @param title 
 */
void menu_list_config(const char *title, U8G2 &u8g2_obj)
{
    strcpy(display_title, title);
    u8g2 = u8g2_obj;
    
}

/**
 * @brief ��Ӳ�������
 * 
 * @param item_title 
 */
void menu_list_add_bool_item(const char *item_title, uint8_t value, void (*value_change_handle)(uint8_t))
{
    menu_list_t *menu = &menu_list[menu_total++];
    menu->menu_type = MENU_ITEM_BOOL;
    menu->item_title = item_title;
    menu->value = value;
    menu->value_change_handle = value_change_handle;
}

/**
 * @brief ���uint����
 * 
 * @param item_title 
 * @param min 
 * @param max 
 */
void menu_list_add_uint_item(const char *item_title, uint8_t min, uint8_t max, uint8_t setp, const char *unit, uint8_t value, void (*value_change_handle)(uint8_t))
{
    menu_list_t *menu = &menu_list[menu_total++];
    menu->menu_type = MENU_ITEM_INT;
    menu->item_title = item_title;
    menu->uint_max = max;
    menu->uint_min = min;
    menu->unit = unit;
    menu->value = value;
    menu->setp = setp;
    menu->value_change_handle = value_change_handle;
}



/**
 * @brief ����ҳ����ʾ����
 * 
 * @param title 
 */
void page_box(const char *title)
{
    u8g2.drawUTF8(1, 16, "~");
    u8g2.drawUTF8(17, 16, title);
    // u8g2.drawFrame(0, 16,128,48);
}
void menu_list_process()
{
    uint8_t index = 0;
    uint8_t menu_location_index = 0;
    
    if (!need_refresh_flag) return ;
    need_refresh_flag = 0;
    menu_list_t *menu = nullptr;
    char index_display_buf[10];
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_wqy13_t_gb2312b);
    page_box(display_title);
    for (index = 0; index < menu_total; index++)
    {
        menu = &menu_list[index];
        if (index < menu_min_index) continue;           // Ѱ��Ҫ��ʾ��С��
        sprintf(index_display_buf, "%d.", index +1);    // ��ʾ���
        u8g2.drawUTF8(0, menu_location_index * 14 + 30, index_display_buf);
        if (index == current_index) u8g2.drawButtonUTF8(16, menu_location_index * 14 + 30, U8G2_BTN_INV, 0,  1,  1, menu->item_title);
        else u8g2.drawUTF8(16, menu_location_index * 14 + 30, menu->item_title);

        // ��ʾValue
        if (menu->menu_type == MENU_ITEM_BOOL)
        {
            if (item_checked_flag && index == current_index) 
                u8g2.drawButtonUTF8(90, menu_location_index * 14 + 30, U8G2_BTN_INV, 0,  1,  1,  menu->value ? "NO" : "OFF");
            else 
                u8g2.drawUTF8(90, menu_location_index * 14 + 30, menu->value ? "NO" : "OFF");
        }

        if (menu->menu_type == MENU_ITEM_INT)
        {
            sprintf(index_display_buf, "%d%s", menu->value, menu->unit ? menu->unit : "");
            if (item_checked_flag && index == current_index)  
                u8g2.drawButtonUTF8(90, menu_location_index * 14 + 30, U8G2_BTN_INV, 0,  1,  1,  index_display_buf);
            else 
                u8g2.drawUTF8(90, menu_location_index * 14 + 30, index_display_buf);
        }
        
        // ��ʾ�������˺��˳�
        if (++menu_location_index >= PAGE_SHOW_ITEM_NUM ) break;
    }
    u8g2.drawRBox(125, current_index * 3 + 4, 3, 60 - (menu_total * 3), 1); 
    u8g2.sendBuffer();
}




/**
 * @brief �ⲿ����
 * 
 */
void menu_list_up()
{
    // �����ѡ��״̬ ��ô����������
    if (item_checked_flag)
    {
        if (menu_list[current_index].menu_type == MENU_ITEM_BOOL)
        {
            menu_list[current_index].value = !menu_list[current_index].value;
        }
        else if (menu_list[current_index].menu_type == MENU_ITEM_INT)
        {
            menu_list[current_index].value = menu_list[current_index].value - menu_list[current_index].setp;
            if(menu_list[current_index].value < menu_list[current_index].uint_min || menu_list[current_index].value > menu_list[current_index].uint_max) 
            {
                menu_list[current_index].value = menu_list[current_index].uint_max;
            }
            
        }
        if (nullptr != menu_list[current_index].value_change_handle) 
            menu_list[current_index].value_change_handle(menu_list[current_index].value = menu_list[current_index].value);
        need_refresh_flag = 1;
        return ;
    }

    // ���������ʱ �ع�����ײ�
    if (current_index == 0) 
    {
        current_index = menu_total - 1;
        if (menu_total >= PAGE_SHOW_ITEM_NUM) menu_min_index = menu_total - PAGE_SHOW_ITEM_NUM;
        else menu_min_index = 0;
    }
    else 
    {
        current_index--;
    }
    
    // ��ҳ
    if (current_index < menu_min_index ) 
    {
        menu_min_index = current_index;
    } 
    need_refresh_flag = 1;
}


/**
 * @brief 
 * 
 */
void menu_list_down()
{

    // ͬ��
    if (item_checked_flag)
    {
        if (menu_list[current_index].menu_type == MENU_ITEM_BOOL)
        {
            menu_list[current_index].value = !menu_list[current_index].value;
        }
        else if (menu_list[current_index].menu_type == MENU_ITEM_INT)
        {
            menu_list[current_index].value = menu_list[current_index].value + menu_list[current_index].setp;
            if(menu_list[current_index].value >  menu_list[current_index].uint_max) 
            {
                menu_list[current_index].value = menu_list[current_index].uint_min;
            }
        }
        if (nullptr != menu_list[current_index].value_change_handle) 
            menu_list[current_index].value_change_handle(menu_list[current_index].value = menu_list[current_index].value);
        need_refresh_flag = 1;
        return ;
    }

    if (current_index >= menu_total - 1) 
    {
        current_index = 0;
        menu_min_index = current_index;
    }
    else 
    {
        current_index++;
    }
    if (current_index > menu_min_index + PAGE_SHOW_ITEM_NUM - 1) 
    {
        menu_min_index = current_index - PAGE_SHOW_ITEM_NUM + 1;
    }
    need_refresh_flag = 1;
}


/**
 * @brief ȷ����
 * 
 */
void menu_list_enter()
{
    // ѡ��״̬
    item_checked_flag = !item_checked_flag;
    need_refresh_flag = 1;
}

void menu_back()
{
    
}
void menu_list_cmd(uint8_t cmd)
{
    if (cmd == MENU_LIST_LEFT_CLICK)
    {
        menu_list_up();
        return ;
    }

    if (cmd == MENU_LIST_RIGHT_CLICK)
    {
        menu_list_down();
        return ;
    }

    if (cmd == MENU_LIST_ENTER_CLICK)
    {
        menu_list_enter();
        return ;
    }
}



/**
 * @brief ���ٲ˵�
 * 
 */
void menu_list_clean()
{
    menu_total = 0;
    // �´ν���ֱ��ˢ��
    need_refresh_flag = 1;
    item_checked_flag = 0;
    current_index = 0;
}