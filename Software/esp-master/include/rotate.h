#ifndef ROTATE_H
#define ROTATE_H


typedef enum {
    ROTATE_LEFT = 1,
    ROTATE_RIGHT,
    ROTATE_PRESS_LEFT,
    ROTATE_PRESS_RIGHT,
    ROTATE_HELD_LEFT,
    ROTATE_HELD_RIGHT,
    ROTATE_PRESS,
    ROTATE_DOUBLE_PRESS,
    ROTATE_RELEASED,
    ROTATE_LONG_PRESSED,
    ROTATE_LONG_PRESS_RELEASED,
    ROTATE_PRESS_RELEASE,
    ROTATE_HELD_RELEASED,
    ROTATE_MAX
} rotate_status_t;

void rotate_init();
void rotate_task();
uint8_t get_rotate(uint8_t *type);


#endif