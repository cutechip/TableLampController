#include <Versatile_RotaryEncoder.h>
#include "rotate.h"
#define clk 14  // (A3)
#define dt 13   // (A2)
#define sw 4   // (A4)




#define   ROTATE_BUF_MAX    64

uint8_t rotate_buf[ROTATE_BUF_MAX];
uint8_t head_index = 0;
uint8_t tail_index = 0;
uint8_t rotate_buf_cnt = 0;

void handleRotate(int8_t rotation);
void handlePressRotate(int8_t rotation);
void handleHeldRotate(int8_t rotation);
void handlePress();
void handleDoublePress();
void handlePressRelease();
void handleLongPress();
void handleLongPressRelease();
void handlePressRotateRelease();
void handleHeldRotateRelease();


Versatile_RotaryEncoder *versatile_encoder;



/**
 * @brief 添加类型
 * 
 * @param type 
 */
void put_rotate(uint8_t type)
{
    rotate_buf[head_index] = type;
    head_index++;
    if (head_index >= ROTATE_BUF_MAX) head_index = 0;
    rotate_buf_cnt++;
}


/**
 * @brief 获取数值
 * 
 * @param type 
 * @return uint8_t 
 */
uint8_t get_rotate(uint8_t *type)
{
    if (rotate_buf_cnt == 0) return 0;
    *type = rotate_buf[tail_index];
    tail_index++;
    if (tail_index >= ROTATE_BUF_MAX) tail_index = 0;
    rotate_buf_cnt--;
    return 1;
}


void rotate_init() {
    pinMode(4, INPUT);
	versatile_encoder = new Versatile_RotaryEncoder(clk, dt, sw);
    versatile_encoder->setInvertedSwitch(true);
    // Load to the encoder all nedded handle functions here (up to 9 functions)
    versatile_encoder->setHandleRotate(handleRotate);
    versatile_encoder->setHandlePressRotate(handlePressRotate);
    versatile_encoder->setHandleHeldRotate(handleHeldRotate);
    versatile_encoder->setHandlePress(handlePress);
    versatile_encoder->setHandleDoublePress(handleDoublePress);
    //versatile_encoder->setHandleDoublePress(nullptr); // Disables Double Press
    versatile_encoder->setHandlePressRelease(handlePressRelease);
    versatile_encoder->setHandleLongPress(handleLongPress);
    versatile_encoder->setHandleLongPressRelease(handleLongPressRelease);
    versatile_encoder->setHandlePressRotateRelease(handlePressRotateRelease);
    versatile_encoder->setHandleHeldRotateRelease(handleHeldRotateRelease);

    // versatile_encoder->setInvertedSwitch(true); // inverts the switch behaviour from HIGH to LOW to LOW to HIGH
    versatile_encoder->setReadIntervalDuration(1); // set 2ms as long press duration (default is 1ms)
    versatile_encoder->setShortPressDuration(35); // set 35ms as short press duration (default is 50ms)
    versatile_encoder->setLongPressDuration(1000); // set 550ms as long press duration (default is 1000ms)
    versatile_encoder->setDoublePressDuration(500); // set 350ms as double press duration (default is 250ms)

}


/**
 * @brief 编码器任务
 * 
 */
void rotate_task()
{
    if (versatile_encoder->ReadEncoder()) {
        // Do something here whenever an encoder action is read
    }
}

// Implement your functions here accordingly to your needs

void handleRotate(int8_t rotation) {
	// Serial.print("#1 Rotated: ");
    if (rotation > 0)
        put_rotate(ROTATE_RIGHT);
    else
        put_rotate(ROTATE_LEFT);
       
}

void handlePressRotate(int8_t rotation) {
	// Serial.print("#2 Pressed and rotated: ");
    if (rotation > 0)
        put_rotate(ROTATE_PRESS_RIGHT);
    else
        put_rotate(ROTATE_PRESS_LEFT);
}

void handleHeldRotate(int8_t rotation) {
	// Serial.print("#3 Held and rotated: ");
    if (rotation > 0)
        put_rotate(ROTATE_HELD_LEFT);
    else
        put_rotate(ROTATE_HELD_RIGHT);
}

void handlePress() {
	// Serial.println("#4.1 Pressed");
    put_rotate(ROTATE_PRESS);
}

void handleDoublePress() {
	// Serial.println("#4.2 Double Pressed");
    put_rotate(ROTATE_DOUBLE_PRESS);
}

void handlePressRelease() {
	// Serial.println("#5 Press released");
    put_rotate(ROTATE_RELEASED);
}

void handleLongPress() {
	// Serial.println("#6 Long pressed");
    put_rotate(ROTATE_LONG_PRESSED);
}

void handleLongPressRelease() {
	// Serial.println("#7 Long press released");
    put_rotate(ROTATE_LONG_PRESS_RELEASED);
}

void handlePressRotateRelease() {
	// Serial.println("#8 Press rotate released");
    put_rotate(ROTATE_PRESS_RELEASE);
}

void handleHeldRotateRelease() {
	// Serial.println("#9 Held rotate released");
    put_rotate(ROTATE_HELD_RELEASED);
}
