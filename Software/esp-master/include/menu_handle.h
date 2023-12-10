#ifndef _MENU_HANDLE_H
#define _MENU_HANDLE_H

void init_default_value(void);
void port1_enable(uint8_t value);
void port1_brightness(uint8_t value);
void port1_warm(uint8_t value);
void port2_enable(uint8_t value);
void port2_brightness(uint8_t value);
void port2_warm(uint8_t value);
void port3_enable(uint8_t value);
void port3_brightness(uint8_t value);
void port3_warm(uint8_t value);
void port4_enable(uint8_t value);
void port4_brightness(uint8_t value);
void port4_warm(uint8_t value);

void usb_pdout_enable(uint8_t value);
void self_starting_night(uint8_t value);
void radar_detection(uint8_t value);
void delayed_shutdown(uint8_t value);
void random_words_enable(uint8_t value);
void random_words_showtime(uint8_t value);

#endif