#ifndef __ADC_h
#define __ADC_h




void adc_gpio_init(void);
void adc_init(void);
uint16_t get_adc(uint8_t ch);
uint16_t get_adc_average(uint8_t ch, uint8_t times);




#endif
