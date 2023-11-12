#ifndef _DRV_PWM_H
#define _DRV_PWM_H

void pwm_gpio_init(void);
void pwm_init(void);
void set_pwm_duty_cycle(uint8_t pwm_num, uint8_t duty);

#endif
