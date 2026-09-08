

#ifndef __APP_GPIO_H__
#define __APP_GPIO_H__




#define PKEY_STAT_SW      14
#define VBUS_IN           13
#define BAT_STAT          17


#define LED_GREEN         8
#define LED_BLUE          11
#define LED_RED           12



#define POWER_HOLD        20


#define BAT_ADC_ENABLE    19
void gpio_init(void);
void prepare_wake_button(void);
void led_purple(void);
void go_system_off(void);
void go_system_on(void);
void led_callback(void);
void Power_hold_init(void);
#endif
