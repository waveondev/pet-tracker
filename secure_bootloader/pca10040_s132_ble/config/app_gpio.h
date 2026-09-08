

#ifndef __APP_GPIO_H__
#define __APP_GPIO_H__




#define PKEY_STAT_SW      19
#define VBUS_IN           17
#define BAT_STAT          22


#define LED_GREEN         11
#define LED_BLUE          12
#define LED_RED           13



#define POWER_HOLD        24


void Power_hold_init(void);
void gpio_init(void);
void prepare_wake_button(void);
void led_purple(void);
void go_system_off(void);
void go_system_on(void);
void led_callback(void);
#endif
