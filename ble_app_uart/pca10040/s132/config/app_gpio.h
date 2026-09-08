

#ifndef __APP_GPIO_H__
#define __APP_GPIO_H__
#include "stdint.h"

typedef enum
{
    LED_IDLE,

    LED_POWER_ON,
    LED_POWER_OFF,
    LED_BATTERY,

    LED_BOOT_WAIT,

    LED_PAIRING,

} led_state_t;

#include <stdbool.h>
#define CENTRAL_CONNECT 0
#define PERIPHERAL_CONNECT 1

#define PARING_TIME           6
#define POWER_TIME            3
#define FACTORY_TIME          10




#define PKEY_STAT_SW_PIN      19
#define VBUS_IN_PIN           17
#define BAT_STAT_PIN          22

#define LED_GREEN_PIN         11
#define LED_BLUE_PIN          12
#define LED_RED_PIN           13

#define POWER_HOLD_PIN        24



#define LSM_INT_PIN           7

#define LSM_SDA               25
#define LSM_SCL               26

#define BAT_ADC               2
void led_ble_ack_input(void);

void gpio_timer_stop(void);
void gpio_timer_start(void);
void Power_Hold_Check(void);
void connect_fail_event(void);
void comunication_connect_event(void);
void led_power_event(void);
void led_battery_check_event(uint32_t level);
void led_pairing_start(uint8_t mode);
void led_pairing_stop(void);
void led_boot_wait(bool enable);
void adc_enable_set(uint8_t state);
void led_all_toggle(void);
void gpio_init(void);
void prepare_wake_button(void);
void led_all_off(void);
void led_all_on(void);
void led_set_RGB(uint8_t R, uint8_t G, uint8_t B);
void power_en_up(void);
void power_en_down(void);
#endif
