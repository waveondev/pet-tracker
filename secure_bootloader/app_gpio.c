#include "app_gpio.h"
#include "nrfx_gpiote.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

extern void go_system_off_rtc(void);
extern void bootloader_reset(bool do_backup);
static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch(pin)
    {
        case PKEY_STAT_SW:
            if (nrf_gpio_pin_read(PKEY_STAT_SW))
            {

                NRF_LOG_INFO("PKEY_STAT_SW input1");
            }
            else
            {
               // go_system_on();
                bootloader_reset(true);
                NRF_LOG_INFO("PKEY_STAT_SW input2");
            }
        break;
    }

}

void prepare_wake_button(void)
{
    // interrupt disable
    nrfx_gpiote_in_event_disable(PKEY_STAT_SW);

    // GPIO를 wake source로 변경
    nrf_gpio_cfg_sense_input(PKEY_STAT_SW,
                             NRF_GPIO_PIN_PULLUP,
                             NRF_GPIO_PIN_SENSE_HIGH);
}

void led_purple(void)
{
    nrf_gpio_pin_toggle(LED_BLUE);
    nrf_gpio_pin_toggle(LED_RED);
}

void go_system_off(void)
{
    nrf_gpio_pin_clear(LED_GREEN);
    nrf_gpio_pin_clear(LED_BLUE);
    nrf_gpio_pin_clear(LED_RED);
}
void go_system_on(void)
{
    nrf_gpio_pin_set(LED_GREEN);
    nrf_gpio_pin_set(LED_BLUE);
    nrf_gpio_pin_set(LED_RED);
}
#include "nrf_bootloader_app_start.h"
#include "nrf_bootloader_fw_activation.h"
#include "nrf_bootloader_dfu_timers.h"
 
void Power_hold_init(void)
{
  nrf_gpio_cfg_output(POWER_HOLD);
  nrf_gpio_pin_set(POWER_HOLD);
}
void led_callback(void)
{
    led_purple();
}
void gpio_init(void)
{
    if (!nrfx_gpiote_is_init())
    {
        nrfx_gpiote_init();
    }


    nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    config.pull = NRF_GPIO_PIN_NOPULL;   // 필요시 PULLDOWN으로 변경

    APP_ERROR_CHECK(nrfx_gpiote_in_init(PKEY_STAT_SW, &config, gpio_handler));
    nrfx_gpiote_in_event_enable(PKEY_STAT_SW, true);

    nrf_gpio_cfg_output(LED_GREEN);
    nrf_gpio_cfg_output(LED_BLUE);
    nrf_gpio_cfg_output(LED_RED);
    Power_hold_init();

    go_system_off();
    //nrf_gpio_pin_set(LED_BLUE);
    //nrf_gpio_pin_set(LED_RED);
}

