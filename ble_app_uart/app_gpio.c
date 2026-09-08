#include "app_gpio.h"
#include "nrfx_gpiote.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_dfu.h"
#include "app_timer.h"
#include "app_sensor.h"
#include "app_adc.h"
#include "app_qc.h"
#include "app_flash.h"
#include "app_ble_tx.h"
void pairing_set(uint8_t state);
void paring_toggle(void);

void Sleep_Set(void);
static uint32_t press_time = 0;
static uint8_t paring_mode = 0;
APP_TIMER_DEF(m_button_timer);
APP_TIMER_DEF(m_led_timer);
static volatile uint32_t button_time = 0;

typedef struct
{
    bool pairing;

    bool factory;
    bool charger;
    bool charger_full;
    bool low_bat;
    bool power_en;

    uint8_t ack_input_tick;
    uint8_t factory_blue_tick;
    uint8_t factory_red_tick;
    uint8_t factory_green_tick;
    uint8_t Comunication_connect_tick;
    uint8_t connect_fail_tick;
    uint8_t battery_green_show_tick;
    uint8_t battery_yellow_show_tick;

    uint8_t low_battery_show_count;
    uint8_t low_battery_show_tick;

    uint8_t power_green_tick;
    uint8_t power_red_tick;

    uint8_t paring_tick;
} led_context_t;

static volatile led_context_t m_led;
void led_ble_ack_input(void)
{
    m_led.ack_input_tick = 5;
}

void connect_fail_event(void)
{
    m_led.connect_fail_tick = 10;
}
void led_charger_full(void)
{
    m_led.charger_full = true;
}

void led_charger_start(void)
{
    m_led.charger = true;
    m_led.charger_full = false;
}
void led_charger_stop(void)
{
    m_led.charger = false;
}

void led_power_event(void)
{
    m_led.power_en = true;
    if(nrf_gpio_pin_out_read(POWER_HOLD_PIN)) 
        m_led.power_red_tick = 20;
    else
        m_led.power_green_tick = 10;
}
void comunication_connect_event(void)
{
    m_led.Comunication_connect_tick = 40;
}
void led_battery_check_event(uint32_t level)
{
    if(level >= 70)
      m_led.battery_green_show_tick= 10;
    else 
      m_led.battery_yellow_show_tick= 10;
}
void led_low_battery_start(void)
{
    led_set_RGB(1,0,0);
    m_led.low_bat = true;
    m_led.low_battery_show_count = 3;
    m_led.low_battery_show_tick = 10;
}
void led_low_battery_stop(void)
{
    led_set_RGB(0,0,0);
    m_led.low_bat = false;
}
void led_factory_start(void)
{
    led_set_RGB(0,0,1);
    m_led.factory = true;
    m_led.factory_blue_tick = 15;
    m_led.factory_red_tick = 15;
    m_led.factory_green_tick = 15;
}
void led_factory_stop(void)
{
    m_led.factory = false;
}
void led_pairing_start(uint8_t mode)
{
    paring_mode = mode;
 
    led_set_RGB(1,0,0);

    m_led.pairing = true;
    m_led.paring_tick = 5;
}

void led_pairing_stop(void)
{
    m_led.pairing = false;
}

static void led_turnoff_timer_handler(void *p_context)
{ 
    if(m_led.power_en)
    {
        
        if(m_led.power_green_tick)
        {
          m_led.power_green_tick--;
          //NRF_LOG_INFO("green_on\r\n");
          led_set_RGB(0,1,0);
          if(m_led.power_green_tick == 0)
           pairing_set(1);

        }
        else if(m_led.power_red_tick)
        {
          m_led.power_red_tick--;
          //NRF_LOG_INFO("red_on\r\n");
          led_set_RGB(1,0,0);
        }
        else
        {
          m_led.power_en = false;
        }
        return;
    }
    if(m_led.factory)
    {
      if(m_led.factory_blue_tick)
      {
        m_led.factory_blue_tick--;
        led_set_RGB(0,0,1);
      }
      else if(m_led.factory_red_tick)
      {
        m_led.factory_red_tick--;
        led_set_RGB(1,0,0);
      }
      else if(m_led.factory_green_tick)
      {
        m_led.factory_green_tick--;
        led_set_RGB(0,1,0);
      }
      else
      {
        led_factory_stop();
        //factory_enable();
        #if DFUMODE
         ble_dfu_buttonless_bootloader_start_prepare();
        #endif
      }
      return;
    }

  //--------------------------------
    if(m_led.charger)
    {
        if (nrf_gpio_pin_read(BAT_STAT_PIN))
        {
            led_charger_full();
        }
        if(m_led.charger_full)
          led_set_RGB(0,1,0);
        else
          led_set_RGB(1,0,0);
        return;
    }

    if(m_led.low_bat)
    {
      m_led.low_battery_show_tick--;
      if(m_led.low_battery_show_tick == 0)
      {
          m_led.low_battery_show_tick = 10;
          nrf_gpio_pin_toggle(LED_RED_PIN);
          m_led.low_battery_show_count--;
          if(m_led.low_battery_show_count == 0)
          {
            led_low_battery_stop();
          }
      }
      return;
    }

    if(m_led.pairing)
    {
        m_led.paring_tick--;
        if(m_led.paring_tick == 0)
        {
            m_led.paring_tick = 5;
            paring_toggle();
        }
        return;
    }

    




    //--------------------------------
    // Highest Priority
    //--------------------------------
    if(m_led.connect_fail_tick)
    {
        m_led.connect_fail_tick--;

        led_set_RGB(1,0,0);

        return;
    }
    if(m_led.Comunication_connect_tick)
    {
        m_led.Comunication_connect_tick--;
        if(m_led.Comunication_connect_tick == 0)
          time_data_send(NULL,0);
        led_set_RGB(0,1,0);
        
        return;
    }

    //--------------------------------
    if(m_led.battery_green_show_tick)
    {
        m_led.battery_green_show_tick--;
        led_set_RGB(0,1,0);
        return;
    }
    if(m_led.battery_yellow_show_tick)
    {
        m_led.battery_yellow_show_tick--;
        led_set_RGB(1,1,0);
        return;
    }
    if(m_led.ack_input_tick)
    {
        m_led.ack_input_tick--;
        led_set_RGB(1,1,1);
        return;
    }

    led_all_off();
}
static uint8_t button_en = 0;
static void button_press_timer_handler(void * p_context)
{
    button_en = nrf_gpio_pin_read(PKEY_STAT_SW_PIN);
    NRF_LOG_INFO("PKEY_STAT_SW_PIN = %d",button_en);
    if(button_en == 0)
    {
        button_time++;
        if(button_time == FACTORY_TIME)
        {   
            if(nrf_gpio_pin_out_read(POWER_HOLD_PIN)) 
              led_factory_start();            
        }
        else if(button_time == POWER_TIME)
        {   
          if(nrf_gpio_pin_out_read(POWER_HOLD_PIN) == 0) 
          {
              led_power_event();
              power_en_up();

          }
          else
          {
              led_power_event();
              power_en_down(); 
          }
        }
        #if 1
        else if(button_time == PARING_TIME)
        {
            if(nrf_gpio_pin_out_read(POWER_HOLD_PIN)) 
                pairing_set(2);
        }
        #endif
       APP_ERROR_CHECK(app_timer_start(m_button_timer,
        APP_TIMER_TICKS(1000),
        NULL));
    } 

}


#if 0
if(button_time >= BOOT_TIME && nrf_gpio_pin_read(VBUS_IN_PIN))
{
    ble_dfu_buttonless_bootloader_start_prepare();
}
else 
#endif
static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
#if 1
    switch(pin)
    {
        case PKEY_STAT_SW_PIN:
            button_en = nrf_gpio_pin_read(PKEY_STAT_SW_PIN);
            if (button_en)
            {
                //app_timer_stop(m_button_timer);
                 ret_code_t err = app_timer_stop(m_button_timer);
                NRF_LOG_INFO("timer stop err=%d", err);
                #if 1
                if(button_time == 0)
                {
                      if(nrf_gpio_pin_out_read(POWER_HOLD_PIN))
                      {
                          uint32_t bat = battery_percent_get();

                          if(bat >= 70)
                          {
                              led_battery_check_event(bat);
                          }
                          else if(bat >= 30)
                          {
                              led_battery_check_event(bat);
                          }
                          else
                          {
                              led_low_battery_start();
                          }
                      }
                }
                #endif
                                NRF_LOG_INFO("PKEY_STAT_SW_PIN input1");
            }
            else
            {
                button_time = 0;
                  APP_ERROR_CHECK(app_timer_start(m_button_timer,
                        APP_TIMER_TICKS(1000),
                        NULL));

                //NRF_LOG_INFO("PKEY_STAT_SW_PIN input2");
            }
            
        break;
        case VBUS_IN_PIN:
            if (nrf_gpio_pin_read(VBUS_IN_PIN))
            {
                NVIC_SystemReset();
            }
        break;
        case LSM_INT_PIN:
            if (nrf_gpio_pin_read(LSM_INT_PIN))
            {
                //NRF_LOG_INFO("LSM_INT_PIN input1\r\n");
                sensor_enable();
            }
            else
            {
                //NRF_LOG_INFO("LSM_INT_PIN input2");
            }
        break;
    }
    #endif

}



void power_en_down(void)
{
    nrf_gpio_pin_clear(POWER_HOLD_PIN);
}
void power_en_up(void)
{
    nrf_gpio_pin_set(POWER_HOLD_PIN);
}
void led_all_toggle(void)
{
    nrf_gpio_pin_toggle(LED_GREEN_PIN);
    nrf_gpio_pin_toggle(LED_BLUE_PIN);
    nrf_gpio_pin_toggle(LED_RED_PIN);
}

void led_all_off(void)
{
    nrf_gpio_pin_clear(LED_GREEN_PIN);
    nrf_gpio_pin_clear(LED_BLUE_PIN);
    nrf_gpio_pin_clear(LED_RED_PIN);
}
void led_all_on(void)
{
    nrf_gpio_pin_set(LED_GREEN_PIN);
    nrf_gpio_pin_set(LED_BLUE_PIN);
    nrf_gpio_pin_set(LED_RED_PIN);
}
void led_set_RGB(uint8_t R, uint8_t G, uint8_t B)
{
    if(R)
      nrf_gpio_pin_set(LED_RED_PIN);
    else
      nrf_gpio_pin_clear(LED_RED_PIN);

    if(G)
      nrf_gpio_pin_set(LED_GREEN_PIN);
    else
      nrf_gpio_pin_clear(LED_GREEN_PIN);

    if(B)
      nrf_gpio_pin_set(LED_BLUE_PIN);
    else
      nrf_gpio_pin_clear(LED_BLUE_PIN);
}

void paring_toggle(void)
{
  static bool pairing_state = false;
  if(paring_mode == CENTRAL_CONNECT)
  {
    if(pairing_state == true) 
    {
      pairing_state = false;
      nrf_gpio_pin_clear(LED_RED_PIN);
      nrf_gpio_pin_set(LED_BLUE_PIN);
    }
    else
    {
      pairing_state = true;
      nrf_gpio_pin_set(LED_RED_PIN);
      nrf_gpio_pin_clear(LED_BLUE_PIN);
    }
    nrf_gpio_pin_clear(LED_GREEN_PIN);
  }
  else
  {
    if(pairing_state == true) 
    {
      pairing_state = false;
      nrf_gpio_pin_clear(LED_RED_PIN);
      nrf_gpio_pin_set(LED_GREEN_PIN);
    }
    else
    {
      pairing_state = true;
      nrf_gpio_pin_set(LED_RED_PIN);
      nrf_gpio_pin_clear(LED_GREEN_PIN);
    }
    nrf_gpio_pin_clear(LED_BLUE_PIN);
  }
}

void gpio_timer_stop(void)
{
    app_timer_stop(m_led_timer);
}


#include "nrf_pwr_mgmt.h"
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}



void gpio_init(void)
{
#if DFUMODE
    nrfx_gpiote_out_uninit(PKEY_STAT_SW_PIN);
    nrfx_gpiote_in_uninit(PKEY_STAT_SW_PIN);
#endif

    if (!nrfx_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrfx_gpiote_init());
    }
    APP_ERROR_CHECK(app_timer_create(&m_button_timer,
                    APP_TIMER_MODE_SINGLE_SHOT,
                    button_press_timer_handler));
    APP_ERROR_CHECK(app_timer_create(&m_led_timer,
                    APP_TIMER_MODE_REPEATED,
                    led_turnoff_timer_handler));

    nrf_gpio_cfg_output(LED_GREEN_PIN);
    nrf_gpio_cfg_output(LED_BLUE_PIN);
    nrf_gpio_cfg_output(LED_RED_PIN);
    nrf_gpio_cfg_output(POWER_HOLD_PIN);

    led_all_off();

    power_en_down();
    //power_en_up();
    APP_ERROR_CHECK(app_timer_start(m_led_timer,
    APP_TIMER_TICKS(100),
    NULL));

    uint32_t hold_target_ticks = APP_TIMER_TICKS(10000); // 10초(10000ms) 틱 변환
    uint32_t check_interval_ticks = APP_TIMER_TICKS(1000); // 1초 간격 로그용

    volatile bool is_pressing = false;
    volatile uint32_t press_start_tick = 0;
    volatile uint32_t last_log_tick = 0;
    uint32_t current_tick = 0;
    bool button_pressed = false;
    NRF_LOG_INFO("PKEY 10s Hold Check Start...");
    nrf_gpio_cfg_input(BAT_STAT_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(PKEY_STAT_SW_PIN,NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(VBUS_IN_PIN,NRF_GPIO_PIN_NOPULL);
    if (nrf_gpio_pin_read(VBUS_IN_PIN))
    {
        Sleep_Set();
        led_charger_start();
  
        while (1)
        {

            current_tick = app_timer_cnt_get();
            button_pressed = (nrf_gpio_pin_read(PKEY_STAT_SW_PIN) == 0);

            #if 1
            if (nrf_gpio_pin_read(PKEY_STAT_SW_PIN) == 0)
            {

                if (!is_pressing)
                {
                    is_pressing = true;
                    press_start_tick = current_tick;
                    last_log_tick = current_tick;
                    NRF_LOG_INFO("PKEY Pressed! Holding...");
                }
                else
                {
                    uint32_t elapsed_ticks = app_timer_cnt_diff_compute(current_tick, press_start_tick);

                    if (app_timer_cnt_diff_compute(current_tick, last_log_tick) >= check_interval_ticks)
                    {
                        last_log_tick = current_tick;
                        uint32_t elapsed_sec = elapsed_ticks / APP_TIMER_TICKS(1000);
                        NRF_LOG_INFO("Holding... %lu / 10 sec", elapsed_sec);
                    }

                    if (elapsed_ticks >= hold_target_ticks)
                    {
                        //led_set_RGB(0,0,1);
                        app_timer_stop(m_led_timer);
                                        //led_charger_full();
                        NRF_LOG_INFO("PKEY 10s Hold Success! QC MODE ENTERED");
                        app_qc_mode();
                    }
                }
            }
            else
            {
                if (is_pressing)
                {
                    NRF_LOG_INFO("PKEY Released! Resetting hold timer.");
                    is_pressing = false;
                }
            }
            #endif
        }
    }


    nrfx_gpiote_in_config_t pek_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    pek_config.pull = NRF_GPIO_PIN_NOPULL;   // 필요시 PULLDOWN으로 변경
    APP_ERROR_CHECK(nrfx_gpiote_in_init(PKEY_STAT_SW_PIN, &pek_config, gpio_handler));
    nrfx_gpiote_in_event_enable(PKEY_STAT_SW_PIN, true);

    nrfx_gpiote_in_config_t lsm_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    lsm_config.pull = NRF_GPIO_PIN_NOPULL;   // 필요시 PULLDOWN으로 변경
    APP_ERROR_CHECK(nrfx_gpiote_in_init(LSM_INT_PIN, &lsm_config, gpio_handler));
    nrfx_gpiote_in_event_enable(LSM_INT_PIN, true);

    nrfx_gpiote_in_config_t vbus_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    vbus_config.pull = NRF_GPIO_PIN_NOPULL;   // 필요시 PULLDOWN으로 변경
    APP_ERROR_CHECK(nrfx_gpiote_in_init(VBUS_IN_PIN, &vbus_config, gpio_handler));
    nrfx_gpiote_in_event_enable(VBUS_IN_PIN, true);

    if(nrf_gpio_pin_read(PKEY_STAT_SW_PIN) == 0)
    {
        APP_ERROR_CHECK(app_timer_start(m_button_timer,
                  APP_TIMER_TICKS(1000),
                  NULL));
    }
}    

