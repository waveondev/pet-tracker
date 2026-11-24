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
#include "nrf_drv_gpiote.h"
#include "app_init_flash.h"
void paring_toggle(void);

void Sleep_Set(void);
static uint32_t press_time = 0;
APP_TIMER_DEF(m_button_timer);
APP_TIMER_DEF(m_led_timer);
APP_TIMER_DEF(m_bat_timer);
static volatile uint32_t button_time = 0;
void pairing_set(void);

void Regimode_set(void);
static uint32_t bat_count = 0;
typedef struct
{
    bool pairing;

    bool factory;
    bool charger;
    bool charger_full;

    bool power_en;
    bool regi_en;
    uint8_t regi_tick;
    uint8_t ack_input_tick;
    uint8_t factory_blue_tick;
    uint8_t factory_red_tick;
    uint8_t factory_green_tick;
    uint8_t factory_white_tick;
    uint8_t Comunication_connect_tick;
    uint8_t connect_fail_tick;
    uint8_t battery_green_show_tick;
    uint8_t battery_red_show_tick;

    uint8_t power_on_tick;
    uint8_t power_red_tick;

    uint8_t paring_on_tick;
    uint8_t paring_off_tick;
} led_context_t;

static volatile led_context_t m_led;
void led_regi_start(void)
{
    m_led.regi_en = true;
    m_led.regi_tick = 2;
    led_set_RGB(0,0,1);
}
void led_regi_stop(void)
{
    m_led.regi_en = false;
}


void led_ble_ack_input(void)
{
    m_led.ack_input_tick = 5;
}

void connect_fail_event(void)
{
    //m_led.connect_fail_tick = 10;
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
    {
        m_led.power_red_tick = 20;
    }
    else
    {
        m_led.power_on_tick = 10;
    }

}
void comunication_connect_event(void)
{
    m_led.Comunication_connect_tick = 20;
}
void led_battery_check_event(uint32_t level)
{
    if(level > 30)
      m_led.battery_green_show_tick= 10;
    else
      m_led.battery_red_show_tick= 10;
}

void led_factory_start(void)
{

    led_set_RGB(1,0,0);
    m_led.factory = true;
    m_led.factory_blue_tick = 15;
    m_led.factory_red_tick = 15;
    m_led.factory_green_tick = 15;
    m_led.factory_white_tick = 15;
}
void led_factory_stop(void)
{
    m_led.factory = false;
}
void led_pairing_start(uint8_t mode)
{
    //m_led.pairing = true;
    //m_led.paring_on_tick = 10;
    //m_led.paring_off_tick = 0;
}

void led_pairing_stop(void)
{
    m_led.pairing = false;
}

extern uint32_t error_flag ;

static void led_turnoff_timer_handler(void *p_context)
{ 

    static uint8_t Comunication_connect_tick_buf;
    if(m_led.Comunication_connect_tick != Comunication_connect_tick_buf)
    {
      Comunication_connect_tick_buf = m_led.Comunication_connect_tick;
      if(Comunication_connect_tick_buf == 0)
          time_data_send(NULL,0);
    }
    if(m_led.power_en)
    {
        
        if(m_led.power_on_tick)
        {
          m_led.power_on_tick--;
          led_set_RGB(1,1,1);
        }
        else if(m_led.power_red_tick)
        {
          m_led.power_red_tick--;
          led_set_RGB(1,0,0);
        }
        else
        {
          m_led.power_en = false;
          led_set_RGB(0,0,0);
        }
        return;
    }
    if(error_flag)
      return;
    if(m_led.factory)
    {
      if(m_led.factory_red_tick)
      {
        m_led.factory_red_tick--;
        led_set_RGB(1,0,0);
      }
      else if(m_led.factory_green_tick)
      {
        m_led.factory_green_tick--;
        led_set_RGB(0,1,0);
      }
      else if(m_led.factory_white_tick)
      {
        m_led.factory_white_tick--;
        led_set_RGB(1,1,1);
      }    
      else if(m_led.factory_blue_tick)
      {

        m_led.factory_blue_tick--;
        led_set_RGB(0,0,1);
      } 
      else
      {
        led_factory_stop();
        //ble_dfu_buttonless_bootloader_start_prepare();
        //
        NVIC_SystemReset();
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

    if(!nrf_gpio_pin_out_read(POWER_HOLD_PIN)) 
    {
        return;
    }

    if(m_led.regi_en)
    {
        static uint8_t regi_count = 0;
        regi_count++;
        if(m_led.regi_tick < regi_count)
        {
            regi_count = 0;
            led_all_toggle();
        }
        return;
    }


    //--------------------------------
    if(m_led.battery_green_show_tick)
    {
        m_led.battery_green_show_tick--;
        led_set_RGB(0,1,0);
        return;
    }
    if(m_led.battery_red_show_tick)
    {
        m_led.battery_red_show_tick--;
        led_set_RGB(1,0,0);
        return;
    }

    
    if(m_led.pairing)
    {
        if(m_led.paring_on_tick)
        {
          m_led.paring_on_tick--;
          if(m_led.paring_on_tick == 0)
            m_led.paring_off_tick = 10;
          led_set_RGB(1,1,0);
        }
        else if(m_led.paring_off_tick)
        {
          m_led.paring_off_tick--;
          if(m_led.paring_off_tick == 0)
            m_led.paring_on_tick = 10;
          led_set_RGB(1,0,1);
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

        led_set_RGB(1,1,1);
        
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
    if (nrf_gpio_pin_read(VBUS_IN_PIN))
      return;
    button_en = nrf_gpio_pin_read(PKEY_STAT_SW_PIN);
    NRF_LOG_INFO("PKEY_STAT_SW_PIN = %d",button_en);

    if(button_en == 0)
    {
        button_time++;
        if(button_time == FACTORY_TIME)
        {   
          //  if(nrf_gpio_pin_out_read(POWER_HOLD_PIN)) 
       
        }
        else if(button_time == POWER_TIME)
        {  
          if(nrf_gpio_pin_out_read(POWER_HOLD_PIN)) 
          {

              led_power_event();
              power_en_down(); 
          }
        }
        #if 1
        else if(button_time == PARING_TIME)
        {
          if(nrf_gpio_pin_out_read(POWER_HOLD_PIN)) 
          {
            led_pairing_stop();
            Regimode_set();
          }
        } 
        #endif
       APP_ERROR_CHECK(app_timer_start(m_button_timer,
        APP_TIMER_TICKS(1000),
        NULL));
    } 
}
static void bat_charge_timer_handler(void *p_context)
{
  if(bat_count < 10000)
  {
    bat_count++;
    fds_factory_write_bat(bat_count);
    //flash_factory_save(NULL,&bat_count);
  }
}

static void sw_gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
            ret_code_t err = NRF_SUCCESS;

            button_en = nrf_gpio_pin_read(PKEY_STAT_SW_PIN);

            if (button_en)
            {
                //app_timer_stop(m_button_timer);
                 err = app_timer_stop(m_button_timer);
                NRF_LOG_INFO("timer stop err=%d", err);
                #if 1
                if(PARING_TIME> button_time && POWER_TIME <= button_time)
                {
                  pairing_set();
                }     
                else if(button_time == 0)
                {
                      if(nrf_gpio_pin_out_read(POWER_HOLD_PIN))
                      {
                          uint32_t bat = battery_percent_get();
                          led_battery_check_event(bat);
                      }
                }
                #endif
                NRF_LOG_INFO("PKEY_STAT_SW_PIN input1");
            }
            else
            {

                button_time = 0;
                 err = app_timer_stop(m_button_timer);
                  NRF_LOG_INFO("timer stop err=%d", err);
                 err = app_timer_start(m_button_timer,
                        APP_TIMER_TICKS(1000),
                        NULL);
                NRF_LOG_INFO("timer start err=%d", err);


                //NRF_LOG_INFO("PKEY_STAT_SW_PIN input2");
            }
}
static void lsm_gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
        if (nrf_gpio_pin_read(LSM_INT_PIN))
        {
            //NRF_LOG_INFO("LSM_INT_PIN input1\r\n");
            sensor_enable();
        }
        else
        {
            //NRF_LOG_INFO("LSM_INT_PIN input2");
        }
}
static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    ret_code_t err = NRF_SUCCESS;
    NRF_LOG_INFO("VBUS interrupt: pin=%u, level=%u",
                 (unsigned int)pin,
                 (unsigned int)nrf_gpio_pin_read(pin));
#if 1
    switch(pin)
    {
        case VBUS_IN_PIN:
            if (nrf_gpio_pin_read(VBUS_IN_PIN) && nrf_gpio_pin_out_read(POWER_HOLD_PIN))
            {
                // 추가 인터럽트 차단 후 리셋
                nrfx_gpiote_in_event_disable(VBUS_IN_PIN);
                NVIC_SystemReset();
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
    nrf_gpio_pin_toggle(LED_BLUE_PIN);
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

  if(pairing_state == true) 
  {
    pairing_state = false;
    nrf_gpio_pin_clear(LED_RED_PIN);
  }
  else
  {
    pairing_state = true;
    nrf_gpio_pin_set(LED_RED_PIN);
  }
}

void gpio_timer_stop(void)
{
    app_timer_stop(m_led_timer);
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
    #if 1
    APP_ERROR_CHECK(app_timer_create(&m_button_timer,
                    APP_TIMER_MODE_SINGLE_SHOT,
                    button_press_timer_handler));
    APP_ERROR_CHECK(app_timer_create(&m_led_timer,
                    APP_TIMER_MODE_REPEATED,
                    led_turnoff_timer_handler));
    APP_ERROR_CHECK(app_timer_create(&m_bat_timer,
    APP_TIMER_MODE_REPEATED,
    bat_charge_timer_handler));


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

    uint32_t hold_target_ticks = APP_TIMER_TICKS(FACTORY_TIME*1000); // 10초(10000ms) 틱 변환
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
    // 1. 3초간 VBUS 입력 대기 시작
    uint32_t start_tick = app_timer_cnt_get();
    uint32_t wait_ticks = APP_TIMER_TICKS(200); // 3초 (3000ms)
    NRF_LOG_INFO("Waiting up to 3 seconds for VBUS connection...");

    while (1)
    {
        current_tick = app_timer_cnt_get();

        // 3. VBUS 없이 3초가 경과했는지 확인
        if (app_timer_cnt_diff_compute(current_tick, start_tick) >= wait_ticks)
        {
          break; // VBUS가 들어왔으므로 대기 루프 탈출
        }
    }
    if(flash_init() != 0)
      error_set_flag(FLASH_ERROR);
    if (nrf_gpio_pin_read(VBUS_IN_PIN))
    {
        static bool facto_en = false;
        //flash_factory_read(NULL,&bat_count);
        fds_factory_read(NULL,&bat_count);
        APP_ERROR_CHECK(app_timer_start(m_bat_timer,
                                          APP_TIMER_TICKS(5 * 60 * 1000), // 5분 (300,000 ms)
                                          NULL));

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
                      if(facto_en == false)
                      {
                          facto_en =true;
                          setting_erase_enable();
                          led_factory_start();     

                          NRF_LOG_INFO("PKEY 10s Hold Success! QC MODE ENTERED");
                      }
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
            tracker_factory();
        }
    }
#if 1
    if(nrf_gpio_pin_read(PKEY_STAT_SW_PIN) == 0)
    {
        APP_ERROR_CHECK(app_timer_start(m_button_timer,
                  APP_TIMER_TICKS(1000),
                  NULL));
    }
#endif
    start_tick = app_timer_cnt_get();
    wait_ticks = APP_TIMER_TICKS(3000); // 3초 (3000ms)
    while (1)
    {
        current_tick = app_timer_cnt_get();

        // 3. VBUS 없이 3초가 경과했는지 확인
        if (app_timer_cnt_diff_compute(current_tick, start_tick) >= wait_ticks)
        {
            NRF_LOG_INFO("3 seconds timeout reached. VBUS not detected.");
        // 2. 3초 이내에 VBUS High(1)가 감지되었는지 확인
          if (nrf_gpio_pin_read(VBUS_IN_PIN))
          {
              Sleep_Set();
              led_charger_start();
              app_timer_stop(m_button_timer);
              // QC 모드 진입
              app_timer_stop(m_led_timer);
              NRF_LOG_INFO("QC MODE ENTERED");
              app_qc_mode();
              break; // VBUS가 들어왔으므로 대기 루프 탈출
          }
          break; // VBUS가 들어왔으므로 대기 루프 탈출
        }

        // CPU 점유율 조절 또는 WFE 처리 (필요시)
        // __WFE();
    }
    led_power_event();
    power_en_up();

    nrfx_gpiote_in_config_t pek_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    pek_config.pull = NRF_GPIO_PIN_NOPULL;   // 필요시 PULLDOWN으로 변경
    APP_ERROR_CHECK(nrfx_gpiote_in_init(PKEY_STAT_SW_PIN, &pek_config, sw_gpio_handler));
    nrfx_gpiote_in_event_enable(PKEY_STAT_SW_PIN, true);

    nrfx_gpiote_in_config_t lsm_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    lsm_config.pull = NRF_GPIO_PIN_NOPULL;   // 필요시 PULLDOWN으로 변경
    APP_ERROR_CHECK(nrfx_gpiote_in_init(LSM_INT_PIN, &lsm_config, lsm_gpio_handler));
    nrfx_gpiote_in_event_enable(LSM_INT_PIN, true);
#endif
#if 1
    nrfx_gpiote_in_config_t vbus_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    vbus_config.pull = NRF_GPIO_PIN_NOPULL;   // 필요시 PULLDOWN으로 변경
    APP_ERROR_CHECK(nrfx_gpiote_in_init(VBUS_IN_PIN, &vbus_config, gpio_handler));
    nrfx_gpiote_in_event_enable(VBUS_IN_PIN, true);

    #endif
    
}    

