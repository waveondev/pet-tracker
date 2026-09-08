#include "app_adc.h"
#include "nrfx_saadc.h"
#include "nrf_drv_saadc.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_gpio.h"
#include "nrf_pwr_mgmt.h"

static nrf_saadc_value_t adc_buf;
static volatile int16_t adc_value;
static uint32_t bat_level = 0;
static uint32_t v_bat_mv = 0;
void LowBat_State(void);
uint8_t battery_percent_stable(uint8_t new_percent)
{
    static uint8_t last_percent = 0;

    if (new_percent > last_percent)
    {
        if ((new_percent - last_percent) >= 2)
            last_percent = new_percent;
    }
    else
    {
        if ((last_percent - new_percent) >= 2)
            last_percent = new_percent;
    }

    return last_percent;
}
uint8_t battery_percent(uint16_t voltage_mv)
{
    int32_t percent;

    if (voltage_mv >= 4200)
        return 100;

    if (voltage_mv <= 3400)
        return 0;

    percent = ((int32_t)(voltage_mv - 3400) * 100 + 400) / 800;

    return (uint8_t)percent;
}
uint32_t battery_voltage_get(void)
{
    return v_bat_mv;
}
uint32_t battery_percent_get(void)
{
    return bat_level;
}
uint16_t saadc_timer_handler(void * p_context)
{
    ret_code_t err;
    nrf_saadc_value_t value;

    saadc_init();
    err = nrfx_saadc_sample_convert(0, &value);

    adc_value = value;
    if (err == NRF_SUCCESS)
    {
      uint32_t pin_voltage = ((uint32_t)adc_value * 2400) / 1024;

      // 2. 분배 저항 비율이 50% 분배라면 원본 배터리 전압은 2배를 곱함
      v_bat_mv = pin_voltage * 2; 


       
       bat_level = battery_percent_stable(battery_percent(v_bat_mv));
       NRF_LOG_INFO("Real Battery: %d%% (RAW:%d)\r\n", battery_percent_stable(battery_percent(v_bat_mv)), adc_value);
       NRF_LOG_INFO("adc %dmv(%d)\r\n", v_bat_mv,adc_value); 
    }
    else
    {
        NRF_LOG_ERROR("ADC fail: 0x%08X", err);
    }
    saadc_deinit();
    LowBat_State();
    return adc_value;
}
#if 0
3.4v - 260
3.5v - 267
3.6v - 272
3.7v - 279

#endif

void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        adc_value = p_event->data.done.p_buffer[0];
        ret_code_t err = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        if (err != NRF_SUCCESS)
        {
            // 에러 체크 (APP_ERROR_CHECK는 인터럽트 안이라 위험하므로 로그나 디버깅용으로 처리)
        }
    }
}

#if 0

NRF_SAADC_GAIN1_6 (1/6배)$0.6\text{V} / (1/6) = 0.6 \times 6$$0\text{V} \sim 3.6\text{V}$ (배터리/VDD 측정용으로 가장 많이 씀)
NRF_SAADC_GAIN1_5 (1/5배)$0.6\text{V} / (1/5) = 0.6 \times 5$$0\text{V} \sim 3.0\text{V}$
NRF_SAADC_GAIN1_4 (1/4배)$0.6\text{V} / (1/4) = 0.6 \times 4$$0\text{V} \sim 2.4\text{V}$
NRF_SAADC_GAIN1_3 (1/3배)$0.6\text{V} / (1/3) = 0.6 \times 3$$0\text{V} \sim 1.8\text{V}$
NRF_SAADC_GAIN1_2 (1/2배)$0.6\text{V} / (1/2) = 0.6 \times 2$$0\text{V} \sim 1.2\text{V}$
NRF_SAADC_GAIN1 (1배)$0.6\text{V} / 1$$0\text{V} \sim 0.6\text{V}$
NRF_SAADC_GAIN2 (2배)$0.6\text{V} / 2$$0\text{V} \sim 0.3\text{V}$ (미세 전압 측정용)
NRF_SAADC_GAIN4 (4배)$0.6\text{V} / 4$$0\text{V} \sim 0.15\text{V}$
#endif
void LowBat_State(void)
{
#if !DEBUG
    if(battery_voltage_get() < 3300)
    {
      NRF_LOG_INFO("low bat %dmv(%d)\r\n", v_bat_mv,adc_value); 
  // 안전을 위해 사용 중이던 SAADC나 디바이스들을 언이닛(Uninit) 해주면 전력 소모를 더 줄일 수 있습니다.
      nrfx_saadc_uninit();
      led_all_off();

       power_en_down();
      // 1. 깨어날 조건(Wake-up Source)을 따로 설정하지 않고 바로 System OFF로 진입합니다.
      // 이렇게 하면 하드웨어 리셋(Reset 핀)이나 배터리를 뺐다 새로 끼우기 전까지 절대 깨어나지 않습니다.
       nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
#endif
}
#include "nrfx_gpiote.h"
void saadc_deinit(void)
{
  nrf_saadc_disable();
    nrfx_saadc_uninit();
    NRF_SAADC->ENABLE = 0;
    *(volatile uint32_t *)((uint32_t)NRF_SAADC + 0xFFC) = 0;
    *(volatile uint32_t *)((uint32_t)NRF_SAADC + 0xFFC);
    nrf_gpio_cfg_default(BAT_ADC);
}
void saadc_init(void)
{
   ret_code_t err;

    nrfx_saadc_config_t config = NRFX_SAADC_DEFAULT_CONFIG;
    // 가끔 읽을 때는 로우파워 모드가 오히려 방해될 수 있으므로 기본값 사용 추천
    config.low_power_mode = false; 
    NRF_SAADC->ENABLE = 1;
    *(volatile uint32_t *)((uint32_t)NRF_SAADC + 0xFFC) = 1;
    *(volatile uint32_t *)((uint32_t)NRF_SAADC + 0xFFC);
    //  1. 핵심: 콜백 함수 자리에 NULL을 넣습니다.
    err = nrfx_saadc_init(&config, saadc_callback);
    APP_ERROR_CHECK(err);

    nrf_saadc_channel_config_t ch_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    ch_config.reference = NRF_SAADC_REFERENCE_INTERNAL; 
    ch_config.gain      = NRF_SAADC_GAIN1_4;            // 0 ~ 1.8V 범위
    ch_config.acq_time  = NRF_SAADC_ACQTIME_40US;       // 147k 고임피던스 대응

    err = nrfx_saadc_channel_init(0, &ch_config);
    APP_ERROR_CHECK(err);


}




