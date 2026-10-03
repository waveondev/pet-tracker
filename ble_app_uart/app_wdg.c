#include "app_wdg.h"
#include "nrf_log.h"
#include "nrfx_wdt.h"
#include "nrf_log_ctrl.h"
#include "app_gpio.h"
// WDT 인스턴스 핸들 및 채널 ID
static nrfx_wdt_channel_id m_wdt_channel_id;
#include "app_timer.h"
APP_TIMER_DEF(m_wdg_timer);
/**
 * @brief WDT 타임아웃 발생 시 호출되는 이벤트 핸들러
 * @note 타임아웃 직전 리셋되기 전에 1회 호출되며, 복잡한 로직이나 지연 처리는 금지됩니다.
 */
static void wdt_event_handler(void)
{
    NRF_LOG_FINAL_FLUSH();
        power_en_down();
    // 타임아웃 발생 시 리셋되기 전 로그 남김 또는 안전 상태 처리
}
static void wdg_clear(void *p_context)
{
    nrfx_wdt_channel_feed(m_wdt_channel_id);
}

#include "nrf.h"
#include "nrf_power.h"
#include "nrf_soc.h"

static void power_failure_warning_init(void)
{
    // POF 활성화
    // VDD가 약 2.7V 이하로 내려가면 POFWARN 이벤트 발생

    
    sd_power_pof_enable(NRF_POWER_POFTHR_V26);
    // POWER_CLOCK IRQ 활성화
   // NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
   // NVIC_SetPriority(POWER_CLOCK_IRQn, 6);
    // NVIC_EnableIRQ(POWER_CLOCK_IRQn);
}

static void power_failure_warning_handler(void)
{
    /*
     * 전원 저하 발생
     *
     * 여기서는 최대한 빠르게 처리해야 함.
     */

    power_en_down();

    /*
     * MCU reset
     */
    NVIC_SystemReset();
}
/**
 * @brief WDT 초기화 함수
 */
void wdt_init(void)
{
    ret_code_t err_code;

    // 1. 기본 WDT 설정 로드
    nrfx_wdt_config_t config = NRFX_WDT_DEAFULT_CONFIG;

    // 2. 타임아웃 시간 설정 (단위: ms) - 예: 5초(5000ms)
    // CPU Sleep 중에도 WDT가 카운트하도록 RUN_SLEEP 설정
    config.reload_value = 10000;
    config.behaviour    = NRF_WDT_BEHAVIOUR_RUN_SLEEP;

    // 3. WDT 드라이버 초기화
    err_code = nrfx_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);

    // 4. Feed 채널 할당 (최소 1개 이상의 채널 할당 필요)
    err_code = nrfx_wdt_channel_alloc(&m_wdt_channel_id);
    APP_ERROR_CHECK(err_code);

    // 5. WDT 동작 시작 (이후 설정 변경 불가)
    nrfx_wdt_enable();
    power_failure_warning_init();
    APP_ERROR_CHECK(app_timer_create(&m_wdg_timer,
                    APP_TIMER_MODE_REPEATED,
                    wdg_clear));
    app_timer_start(m_wdg_timer, APP_TIMER_TICKS(1000), NULL);
    NRF_LOG_INFO("WDT Initialized (Timeout: %d ms)", config.reload_value);
}

