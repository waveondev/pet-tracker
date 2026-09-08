#include "app_rtc.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "app_sensor_flash.h"
static uint32_t m_base_epoch = 0;       // 기준이 되는 Unix Timestamp (초)
static uint32_t m_base_rtc_ticks = 0;   // 기준 시점의 app_timer RTC Ticks
static bool     m_is_synchronized = false;

void rtc_sync_init(void)
{
    m_base_epoch = 0;
    m_base_rtc_ticks = app_timer_cnt_get();
    m_is_synchronized = false;
}

void rtc_sync_set_time(Motion_Packet_t* Motion_Packet)
{
    uint32_t received_epoch = Motion_Packet->time_res.epoch_sec;
    // 현재 타이머 틱값과 맞물려 기준 시각을 업데이트
    m_base_rtc_ticks = app_timer_cnt_get();
    m_base_epoch = received_epoch;
    m_is_synchronized = true;

    struct tm t_info;
    rtc_sync_get_time_tm(&t_info);
    NRF_LOG_INFO("Current Time: %04d-%02d-%02d %02d:%02d:%02d\r\n",
                 t_info.tm_year + 1900, t_info.tm_mon + 1, t_info.tm_mday,
                 t_info.tm_hour, t_info.tm_min, t_info.tm_sec);


}

uint32_t rtc_sync_get_time(void)
{
    if (!m_is_synchronized) {
        return 0; // 동기화되지 않은 경우
    }

    uint32_t current_ticks = app_timer_cnt_get();
    
    // 24비트 RTC 카운터의 오버플로우(Roll-over) 처리 계산
    uint32_t elapsed_ticks = app_timer_cnt_diff_compute(current_ticks, m_base_rtc_ticks);

    // Ticks를 초(Second) 단위로 변환
    // APP_TIMER_CONFIG_FREQ = 32768Hz / (PRESCALER + 1)
    uint32_t elapsed_sec = elapsed_ticks / APP_TIMER_CLOCK_FREQ;

    return m_base_epoch + elapsed_sec;
}

void rtc_sync_get_time_tm(struct tm * time_info)
{
    time_t current_epoch = (time_t)rtc_sync_get_time();
    if (time_info != NULL) {
        // Epoch time을 tm 구조체로 변환 (UTC 기준)
        struct tm * t = gmtime(&current_epoch);
        if (t != NULL) {
            *time_info = *t;
        }
    }
}