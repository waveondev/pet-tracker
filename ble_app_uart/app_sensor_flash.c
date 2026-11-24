#include "app_sensor_flash.h"
#include "nordic_common.h"
#include "nrf.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "fds.h"
#include "nrf.h"
#include "app_flash.h"

#include "app_ble_tx.h"

#define MOTION_DAY_SIZE      0x9000   // 
#define TOTAL_DAYS           6       // 30일 순환
#define MOTION_END_ADDR      (FLASH_START_ADDR + (MOTION_DAY_SIZE * TOTAL_DAYS)) //

#define LOGS_PER_DAY        34560
#define DATA_SIZE_PER_DAY     (LOGS_PER_DAY * sizeof(uint8_t))

static retained_data_t Retained_data;
// 현재 플래시 주소 위치 및 전역 데이터 카운터
static uint32_t m_current_addr = FLASH_START_ADDR;
static uint32_t m_last_send_addr = FLASH_START_ADDR;
static uint32_t m_target_send_addr = FLASH_START_ADDR;
static uint32_t m_current_send_addr = FLASH_START_ADDR;
static uint8_t m_save_index = 0;



uint8_t get_save_index(void)
{
  return m_save_index;
}
/**
 * @brief 현재 주소를 8바이트 증가시키고, 필요시 날짜 점프 및 30일 롤백을 수행하여 반환
 */
uint32_t get_next_flash_addr(uint32_t addr, uint8_t len)
{
    uint32_t next = addr + len;

    if (next >= MOTION_END_ADDR)
    {
        next = FLASH_START_ADDR;
    }

    return next;
}

/**
 * @brief 현재 전송 주소부터 목표 주소 사이에 존재하는 실제 데이터(8바이트 노드)의 총 개수를 계산하는 함수
 * @return uint32_t 실제 저장된 데이터의 개수 (짜투리 패딩 공간은 제외함)
 */
uint32_t calculate_total_send_count(void)
{
    uint32_t total_bytes = 0;
    uint32_t padding_bytes = 0;
    uint32_t start_addr = m_current_send_addr;
    uint32_t end_addr = m_target_send_addr;

    // 1. 주소 역전 여부에 따른 순수 바이트 크기 계산
    if (start_addr <= end_addr)
    {
        // 정방향인 경우 두 주소의 차이
        total_bytes = end_addr - start_addr;
        

        uint32_t start_day = (start_addr - FLASH_START_ADDR) / MOTION_DAY_SIZE;
        uint32_t end_day = (end_addr - FLASH_START_ADDR) / MOTION_DAY_SIZE;
        
        // 하루당 768 바이트
        padding_bytes = (end_day - start_day) * (MOTION_DAY_SIZE - (LOGS_PER_DAY * sizeof(uint8_t)));
    }
    else
    {
        // 역전된 경우: (시작점 ~ 마지노선) + (0x40000 ~ 목표점)
        total_bytes = (MOTION_END_ADDR - start_addr) + (end_addr - FLASH_START_ADDR);
        
        // 각각의 영역에서 패딩 계산
        uint32_t start_day = (start_addr - FLASH_START_ADDR) / MOTION_DAY_SIZE;
        uint32_t max_day = (MOTION_END_ADDR - FLASH_START_ADDR) / MOTION_DAY_SIZE;
        uint32_t end_day = (end_addr - FLASH_START_ADDR) / MOTION_DAY_SIZE;
        
        uint32_t first_part_days = max_day - start_day;
        uint32_t second_part_days = end_day; // 0x40000(0일차)부터 시작하므로 end_day 값이 곧 지난 날짜 수
        
        uint32_t total_padding_days = first_part_days + second_part_days;
        padding_bytes = total_padding_days * (MOTION_DAY_SIZE - (LOGS_PER_DAY * sizeof(uint8_t)));
    }

    // 2. 전체 바이트에서 짜투리 패딩을 빼면 순수 데이터 바이트만 남음
    if (total_bytes < padding_bytes) return 0; // 예외 방어
    uint32_t pure_data_bytes = total_bytes - padding_bytes;

    // 3. 8바이트로 나누면 정확한 데이터 개수(Count)가 나옵니다.
    return (pure_data_bytes / sizeof(uint8_t));
}

void Sensor_flash_set(uint8_t data)
{
    static uint8_t my_data[4];
  
    my_data[m_save_index++] = data;

    if (m_save_index < 4)
    {
        return;
    }

    m_save_index = 0;

    uint32_t next_addr = get_next_flash_addr(m_current_addr, 4);

    // 다음 기록 위치가 읽기 시작 위치를 따라잡으면
    // last 포인터를 4KB 전진
    if (next_addr == m_last_send_addr)
    {
        uint32_t new_last = m_last_send_addr + FLASH_PAGE_SIZE;

        if (new_last >= MOTION_END_ADDR)
        {
            new_last = FLASH_START_ADDR;
        }

        m_last_send_addr = new_last;

        NRF_LOG_WARNING(
            "Ring buffer full! new last = 0x%08x",
            m_last_send_addr
        );
    }

    ret_code_t rc = flash_write(
        m_current_addr,
        my_data,
        sizeof(my_data)
    );

    if (rc == NRF_SUCCESS)
    {
        m_current_addr = next_addr;
    }
}


static uint16_t max_idx = 0;

uint8_t sensor_get_data(Motion_Packet_t* data)
{
    uint8_t pack_idx = 0;
    uint8_t* motion_ptr = data->motion_data.MLC_data;
    uint8_t my_data;

    // 이미 목표 주소에 도달했거나 최대 데이터 수에 도달한 경우
    if ((m_current_send_addr == m_target_send_addr) ||
        (max_idx >= MAX_TRANS_DATA))
    {
        return 0;
    }

    while ((pack_idx < (MAX_TRANS_DATA/30)) &&
           (m_current_send_addr != m_target_send_addr) &&
           (max_idx < MAX_TRANS_DATA))
    {
        flash_read(m_current_send_addr, &my_data, sizeof(my_data));

        motion_ptr[pack_idx] = my_data;

        pack_idx++;
        max_idx++;

        uint32_t next_addr = get_next_flash_addr(m_current_send_addr, 1);

        // 주소 이동 실패
        if (next_addr == m_current_send_addr)
        {
            NRF_LOG_ERROR("Flash Address stuck!\r\n");
            m_current_send_addr = m_target_send_addr;
            break;
        }

        m_current_send_addr = next_addr;
    }

    // 디버깅 로그
    NRF_LOG_INFO("pack_idx=%u, max_idx=%u", pack_idx, max_idx);
    NRF_LOG_INFO("current=0x%08X, target=0x%08X",
                 m_current_send_addr, m_target_send_addr);

    // 더 읽을 데이터가 남아 있을 때만 1 반환
    if ((m_current_send_addr != m_target_send_addr) &&
        (max_idx < MAX_TRANS_DATA))
    {
      NRF_LOG_INFO("current=0x%08X, target=0x%08X",
               m_current_send_addr, m_target_send_addr);
        return 1;
    }

    return 0;
}

static uint32_t normalize_day_addr(uint32_t addr)
{
    uint32_t day =
        (addr - FLASH_START_ADDR) / MOTION_DAY_SIZE;

    return FLASH_START_ADDR +
           day * MOTION_DAY_SIZE;
}

void sensing_ack_input(void)
{
    m_last_send_addr = m_current_send_addr;
       // normalize_day_addr(m_current_send_addr);
}

void sensing_send_init(void)
{
    max_idx = 0;
    m_target_send_addr = m_current_addr;
    m_current_send_addr = m_last_send_addr;
    NRF_LOG_INFO("m_target_send_addr = %08x m_current_addr = %08x\n", m_target_send_addr,m_current_addr);
    NRF_LOG_INFO("m_current_send_addr = %08x m_last_send_addr = %08x\n",m_current_send_addr ,m_last_send_addr);
}

