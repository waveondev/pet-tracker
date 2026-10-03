#include "app_sensor_flash.h"
#include "nordic_common.h"
#include "nrf.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "fds.h"
#include "nrf.h"
#include "app_flash.h"


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




retained_data_t* Sensor_Get_Seq(void)
{
  return &Retained_data;
}


/**
 * @brief 현재 주소를 8바이트 증가시키고, 필요시 날짜 점프 및 30일 롤백을 수행하여 반환
 */
uint32_t get_next_flash_addr(uint32_t addr, uint8_t len)
{
    uint32_t next = addr + len; // 8바이트 전진
    
    // 1. 하루치 마감(1440개 * 8바이트 = 11520바이트) 구역에 도달했는지 확인
    uint32_t offset_in_day = (next - FLASH_START_ADDR) % MOTION_DAY_SIZE;
    if (offset_in_day >= (LOGS_PER_DAY * len))
    {
        // 16KB(0x4000) 경계의 다음 날 시작 주소로 점프
        uint32_t day_idx = (next - FLASH_START_ADDR) / MOTION_DAY_SIZE;
        next = FLASH_START_ADDR + ((day_idx + 1) * MOTION_DAY_SIZE);
    }
    
    // 2. 30일 마지노선(0xB8000) 도달 시 첫 주소로 랩어라운드
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
    static uint8_t index = 0;
    

    my_data[index] = data;

    index++;

    tracker_setting_t* setting = Tracker_Get_Setting();
    if(index > 3)
    {
        index = 0;
        // 1. 머리(current)가 다음에 이동할 예상 주소를 똑똑한 헬퍼 함수로 계산! (날짜 점프 포함)
        uint32_t next_addr = get_next_flash_addr(m_current_addr, 4);
        // 머리가 꼬리를 덮으려 하면 가장 오래된 하루(0x3000)를 삭제
        if ((next_addr == m_last_send_addr) &&
            (m_current_addr != m_last_send_addr))
        {
            uint32_t tail_day_idx = (m_last_send_addr - FLASH_START_ADDR) / MOTION_DAY_SIZE;
            m_last_send_addr = FLASH_START_ADDR + ((tail_day_idx + 1) * MOTION_DAY_SIZE);
            
            if (m_last_send_addr >= MOTION_END_ADDR)
            {
                m_last_send_addr = FLASH_START_ADDR;
            }
            
            NRF_LOG_WARNING("Ring buffer full! Dropped 1 whole day. New last_send_addr: 0x%08x", m_last_send_addr);
        }

        NRF_LOG_INFO("flash = %p my_data = %d %d %d %d  \n",m_current_addr, my_data[0],my_data[1],my_data[2],my_data[3]);
        // 3. 플래시 쓰기
        ret_code_t rc = flash_write(m_current_addr, (uint8_t*)my_data, sizeof(my_data));
       
        if (rc == NRF_SUCCESS)
        {
            // ★ [정답] 똑똑하게 계산해 둔 next_addr를 그대로 현재 주소에 덮어씌움!
            m_current_addr = next_addr; 
        }
        
        // 데이터 카운트만 올려주고 끝! (아래쪽에 있던 중복 점프 로직은 싹 삭제)

    }
}


static uint16_t max_idx = 0;
uint8_t sensor_get_data(Motion_Packet_t* data)
{
    uint8_t pack_idx = 0;
    
    uint8_t* motion_ptr = data->motion_data.MLC_data;
    uint8_t my_data;

// 이미 목표 주소에 도달했다면 바로 종료
    if (m_current_send_addr == m_target_send_addr) {
        return 0;
    }
    while ((pack_idx < 18) && (m_current_send_addr != m_target_send_addr))
    {
        flash_read(m_current_send_addr, (uint8_t*)&my_data, sizeof(uint8_t));

        motion_ptr[pack_idx] = my_data;

        pack_idx++;  
        max_idx++;


        uint32_t next_addr = get_next_flash_addr(m_current_send_addr, 1);
        
        // [안전장치] 다음 주소가 목표 주소를 넘어서거나 오버플로우 발생 시 중단
        if (next_addr == m_current_send_addr) { 
            NRF_LOG_ERROR("Flash Address stuck!\r\n");
            m_current_send_addr = m_target_send_addr; // 강제 종료 조건 달성
            break;
        }

        m_current_send_addr = next_addr;
        if(max_idx >= 100)
          return 0;
    }

    if ((pack_idx > 0) && (m_current_send_addr != m_target_send_addr)) 
    {
        return 1; // 더 보낼 데이터가 남아있음 (타이머 계속 진행)
    } 
    else if (pack_idx > 0)
    {
        // 이번 패킷이 마지막 데이터인 경우!
        // (데이터 전송은 진행하되, 다음 타이머는 켜지 않도록 0 반환)
        return 0; 
    }
    else 
    {
        return 0; // 보낼 데이터가 전혀 없음
    }
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

