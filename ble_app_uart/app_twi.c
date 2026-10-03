#include "app_twi.h"
#include "nrfx_twi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_sensor.h"
#include "app_gpio.h"
#include "nrf_pwr_mgmt.h"
#include "app_qc.h"
#define TWI_INSTANCE_ID 0
#define LSM6DSV_ADDR 0x6A
static const nrfx_twi_t m_twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);
#include "nrfx_gpiote.h"
// TWI 상태를 추적하기 위한 플래그 (선택 사항)
static volatile bool m_xfer_done = false;
uint32_t I2C_Transmit(uint8_t* data, uint16_t len)
{
    m_xfer_done = false;
    ret_code_t err = nrfx_twi_tx(&m_twi, LSM6DSV_ADDR, data, len, false);
    if (err != NRFX_SUCCESS)
    {
        return err;
    }

    // 전력 관리 슬립 대신 단순 대기 (타임아웃 안전장치 포함 권장)
    uint32_t timeout = 10000; // 적절한 타임아웃 틱 또는 카운터
    while (!m_xfer_done && timeout--)
    {
        // 빈 루프 또는 __NOP()로 대기 (인터럽트에 의해 m_xfer_done이 true가 됨)
    }

    if (!m_xfer_done) {
        return NRF_ERROR_TIMEOUT; // 타임아웃 에러 처리
    }

    return err;
}
uint32_t I2C_Transmitreceive(uint8_t* data,uint16_t len,uint8_t* rxdata, uint16_t rx_len)
{
    m_xfer_done = false;
    ret_code_t err = 0;
    // 1. 레지스터 주소 전송 (no_stop = true 로 Repeated Start 생성)
    err = nrfx_twi_tx(&m_twi, LSM6DSV_ADDR, data, 1, true);
    if (err != NRFX_SUCCESS) 
      return err;

    // 2. TX 완료될 때까지 비동기 대기 (MCU는 다른 작업을 하거나 쉼)
    while (!m_xfer_done)
    {
        nrf_pwr_mgmt_run(); // Low Power Wait
    }

    m_xfer_done = false;

    // 3. 데이터 수신 시작
    err = nrfx_twi_rx(&m_twi, LSM6DSV_ADDR, rxdata, rx_len); 
    if (err != NRFX_SUCCESS)
      return err;

    // 4. RX 완료 대기
    while (!m_xfer_done)
    {
        nrf_pwr_mgmt_run(); // Low Power Wait
    }

    return err;
}


// ① TWI 비동기 콜백 이벤트 핸들러
static void twi_handler(nrfx_twi_evt_t const * p_event, void * p_context)
{
    static uint32_t twi_err = 0;
    switch (p_event->type)
    {
        case NRFX_TWI_EVT_DONE:
            // TX 또는 RX 통신 성공 완료
            m_xfer_done = true;
            //NRF_LOG_INFO("TWI Transfer Done.");
            break;

        case NRFX_TWI_EVT_ADDRESS_NACK:
            // 상대 칩이 주소 응답(ACK)을 하지 않음 (슬레이브 미연결/주소 오류)
            m_xfer_done = true;
            if(twi_err == 0)
            {
                twi_err = 1;
                error_set_flag(MOTION_ERROR);
            }
            break;

        case NRFX_TWI_EVT_DATA_NACK:
            // 데이터 전송 중 NACK 발생
            m_xfer_done = true;
            break;

        default:
            m_xfer_done = true;
            break;
    }
}

// ---------------------------------------------------------------------------
// 🔒 1. 완벽한 자물쇠 해제 (초저전력 드라이버 종료)
// ---------------------------------------------------------------------------
void twi_deinit(void)
{
    // ① TWI 하드웨어 비활성화
    nrfx_twi_disable(&m_twi); 

    // ② [필수 추가] TWI 드라이버 리소스 및 인터럽트 완벽 해제 (중복 init 에러 방지)
    nrfx_twi_uninit(&m_twi); 
    NRF_TWI0->ENABLE = 0;
    *(volatile uint32_t *)((uint32_t)NRF_TWI0 + 0xFFC) = 0;
    *(volatile uint32_t *)((uint32_t)NRF_TWI0 + 0xFFC); // Dummy read
    // ③ [풀업 누설 차단] 핀을 일반 고저항 입력(Disconnected) 상태로 완벽히 격리
   // nrf_gpio_cfg_default(LSM_SDA); 
    
   // nrf_gpio_cfg_default(LSM_SCL);

}

// ---------------------------------------------------------------------------
// 🔓 2. 안전한 드라이버 초기화 (순서 교정 완료)
// ---------------------------------------------------------------------------
void twi_init(void)
{
    // ① [교정] TWI 모듈이 핀을 잡기 전에 '내부 풀업 저항' 설정을 미리 켜둡니다.
    nrf_gpio_cfg_input(LSM_SDA, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(LSM_SCL, NRF_GPIO_PIN_PULLUP);
    NRF_TWI0->ENABLE = 1;
    *(volatile uint32_t *)((uint32_t)NRF_TWI0 + 0xFFC) = 1;
        *(volatile uint32_t *)((uint32_t)NRF_TWI0 + 0xFFC); // Dummy read
    const nrfx_twi_config_t config = {
        .scl                = LSM_SCL,
        .sda                = LSM_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW,
        .hold_bus_uninit     = true // uninit 시 드라이버가 알아서 핀을 안 건드리게 설정
    };

    // ② TWI 드라이버 초기화 (미리 켜둔 풀업 상태를 드라이버가 안전하게 인계받음)
    ret_code_t err = nrfx_twi_init(&m_twi, &config, twi_handler, NULL);
    APP_ERROR_CHECK(err);
    // ③ TWI 하드웨어 활성화 (이제 오픈 드레인 통신 라인으로 완벽 가동)
    nrfx_twi_enable(&m_twi);

    // ❌ (기존 버그점) 여기에 있던 nrf_gpio_cfg_input 중복 덮어쓰기는 완전 삭제했습니다!

    // ④ 통신 준비 완료 후 장치 ID 체크

}

