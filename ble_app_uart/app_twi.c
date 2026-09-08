#include "app_twi.h"
#include "nrfx_twi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_sensor.h"
#include "app_gpio.h"
#define TWI_INSTANCE_ID 0
#define LSM6DSV_ADDR 0x6A
static const nrfx_twi_t m_twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);
#include "nrfx_gpiote.h"

void I2C_Transmit(uint8_t* data, uint16_t len)
{
    nrfx_twi_tx(&m_twi, LSM6DSV_ADDR, data, len, false);
}
void I2C_Transmitreceive(uint8_t* data,uint16_t len,uint8_t* rxdata, uint16_t rx_len)
{
    nrfx_twi_tx(&m_twi, LSM6DSV_ADDR, data, len,true);
    if(rx_len != 0 && rxdata != NULL)
      nrfx_twi_rx(&m_twi, LSM6DSV_ADDR, rxdata, rx_len);
}
void I2C_Receive(uint8_t* data,uint16_t len)
{
    nrfx_twi_rx(&m_twi, LSM6DSV_ADDR, data, len);
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
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .hold_bus_uninit     = true // uninit 시 드라이버가 알아서 핀을 안 건드리게 설정
    };

    // ② TWI 드라이버 초기화 (미리 켜둔 풀업 상태를 드라이버가 안전하게 인계받음)
    ret_code_t err = nrfx_twi_init(&m_twi, &config, NULL, NULL);
    APP_ERROR_CHECK(err);

    // ③ TWI 하드웨어 활성화 (이제 오픈 드레인 통신 라인으로 완벽 가동)
    nrfx_twi_enable(&m_twi);

    // ❌ (기존 버그점) 여기에 있던 nrf_gpio_cfg_input 중복 덮어쓰기는 완전 삭제했습니다!

    // ④ 통신 준비 완료 후 장치 ID 체크
    //Sensor_Get_Id();
}

