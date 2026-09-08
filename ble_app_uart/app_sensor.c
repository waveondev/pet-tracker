#include "app_sensor.h"
#include "nordic_common.h"
#include "nrf.h"
#include <math.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_twi.h"

#include "app_gpio.h"
#include "nrf_gpio.h"

#include "lsm6dsv16b.h"

#include "stdio.h"


#include "app_adc.h"
#include "app_sensor_flash.h"
typedef struct 
{
  uint8_t regi;
  uint8_t data;
}Sensor_Setting_t;
#define GYRO 0
#if GYRO
#define DATA_SIZE 150
#else
#define DATA_SIZE 75
#endif






#if 1

static Sensor_Setting_t Sensor_value[] =
{
  {CTRL1,0x63},
  #if GYRO
  {CTRL2,0x53},
  #endif
  //{CTRL2,0x0},
  //{CTRL3,0x44},
  {CTRL9,0x20},
  {FIFO_CTRL1,DATA_SIZE},

 // {FIFO_CTRL2,0},

  #if GYRO
  {FIFO_CTRL3,0x33},
  #else
  {FIFO_CTRL3,0x03},
  #endif
 // {FIFO_CTRL3,0x03},
  {FIFO_CTRL4,0x01},
  {INT1_CTRL,0x08}
};



uint16_t fifo_get_size(void)
{
    uint8_t buffer[2];
    uint8_t status[2];
    lsm6dsv16b_fifo_status_t fifo_status = {0};
    buffer[0] = FIFO_STATUS1;
    //I2C_Transmitreceive(buffer, 1,status,2);
    //I2C_Receive(status,2);
    lsm6dsv16b_fifo_status_get(NULL,&fifo_status);

    uint16_t fifo_level = fifo_status.fifo_level;
    return fifo_level;
}
void Sensor_sleep(void)
{
    uint8_t buffer[2];

   for (int setup_size = 0;
     setup_size < sizeof(Sensor_value)/sizeof(Sensor_value[0]);
     setup_size++)
    {
        buffer[0] = Sensor_value[setup_size].regi;
        buffer[1] = 0;
        I2C_Transmit(buffer, 2);
    }

}
void Sensor_init(void)
{
    uint8_t buffer[2];

    uint8_t reg;
    uint8_t val;

    twi_init();

   Sensor_sleep();
 
   for (int setup_size = 0;
     setup_size < sizeof(Sensor_value)/sizeof(Sensor_value[0]);
     setup_size++)
    {
        buffer[0] = Sensor_value[setup_size].regi;
        buffer[1] = Sensor_value[setup_size].data;
        I2C_Transmit(buffer, 2);
    }
    
    (void)fifo_get_size();

    twi_deinit();
}
bool sensor_exfire = false;

void sensor_enable(void)
{
    sensor_exfire = true;
}

// ±2g 모드에서의 감도 (0.061 mg/LSB -> g 단위로 변환하기 위해 0.000061f 곱함)
#define LSM6DSV_XL_SENSITIVITY_2G   0.000061f
#define LSM6DSV_XL_SENSITIVITY_4G   0.000122f
#define LSM6DSV_XL_SENSITIVITY_8G   0.000244f
#define LSM6DSV_XL_SENSITIVITY_16G  0.000488f
float enmo_get(int16_t raw_x, int16_t raw_y, int16_t raw_z)
{

    float acc_x, acc_y, acc_z;
    float norm, enmo;


// Raw 데이터를 g 단위 실수(float)로 변환
    acc_x = (float)raw_x * LSM6DSV_XL_SENSITIVITY_2G;
    acc_y = (float)raw_y * LSM6DSV_XL_SENSITIVITY_2G;
    acc_z = (float)raw_z * LSM6DSV_XL_SENSITIVITY_2G;

// 1. Euclidean Norm 계산 (벡터 크기)
    norm = sqrtf((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

    // 2. Minus One (중력 가속도 1g 빼기)
    enmo = norm - 1.0f;

    // 3. 음수 값은 0으로 보정 (정지 상태 혹은 미세 움직임 노이즈 제거)
    if (enmo < 0.0f) {
        enmo = 0.0f;
    }

    return enmo;
}

#define LSM6DSV_GY_SENSITIVITY_2000DPS    0.070f

float gyro_intensity_get(int16_t raw_x, int16_t raw_y, int16_t raw_z)
{
    float gyro_x, gyro_y, gyro_z;
    float gvm;

    // 1. Raw 데이터를 dps(Degree per Second, 초당 회전 각도) 단위 실수로 변환
    // LSM6DSV16BX의 자이로 Full Scale 설정에 맞는 감도(Sensitivity)를 곱해줍니다.
    // 여기서는 예시로 2000dps 설정일 때의 감도를 곱한다고 가정합니다.
    gyro_x = (float)raw_x * LSM6DSV_GY_SENSITIVITY_2000DPS;
    gyro_y = (float)raw_y * LSM6DSV_GY_SENSITIVITY_2000DPS;
    gyro_z = (float)raw_z * LSM6DSV_GY_SENSITIVITY_2000DPS;

    // 2. 3축 벡터 합성 (회전의 세기 구하기)
    gvm = sqrtf((gyro_x * gyro_x) + (gyro_y * gyro_y) + (gyro_z * gyro_z));

    // 3. 센서가 가만히 있어도 발생하는 미세한 노이즈(바이어스) 제거 보정
    // 가만히 놔뒀을 때 dps가 약 1~2도 정도 흔들린다면 그 이하 값은 0으로 자릅니다.
    if (gvm < 2.0f) { 
        gvm = 0.0f;
    }

    return gvm; // 최종 회전 운동량 세기(dps 단위) 반환
}

uint32_t parse_fifo(uint8_t *buf)
{ 
    float total_enmo = 0.0f;
    float total_gyro = 0.0f;
    uint16_t acc_samples = 0;
    uint16_t gyro_samples = 0;
    for (int i = 0; i < DATA_SIZE; i++)
    {
        int idx = i * 7;

        uint8_t tag = (buf[idx] & 0xf8) >> 3;
        //NRF_LOG_INFO(" tag = %02x\n", tag);
        if(tag == LSM6DSV16B_XL_NC_TAG)
        {
          // 1. 가장 먼저 들어오는 바이트는 Z축 데이터입니다.
          int16_t az = (int16_t)((buf[idx + 2] << 8) | buf[idx + 1]);

          // 2. 중간에 들어오는 바이트는 Y축 데이터입니다.
          int16_t ay = (int16_t)((buf[idx + 4] << 8) | buf[idx + 3]);

          // 3. 마지막에 들어오는 바이트가 X축 데이터가 됩니다.
          int16_t ax = (int16_t)((buf[idx + 6] << 8) | buf[idx + 5]);

          total_enmo += enmo_get(ax, ay, az);
          acc_samples++;
         }
          else if (tag == 0x01) 
          { 
              // 자이로스코프 (X -> Y -> Z 순서)
              int16_t gx = (int16_t)((buf[idx + 2] << 8) | buf[idx + 1]);
              int16_t gy = (int16_t)((buf[idx + 4] << 8) | buf[idx + 3]);
              int16_t gz = (int16_t)((buf[idx + 6] << 8) | buf[idx + 5]);
        
              total_gyro += gyro_intensity_get(gx, gy, gz);
              gyro_samples++;
          }

    }
// 2. 각각의 평균값 계산
    float total_activity_score = 0.0f;
    float avg_enmo = (acc_samples > 0) ? (total_enmo / (float)acc_samples) : 0.0f;
    float avg_gvm = (gyro_samples > 0) ? (total_gyro / (float)gyro_samples) : 0.0f;

    if(acc_samples != 0)
    {
      total_activity_score += (total_enmo / (float)acc_samples);
    }
    if(gyro_samples != 0)
      total_activity_score += ((total_gyro / (float)gyro_samples) * 0.002f);


    return (uint32_t)(total_activity_score*1000);
}

#else

#endif



uint32_t BLE_Send_byte(uint8_t* data, uint16_t len);
uint32_t Sensor_Get_data(uint8_t* data, uint16_t len)
{
     uint8_t reg = 0x78;
      uint8_t tag = 0;
      uint8_t cnt = 0;
      // 1. register address write
      I2C_Transmitreceive(&reg, 1,data, len);
      //tag =  (data[0] & 0xf8) >> 3; 
           
      return parse_fifo(data);
}

void Sensor_Get_Id(void)
{
    uint8_t reg = 0x0F;
    uint8_t value = 0;
    I2C_Transmitreceive(&reg,1,&value,1);
    //nrfx_twi_tx(&m_twi, LSM6DSV_ADDR, &reg, 1, true);
    //nrfx_twi_rx(&m_twi, LSM6DSV_ADDR, &value, 1);

    NRF_LOG_INFO("WHO_AM_I = %x", value);
}
uint8_t fifo_buf[DATA_SIZE*7]; // DATA_SIZE × 7 bytes
void Sensor_update(void)
{
  uint32_t total_activity_score = 0;
  if(sensor_exfire || nrf_gpio_pin_read(LSM_INT_PIN))
  {

      twi_init();
      uint16_t fifo_size = fifo_get_size();
      //NRF_LOG_INFO("fifo_size = %d\n",  fifo_size);
      
      sensor_exfire = false;
      if(fifo_size >= DATA_SIZE)
      {

          total_activity_score = Sensor_Get_data(fifo_buf,sizeof(fifo_buf));
          Sensor_flash_set(total_activity_score);
      
          NRF_LOG_INFO(" value = %d\n", total_activity_score);
      }
     // Sensor_init();
      twi_deinit();
      saadc_timer_handler(NULL);
  }
}


void lsb6_ble_data(Motion_Packet_t* rx_packet)
{
    // 1. 포인터 검사 및 데이터 길이 유효성 예외 처리
    if (rx_packet == NULL) return;

    uint8_t len = rx_packet->lsm6_data_req_res.data_len;
    uint8_t start_addr = rx_packet->lsm6_data_req_res.start_address;

    // 최대 16바이트 제한 초과 시 실행 안 함 (메모리 오염 방지)
    if (len == 0 || len > 16) return;

    // -----------------------------------------------------------------------
    // [WRITE] I2C 레지스터 쓰기 (Read/Write == 1)
    // -----------------------------------------------------------------------
    if (rx_packet->lsm6_data_req_res.read_write == 1)
    {
        // [시작주소 1바이트] + [데이터 최대 16바이트] = 17바이트 버퍼
        uint8_t tx_buf[17]; 

        tx_buf[0] = start_addr;
        memcpy(&tx_buf[1], rx_packet->lsm6_data_req_res.data, len);

        // 단 한 번의 I2C 트랜잭션으로 (len + 1) 바이트 연속 전송
        I2C_Transmit(tx_buf, len + 1);
    }
    // -----------------------------------------------------------------------
    // [READ] I2C 레지스터 읽기 (Read/Write == 0)
    // -----------------------------------------------------------------------
    else if (rx_packet->lsm6_data_req_res.read_write == 0)
    {
        // 시작 주소(1바이트)를 전송하고, len 바이트만큼 한 번에 연속 수신
        I2C_Transmitreceive(&start_addr, 1, rx_packet->lsm6_data_req_res.data, len);
    }
    else
    {
        return;
    }

    BLE_Send_byte((uint8_t*)rx_packet,sizeof(Motion_Packet_t));

}