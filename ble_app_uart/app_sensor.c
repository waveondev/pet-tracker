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
#include <stdlib.h>

#include "app_adc.h"
#include "app_sensor_flash.h"
uint16_t fifo_get_size(void);
uint32_t BLE_Send_byte(uint8_t* data, uint16_t len);
uint32_t Sensor_Get_data(uint8_t* data, uint16_t len);
void sensor_flag_input(void);
static Motion_Packet_t next_data;
typedef struct 
{
  uint8_t regi;
  uint8_t data;
}Sensor_Setting_t;

typedef struct 
{
  int16_t ax; 
  int16_t ay;
  int16_t az;
  int16_t gx; 
  int16_t gy;
  int16_t gz;
  uint8_t mlc[4];
}Sensor_RawData_t;


Sensor_RawData_t Sensor_RawData;

#define DATA_SIZE 75



volatile bool sensor_exfire = false;


volatile bool debugmode_flag = false;


// ±2g 모드에서의 감도 (0.061 mg/LSB -> g 단위로 변환하기 위해 0.000061f 곱함)
#define LSM6DSV_XL_SENSITIVITY_2G   0.000061f
#define LSM6DSV_XL_SENSITIVITY_4G   0.000122f
#define LSM6DSV_XL_SENSITIVITY_8G   0.000244f
#define LSM6DSV_XL_SENSITIVITY_16G  0.000488f

#define LSM6DSV_GY_SENSITIVITY_125DPS    0.004375f
#define LSM6DSV_GY_SENSITIVITY_250DPS    0.00875f
#define LSM6DSV_GY_SENSITIVITY_500DPS    0.0175f
#define LSM6DSV_GY_SENSITIVITY_1000DPS    0.035f
#define LSM6DSV_GY_SENSITIVITY_2000DPS    0.070f
#define LSM6DSV_GY_SENSITIVITY_4000DPS    0.140f

static float XL_SENS = LSM6DSV_XL_SENSITIVITY_2G;
static float GY_SENS = LSM6DSV_GY_SENSITIVITY_2000DPS;
static uint32_t data_size = DATA_SIZE;

#if  1// NORMAL

static Sensor_Setting_t Sensor_value[] =
{
  {CTRL1,0x63},
  #if GYRO
  {CTRL2,0x53},
  #endif

  {CTRL9,0x20},
  {FIFO_CTRL1,0},

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



static Sensor_Setting_t debug_Sensor_value[] =
{
    {0x10, 0x00},
    {0x11, 0x00},
    {0x01, 0x80},
    {0x04, 0x00},
    {0x05, 0x00},
    {0x17, 0x40},
    {0x02, 0x11},
    {0x08, 0xEA},
    {0x09, 0xF6},
    {0x09, 0x03},
    {0x09, 0x14},
    {0x09, 0x04},
    {0x09, 0x01},
    {0x09, 0x00},
    {0x09, 0x14},
    {0x09, 0x01},
    {0x09, 0x96},
    {0x02, 0x11},
    {0x08, 0xFA},
    {0x09, 0x5C},
    {0x09, 0x03},
    {0x09, 0x2E},
    {0x09, 0x04},
    {0x09, 0x3A},
    {0x09, 0x04},
    {0x02, 0x31},
    {0x08, 0x5C},
    {0x09, 0x0D},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x3C},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x69},
    {0x09, 0xBA},
    {0x09, 0xC8},
    {0x09, 0x39},
    {0x09, 0x0D},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x3C},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x4B},
    {0x09, 0xBE},
    {0x09, 0xC8},
    {0x09, 0x39},
    {0x09, 0x0D},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x3C},
    {0x09, 0x00},
    {0x09, 0xBC},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0xD5},
    {0x09, 0xBD},
    {0x09, 0xC8},
    {0x09, 0x39},
    {0x09, 0x0D},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x3C},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x09},
    {0x09, 0xBF},
    {0x09, 0x38},
    {0x09, 0x3B},
    {0x09, 0x1D},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x3C},
    {0x09, 0x00},
    {0x09, 0xBC},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x2A},
    {0x09, 0xBB},
    {0x09, 0x38},
    {0x09, 0x3B},
    {0x09, 0x3F},
    {0x09, 0x00},
    {0x09, 0x01},
    {0x09, 0x24},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x01},
    {0x09, 0x2C},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x04},
    {0x09, 0x30},
    {0x09, 0x3F},
    {0x09, 0x31},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x04},
    {0x09, 0x34},
    {0x09, 0x0E},
    {0x09, 0x32},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x17},
    {0x09, 0x30},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0xC2},
    {0x09, 0x1A},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0xF9},
    {0x09, 0xBB},
    {0x09, 0x17},
    {0x09, 0x38},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0xC2},
    {0x09, 0x22},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0xE5},
    {0x09, 0xBB},
    {0x09, 0x18},
    {0x09, 0x3C},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x8E},
    {0x09, 0x12},
    {0x09, 0x00},
    {0x09, 0x3C},
    {0x09, 0x1F},
    {0x09, 0x00},
    {0x02, 0x41},
    {0x08, 0x2E},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x09, 0x00},
    {0x01, 0x00},
    {0x01, 0x80},
    {0x17, 0x40},
    {0x02, 0x41},
    {0x08, 0x3A},
    {0x09, 0x20},
    {0x09, 0x4D},
    {0x09, 0x0D},
    {0x09, 0x83},
    {0x09, 0xB1},
    {0x09, 0x1F},
    {0x09, 0x20},
    {0x09, 0xC1},
    {0x09, 0x90},
    {0x09, 0x33},
    {0x09, 0x05},
    {0x09, 0x80},
    {0x09, 0x40},
    {0x09, 0x48},
    {0x09, 0x40},
    {0x09, 0xC2},
    {0x09, 0x13},
    {0x09, 0x4D},
    {0x09, 0x03},
    {0x09, 0xE5},
    {0x09, 0x36},
    {0x09, 0x38},
    {0x09, 0x07},
    {0x09, 0x85},
    {0x09, 0x40},
    {0x09, 0x48},
    {0x09, 0x30},
    {0x09, 0xE2},
    {0x09, 0x36},
    {0x09, 0x47},
    {0x09, 0x09},
    {0x09, 0x89},
    {0x09, 0x47},
    {0x09, 0xC6},
    {0x09, 0x03},
    {0x09, 0xEB},
    {0x09, 0x1C},
    {0x09, 0xC6},
    {0x09, 0x0C},
    {0x09, 0x8B},
    {0x09, 0xC7},
    {0x09, 0x3C},
    {0x09, 0x20},
    {0x09, 0xA1},
    {0x09, 0x93},
    {0x09, 0x3C},
    {0x09, 0x30},
    {0x09, 0xE0},
    {0x09, 0x20},
    {0x09, 0x4C},
    {0x09, 0x13},
    {0x09, 0xE3},
    {0x09, 0x4B},
    {0x09, 0x4A},
    {0x09, 0x20},
    {0x09, 0xA9},
    {0x09, 0x77},
    {0x09, 0x38},
    {0x09, 0xF3},
    {0x09, 0xC5},
    {0x09, 0x27},
    {0x09, 0x31},
    {0x09, 0x00},
    {0x09, 0xC0},
    {0x09, 0xA0},
    {0x09, 0x4C},
    {0x09, 0x10},
    {0x09, 0xA2},
    {0x09, 0x84},
    {0x09, 0x40},
    {0x09, 0x21},
    {0x09, 0xC0},
    {0x09, 0x86},
    {0x09, 0x41},
    {0x09, 0x13},
    {0x09, 0xE5},
    {0x01, 0x80},
    {0x17, 0x00},
    {0x04, 0x00},
    {0x05, 0x10},
    {0x02, 0x01},
    {0x01, 0x00},
    {0x5E, 0x02},
    {0x01, 0x80},
    {0x0D, 0x01},
    {0x60, 0x25},
    {0x45, 0x02},
    {0x01, 0x00},
    {0x15, 0x04},
    {0x17, 0x02},
    {0x10, 0x65},
    {0x11, 0x55},
    {CTRL9,0x20},
    {FIFO_CTRL1,0},

    {FIFO_CTRL3,0x55},
    {FIFO_CTRL4,0x01},
    {INT1_CTRL,0x08}
};


#endif

// FIFO 래치 해제 및 초기화
void lsm6dsv_fifo_flush(uint8_t* fifo_buf, uint32_t data_len)
{
    lsm6dsv16b_fifo_status_t fifo_status = {0};

     uint8_t reg = 0x78;
     uint32_t ret = 0;
    while(1)
    {
      ret = lsm6dsv16b_fifo_status_get(NULL, &fifo_status);
      if(fifo_status.fifo_level == 0 || ret != NRF_SUCCESS)
          break;
      I2C_Transmitreceive(&reg, 1,fifo_buf, data_len);
    }

}
void Sensor_update(void)
{
  uint32_t total_activity_score = 0;
  uint8_t buffer[2];

  if(sensor_exfire|| nrf_gpio_pin_read(LSM_INT_PIN))
  {

    uint32_t data_len = (data_size *7);
    uint8_t *fifo_buf = calloc(1,data_len);
    if(fifo_buf == NULL)
    {
      NRF_LOG_INFO("fifo_buf full");
      return;
    }
     
    sensor_exfire = false;
    twi_init();

    lsm6dsv16b_fifo_status_t fifo_status = {0};
    buffer[0] = FIFO_STATUS1;

    lsm6dsv16b_fifo_status_get(NULL,&fifo_status);
    #if 0
    NRF_LOG_INFO(
        "FIFO: level=%d TH=%d FULL=%d OVR=%d BDR=%d",
        fifo_status.fifo_level,
        fifo_status.fifo_th,
        fifo_status.fifo_full,
        fifo_status.fifo_ovr,
        fifo_status.fifo_bdr
    );
    #endif

    if(fifo_status.fifo_th)
    {
        total_activity_score = Sensor_Get_data(fifo_buf,data_len);
        if(debugmode_flag == false)
          Sensor_flash_set(total_activity_score);
    }
    else
    {
      // Embedded Function register access
      uint8_t tx[2] = {0x01, 0x80};
      I2C_Transmit(tx, 2);

      // MLC1~4
      uint8_t reg = 0x70;

      I2C_Transmitreceive(&reg, 1, &Sensor_RawData.mlc[0], 4);

        NRF_LOG_INFO("MLC1=%02X MLC2=%02X MLC3=%02X MLC4=%02X",\
                     Sensor_RawData.mlc[0], Sensor_RawData.mlc[1], Sensor_RawData.mlc[2], Sensor_RawData.mlc[3]);
      tx[1] = 0x00;
      I2C_Transmit(tx, 2);


    }

    saadc_timer_handler(NULL);
    if(debugmode_flag)
    {
      memset(&next_data,0,sizeof(next_data));
      next_data.event_code = LSM6_DATA_RESPONSE;
      next_data.lsm6_data_req_res.read_write = 3;
      next_data.lsm6_data_req_res.start_address = 0;
      next_data.lsm6_data_req_res.data_len = sizeof(Sensor_RawData_t);
      memcpy(next_data.lsm6_data_req_res.data,&Sensor_RawData,sizeof(Sensor_RawData_t));
      BLE_Send_byte((uint8_t*)&next_data,sizeof(Motion_Packet_t));
    }

    lsm6dsv_fifo_flush(fifo_buf,data_len);
    free(fifo_buf);

    twi_deinit();
  }
  sensor_flag_input();
}

uint16_t fifo_get_size(void)
{
    uint8_t buffer[2];
    uint8_t status[2];
    lsm6dsv16b_fifo_status_t fifo_status = {0};
    buffer[0] = FIFO_STATUS1;
    //I2C_Transmitreceive(buffer, 1,status,2);
    //I2C_Receive(status,2);
    lsm6dsv16b_fifo_status_get(NULL,&fifo_status);
    NRF_LOG_INFO(
        "FIFO: level=%d TH=%d FULL=%d OVR=%d BDR=%d",
        fifo_status.fifo_level,
        fifo_status.fifo_th,
        fifo_status.fifo_full,
        fifo_status.fifo_ovr,
        fifo_status.fifo_bdr
    );
    uint16_t fifo_level = fifo_status.fifo_level;
    return fifo_level;
}
void Sensor_sleep(void)
{
    uint8_t buffer[2];

   for (int setup_size = 0;
     setup_size < sizeof(debug_Sensor_value)/sizeof(debug_Sensor_value[0]);
     setup_size++)
    {
        buffer[0] = debug_Sensor_value[setup_size].regi;
        buffer[1] = 0;
        I2C_Transmit(buffer, 2);
    }

}

static void sensor_normal_init(void)
{
    uint8_t buffer[2];
      NRF_LOG_INFO("Sensor_value = %d", sizeof(Sensor_value));
   for (int setup_size = 0;
     setup_size < sizeof(Sensor_value)/sizeof(Sensor_value[0]);
     setup_size++)
    {
        buffer[0] = Sensor_value[setup_size].regi;
        if(FIFO_CTRL1 == Sensor_value[setup_size].regi)
          buffer[1] = data_size;
        else
          buffer[1] = Sensor_value[setup_size].data;
        I2C_Transmit(buffer, 2);
    }
}
static void sensor_debug_init(void)
{
    uint8_t buffer[2];
        NRF_LOG_INFO("debug_Sensor_value = %d", sizeof(debug_Sensor_value));

   for (int setup_size = 0;
     setup_size < sizeof(debug_Sensor_value)/sizeof(debug_Sensor_value[0]);
     setup_size++)
    {
        buffer[0] = debug_Sensor_value[setup_size].regi;
        if(FIFO_CTRL1 == debug_Sensor_value[setup_size].regi)
          buffer[1] = data_size;
        else
          buffer[1] = debug_Sensor_value[setup_size].data;
        I2C_Transmit(buffer, 2);
    }
}


bool Sensor_init(bool debug)
{


    uint8_t reg;
    uint8_t val;

    twi_init();
    if(Sensor_Get_Id() == false)
    {
      twi_deinit();
      return false;
    }
    
   Sensor_sleep();
 
    if(debug == true)
    {
        sensor_debug_init();
    }
    else 
    {
        sensor_normal_init();
    }
    (void)fifo_get_size();

    twi_deinit();
    return true;
}

void sensor_enable(void)
{
    sensor_exfire = true;
}


float enmo_get(int16_t raw_x, int16_t raw_y, int16_t raw_z)
{

    float acc_x, acc_y, acc_z;
    float norm, enmo;


// Raw 데이터를 g 단위 실수(float)로 변환
    acc_x = (float)raw_x * XL_SENS;
    acc_y = (float)raw_y * XL_SENS;
    acc_z = (float)raw_z * XL_SENS;
    if(debugmode_flag)
    {
      Sensor_RawData.ax = (int16_t)(acc_x * 1000);
      Sensor_RawData.ay = (int16_t)(acc_y * 1000);
      Sensor_RawData.az = (int16_t)(acc_z * 1000);
    }
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

float gyro_intensity_get(int16_t raw_x, int16_t raw_y, int16_t raw_z)
{
    float gyro_x, gyro_y, gyro_z;
    float gvm;

    // 1. Raw 데이터를 dps(Degree per Second, 초당 회전 각도) 단위 실수로 변환
    // LSM6DSV16BX의 자이로 Full Scale 설정에 맞는 감도(Sensitivity)를 곱해줍니다.
    // 여기서는 예시로 2000dps 설정일 때의 감도를 곱한다고 가정합니다.
    gyro_x = (float)raw_x * GY_SENS;
    gyro_y = (float)raw_y * GY_SENS;
    gyro_z = (float)raw_z * GY_SENS;
    if(debugmode_flag)
    {
      Sensor_RawData.gx = (int16_t)(gyro_x * 10.0f);
      Sensor_RawData.gy = (int16_t)(gyro_y * 10.0f);
      Sensor_RawData.gz = (int16_t)(gyro_z * 10.0f);
    }
    // 2. 3축 벡터 합성 (회전의 세기 구하기)
    gvm = sqrtf((gyro_x * gyro_x) + (gyro_y * gyro_y) + (gyro_z * gyro_z));

    // 3. 센서가 가만히 있어도 발생하는 미세한 노이즈(바이어스) 제거 보정
    // 가만히 놔뒀을 때 dps가 약 1~2도 정도 흔들린다면 그 이하 값은 0으로 자릅니다.
    if (gvm < 2.0f) { 
        gvm = 0.0f;
    }

    return gvm; // 최종 회전 운동량 세기(dps 단위) 반환
}

uint32_t parse_fifo(uint8_t *buf, uint32_t len)
{ 
    float total_enmo = 0.0f;
    float total_gyro = 0.0f;
    uint16_t acc_samples = 0;
    uint16_t gyro_samples = 0;
    for (int i = 0; i < len; i++)
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
         else if (tag == LSM6DSV16B_GY_NC_TAG) 
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


uint32_t Sensor_Get_data(uint8_t* data, uint16_t len)
{
     uint8_t reg = 0x78;
      uint8_t tag = 0;
      uint8_t cnt = 0;
      // 1. register address write
      I2C_Transmitreceive(&reg, 1,data, len);
      //tag =  (data[0] & 0xf8) >> 3; 
           
      return parse_fifo(data, data_size);
}

bool Sensor_Get_Id(void)
{
    uint8_t reg = 0x0F;
    uint8_t value = 0;
    I2C_Transmitreceive(&reg,1,&value,1);
    //nrfx_twi_tx(&m_twi, LSM6DSV_ADDR, &reg, 1, true);
    //nrfx_twi_rx(&m_twi, LSM6DSV_ADDR, &value, 1);

    NRF_LOG_INFO("WHO_AM_I = %x", value);
    if(value == 0x71) return true;
    else return false;
}


static bool rx_flag = false;
void lsb6_ble_data(Motion_Packet_t* rx_packet)
{
    // 1. 포인터 검사 및 데이터 길이 유효성 예외 처리
    if (rx_packet == NULL) return;

    memcpy(&next_data,rx_packet,sizeof(Motion_Packet_t));

    rx_flag = true;
}

void sensor_flag_input(void)
{
  if(rx_flag == true)
  {
    rx_flag = false;

    uint8_t len = next_data.lsm6_data_req_res.data_len;
    uint8_t start_addr = next_data.lsm6_data_req_res.start_address;
    uint8_t tx_buf[17]; 
    bool send_flag = false;
    // 최대 16바이트 제한 초과 시 실행 안 함 (메모리 오염 방지)
    if (len == 0 || len > 16) return;
    twi_init();
  
    switch(next_data.lsm6_data_req_res.read_write)
    {
      case 0:
        I2C_Transmitreceive(&start_addr, 1, next_data.lsm6_data_req_res.data, len);
        send_flag = true;
        twi_deinit();
      break;
      case 1:
        tx_buf[0] = start_addr;
        memcpy(&tx_buf[1], next_data.lsm6_data_req_res.data, len);

        // 단 한 번의 I2C 트랜잭션으로 (len + 1) 바이트 연속 전송
        I2C_Transmit(tx_buf, len + 1);
        send_flag = true;
       twi_deinit();
      break;
      case 2:
        twi_deinit();
        debugmode_flag = true;
        XL_SENS = LSM6DSV_XL_SENSITIVITY_8G;
        GY_SENS = LSM6DSV_GY_SENSITIVITY_2000DPS;
        data_size = 2;
        Sensor_init(true);
      break;
      case 3:
        twi_deinit();
        debugmode_flag = false;
        XL_SENS = LSM6DSV_XL_SENSITIVITY_2G;
        GY_SENS = LSM6DSV_GY_SENSITIVITY_2000DPS;
        data_size = DATA_SIZE;
        Sensor_init(false);
      break;
    }


    if(send_flag)
    {
      next_data.event_code = LSM6_DATA_RESPONSE;
      BLE_Send_byte((uint8_t*)&next_data,sizeof(Motion_Packet_t));
    }
  }
}