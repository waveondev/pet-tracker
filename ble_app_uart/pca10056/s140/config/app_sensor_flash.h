

#ifndef __APP_SENSOR_FLASH_H__
#define __APP_SENSOR_FLASH_H__

#include "stdint.h"


#define MOTION_START_RESPONSE 0x10
#define MOTION_DATA           0x11
#define HEALTH_DATA_REQUEST   0x12
#define HEALTH_DATA_RESPONSE  0x13

#define MOTION_START_REQUEST  0x20
#define MOTION_DATA_ACK       0x21

#define OTA_MODE_REQUEST      0x30
#define OTA_MODE_RESPONSE     0x31


#define FACTORY_REQUEST       0xF0
#define FACTORY_RESPONSE      0xF1


typedef struct {
    uint32_t seq;
    uint32_t target_seq;
    uint32_t last_send_seq;
    uint32_t start_send_seq;
    uint32_t count;
    uint32_t data;
    uint8_t  reserved[100]; 
} retained_data_t;



typedef union {
    struct {
        uint16_t data : 14;  
        uint16_t type : 2;  
    } bit;                   
    uint16_t word;           
} pack_data;

typedef union {
      struct {
        uint8_t Bat_Status : 2;  
        uint8_t IMU_Err : 1;  
        uint8_t BLE_Err : 1;  
        uint8_t storage : 1;
        uint8_t reset_reason : 2;
        uint8_t reserved : 1;
    } bit;    
    uint8_t byte;
} fault_code;

#pragma pack(push, 1)
typedef struct {
  uint8_t event_code;
  union
  {
      struct
      {
        uint8_t interval;
        uint16_t total_points;
        uint16_t padding[8];
      } motion_req;
      struct
      {
        uint8_t seq;
        pack_data pack_data_0;
        pack_data pack_data_1;
        pack_data pack_data_2;
        pack_data pack_data_3;
        pack_data pack_data_4;
        pack_data pack_data_5;
        pack_data pack_data_6;
        pack_data pack_data_7;
        pack_data pack_data_8;
      } motion_data;
      struct
      {
        uint8_t req_type;
        uint16_t padding[9];
      } health_data_req;
      struct
      {
        uint32_t uptime_sec;
        uint8_t Bat_Level;
        uint8_t Bat_Voltage;
        uint8_t major;
        uint8_t minor;
        uint8_t patch;
        int8_t target_rssi;
        fault_code fault_flag;
        uint16_t padding[4];
      } health_data_res;
      struct
      {
        uint8_t req_type;
        uint16_t padding[9];
      } motion_res;
      struct
      {
        uint8_t ack_seq_no;
        uint16_t padding[9];
      } motion_ack;
      struct
      {
        uint8_t cmd_type;
        uint16_t padding[9];
      } ota_req;
      struct
      {
        uint8_t cmd_type;
        uint8_t status;
        uint8_t padding[17];
      } ota_res;

  };
} Motion_Packet_t;
#pragma pack(pop)


retained_data_t* Sensor_Get_Seq(void);
void Sensor_flash_set(uint32_t data);
void sensor_send_timer_init(void);

#endif
