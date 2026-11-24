#ifndef __APP_INIT_FLASH_H__
#define __APP_INIT_FLASH_H__
#include <stdint.h>
#include <stdbool.h>
#include "sdk_errors.h"

#define APP_FDS_FILE_ID         0x1001
#define APP_FDS_KEY_DATE        0x2001  // active_time 저장용 Key
#define APP_FDS_KEY_BAT         0x2002  // bat_count   저장용 Key

#define FDS_DEFAULT_EMPTY_VAL   0xFFFFFFFF

typedef struct __attribute__((aligned(4))) {
    uint32_t active_time;
    uint32_t bat_count;
} Factory_t;

void       system_fds_startup(void);
void       app_fds_process(void);

// 외부 호출용 읽기/쓰기/초기화 API
void       fds_factory_read(uint32_t * p_date, uint32_t * p_bat);
void       fds_factory_write_date(uint32_t date);
void       fds_factory_write_bat(uint32_t bat);
ret_code_t fds_factory_erase_all(void);



#endif