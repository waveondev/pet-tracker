#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// nRF5 SDK 필수 헤더 파일들
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"  // SoftDevice용 fstorage 드라이버 헤더
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "sdk_errors.h"       // ret_code_t 정의 헤더
#include "app_flash.h"

