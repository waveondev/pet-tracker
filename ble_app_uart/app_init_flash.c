#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "fds.h"
#include "nrf_log.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "app_init_flash.h"

static volatile bool m_fds_init_done  = false;
static volatile bool m_fds_write_done = false;
static volatile bool m_fds_gc_done    = false;
static volatile bool m_fds_del_done   = false;

// FDS 비동기 write 시 데이터 유지를 위한 4바이트 정렬 static 버퍼
static uint32_t m_fds_write_buf[32] __attribute__((aligned(4)));

// 디폴트 값을 0xFFFFFFFF로 초기화
static Factory_t factory_value = {
    .active_time = FDS_DEFAULT_EMPTY_VAL,
    .bat_count   = 0
};

// 저장/초기화 요청 플래그 분리
static volatile bool m_save_date_flag = false;
static volatile bool m_save_bat_flag  = false;
static volatile bool m_erase_all_flag = false;

/**
 * @brief FDS 이벤트 콜백 핸들러
 */
static void fds_evt_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
               
            }
            else
            {
                NRF_LOG_ERROR("FDS init failed: 0x%X", p_evt->result);
            }
             m_fds_init_done = true;
            break;

        case FDS_EVT_WRITE:
        case FDS_EVT_UPDATE:
            if (p_evt->result == NRF_SUCCESS)
            {
                
            }
            else
            {
                NRF_LOG_ERROR("FDS write/update failed: 0x%X", p_evt->result);
            }
            m_fds_write_done = true;
            break;

        case FDS_EVT_DEL_RECORD:
        case FDS_EVT_DEL_FILE:
            if (p_evt->result == NRF_SUCCESS)
            {
               
            }
            else
            {
                NRF_LOG_ERROR("FDS delete failed: 0x%X", p_evt->result);
            }
             m_fds_del_done = true;
            break;

        case FDS_EVT_GC:
            if (p_evt->result == NRF_SUCCESS)
            {
                
            }
            else
            {
                NRF_LOG_ERROR("FDS GC failed: 0x%X", p_evt->result);
            }
            m_fds_gc_done = true;
            break;

        default:
            break;
    }
}
#include "app_qc.h"
/**
 * @brief FDS 초기화 (BLE_Init 이후 호출)
 */
static void app_fds_init(void)
{
    ret_code_t rc;

    rc = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(rc);

    rc = fds_init();
    APP_ERROR_CHECK(rc);

    while (!m_fds_init_done)
    {
        sd_app_evt_wait();
    }

    uint32_t const bl_addr   = NRF_UICR->NRFFW[0];
    uint32_t const end_addr = (bl_addr != 0xFFFFFFFF) ? bl_addr : (0x77000);
    uint32_t const fds_end   = end_addr - (FDS_VIRTUAL_PAGES_RESERVED * 0x1000);
    uint32_t const fds_start = fds_end  - (FDS_VIRTUAL_PAGES * 0x1000);
    NRF_LOG_INFO("FDS Ready Range: 0x%08X 0x%08X ~ 0x%08X", bl_addr, fds_start, fds_end - 1);
}

/**
 * @brief Garbage Collection 실행 (페이지 꽉 찼을 때 자동 호출됨)
 */
static ret_code_t run_garbage_collection(void)
{
    NRF_LOG_INFO("FDS Full -> Running GC...");
    m_fds_gc_done = false;

    ret_code_t rc = fds_gc();
    if (rc != NRF_SUCCESS) return rc;

    while (!m_fds_gc_done)
    {
        sd_app_evt_wait();
    }
    NRF_LOG_INFO("FDS GC Complete.");
    return NRF_SUCCESS;
}

/**
 * @brief 구조체/데이터 저장 공통 함수
 */
static ret_code_t app_fds_save_struct(uint16_t file_id, uint16_t key, const void * p_data, uint32_t byte_len)
{
    if (p_data == NULL || byte_len == 0 || (byte_len % 4) != 0 || byte_len > sizeof(m_fds_write_buf))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memcpy(m_fds_write_buf, p_data, byte_len);

    fds_record_t        record;
    fds_record_desc_t   desc = {0};
    fds_find_token_t    ftok = {0};

    record.file_id           = file_id;
    record.key               = key;
    record.data.p_data       = m_fds_write_buf;
    record.data.length_words = byte_len / 4;

    m_fds_write_done = false;
    ret_code_t rc;

    if (fds_record_find(file_id, key, &desc, &ftok) == NRF_SUCCESS)
    {
        rc = fds_record_update(&desc, &record);
    }
    else
    {
        rc = fds_record_write(&desc, &record);
    }

    if (rc == FDS_ERR_NO_SPACE_IN_FLASH)
    {
        rc = run_garbage_collection();
        if (rc != NRF_SUCCESS) return rc;

        memset(&ftok, 0, sizeof(ftok));
        if (fds_record_find(file_id, key, &desc, &ftok) == NRF_SUCCESS)
            rc = fds_record_update(&desc, &record);
        else
            rc = fds_record_write(&desc, &record);
    }

    if (rc != NRF_SUCCESS) return rc;

    while (!m_fds_write_done)
    {
        sd_app_evt_wait();
    }

    return NRF_SUCCESS;
}

/**
 * @brief 구조체/데이터 읽기 공통 함수
 */
static bool app_fds_load_struct(uint16_t file_id, uint16_t key, void * p_out_data, uint32_t byte_len)
{
    fds_record_desc_t   desc = {0};
    fds_find_token_t    ftok = {0};
    fds_flash_record_t  flash_record = {0};

    if (fds_record_find(file_id, key, &desc, &ftok) != NRF_SUCCESS)
    {
        return false;
    }

    if (fds_record_open(&desc, &flash_record) != NRF_SUCCESS)
    {
        return false;
    }

    uint32_t stored_bytes = flash_record.p_header->length_words * 4;
    uint32_t copy_len = (byte_len < stored_bytes) ? byte_len : stored_bytes;
    memcpy(p_out_data, flash_record.p_data, copy_len);

    fds_record_close(&desc);

    return true;
}

static ret_code_t app_fds_save_u32(uint16_t file_id, uint16_t key, uint32_t value)
{
    return app_fds_save_struct(file_id, key, &value, sizeof(uint32_t));
}

static bool app_fds_load_u32(uint16_t file_id, uint16_t key, uint32_t * p_out_value)
{
    return app_fds_load_struct(file_id, key, p_out_value, sizeof(uint32_t));
}

/**
 * @brief 1. 부팅 시 초기화 및 데이터 불러오기 (BLE_Init 이후 1회 호출)
 *        최초 부팅 시 저장된 값이 없으면 둘 다 0xFFFFFFFF로 디폴트 저장
 */
void system_fds_startup(void)
{

    app_fds_init();

    // 1) DATE(active_time) 로드 -> 없으면 0xFFFFFFFF 디폴트 저장
    if (!app_fds_load_u32(APP_FDS_FILE_ID, APP_FDS_KEY_DATE, &factory_value.active_time))
    {
        factory_value.active_time = FDS_DEFAULT_EMPTY_VAL;
        app_fds_save_u32(APP_FDS_FILE_ID, APP_FDS_KEY_DATE, factory_value.active_time);
    }

    // 2) BAT(bat_count) 로드 -> 없으면 0xFFFFFFFF 디폴트 저장
    if (!app_fds_load_u32(APP_FDS_FILE_ID, APP_FDS_KEY_BAT, &factory_value.bat_count))
    {
        factory_value.bat_count = 0;
        app_fds_save_u32(APP_FDS_FILE_ID, APP_FDS_KEY_BAT, factory_value.bat_count);
    }

    NRF_LOG_INFO("FDS Loaded -> DATE: 0x%08X, BAT: 0x%08X",
                 factory_value.active_time, factory_value.bat_count);
}

/**
 * @brief 2. 현재 메모리에 로드된 DATE, BAT 값 읽기
 */
void fds_factory_read(uint32_t * p_date, uint32_t * p_bat)
{
    if (p_date != NULL)
    {
        *p_date = factory_value.active_time;
    }
    if (p_bat != NULL)
    {
        *p_bat = factory_value.bat_count;
    }
}

/**
 * @brief 3. DATE 쓰기 요청 (기존 값이 0xFFFFFFFF일 때만 최초 1회 기록, 아니면 무시)
 */
void fds_factory_write_date(uint32_t date)
{
    // 현재 저장된 값이 0xFFFFFFFF가 아니면 이미 기록된 것이므로 쓰지 않음
    if (factory_value.active_time != FDS_DEFAULT_EMPTY_VAL)
    {
        return;
    }

    // 들어온 값도 0xFFFFFFFF가 아닐 때만 저장 요청
    if (date != FDS_DEFAULT_EMPTY_VAL)
    {
        factory_value.active_time = date;
        m_save_date_flag = true;
    }
}

/**
 * @brief 4. BAT 쓰기 요청 (호출될 때마다 계속 덮어쓰기)
 */
void fds_factory_write_bat(uint32_t bat)
{
    factory_value.bat_count = bat;
    m_save_bat_flag = true;
}

/**
 * @brief 5. 공장 초기화 (파일 삭제 후 GC 실행, 둘 다 0xFFFFFFFF로 재초기화)
 */
ret_code_t fds_factory_erase_all(void)
{
    m_fds_del_done = false;
    ret_code_t rc = fds_file_delete(APP_FDS_FILE_ID);
    if (rc != NRF_SUCCESS) return rc;

    while (!m_fds_del_done)
    {
        sd_app_evt_wait();
    }

    rc = run_garbage_collection();
    if (rc != NRF_SUCCESS) return rc;

    // 초기화 후 다시 0xFFFFFFFF 디폴트 상태로 복원
    factory_value.active_time = FDS_DEFAULT_EMPTY_VAL;
    factory_value.bat_count   = 0;
    app_fds_save_u32(APP_FDS_FILE_ID, APP_FDS_KEY_DATE, FDS_DEFAULT_EMPTY_VAL);
    app_fds_save_u32(APP_FDS_FILE_ID, APP_FDS_KEY_BAT, FDS_DEFAULT_EMPTY_VAL);

    return NRF_SUCCESS;
}

/**
 * @brief 6. main의 while(1) 루프 안에서 실행할 실제 Flash 기록 처리 함수
 */
void app_fds_process(void)
{
    if (m_save_date_flag)
    {
        m_save_date_flag = false;
        ret_code_t rc = app_fds_save_u32(APP_FDS_FILE_ID, APP_FDS_KEY_DATE, factory_value.active_time);
        if (rc == NRF_SUCCESS)
        {
            NRF_LOG_INFO("FDS DATE Saved: %u (0x%08X)", factory_value.active_time, factory_value.active_time);
        }
    }

    if (m_save_bat_flag)
    {
        m_save_bat_flag = false;
        ret_code_t rc = app_fds_save_u32(APP_FDS_FILE_ID, APP_FDS_KEY_BAT, factory_value.bat_count);
        if (rc == NRF_SUCCESS)
        {
            NRF_LOG_INFO("FDS BAT Saved: %u", factory_value.bat_count);
        }
    }
}