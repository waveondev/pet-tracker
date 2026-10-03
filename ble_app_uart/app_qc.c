#include "app_qc.h"
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "ble_nus_c.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "app_flash.h"

/* DFUMODE 1 */
//FLASH1 RX 0x0 0x100000 RAM1 RWX 0x20000000 0x40000 uicr_bootloader_start_address RX 0x10001014 0x4

/* DFUMODE 0 */
//FLASH1 RX 0x0 0x100000 RAM1 RWX 0x20000000 0x40000 

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include "main.h"
#include "app_peripheral_con.h"
#include "app_central_con.h"
#include "app_gpio.h"
#include "app_adc.h"
#include "app_twi.h"

#include "app_sensor_flash.h"
#include "app_ble_tx.h"
#include "nrfx_gpiote.h"
#include "app_sensor.h"
#include "app_wdg.h"
APP_TIMER_DEF(m_sensor_timer);
APP_TIMER_DEF(m_redled_timer);
APP_TIMER_DEF(m_blueled_timer);
APP_TIMER_DEF(m_greenled_timer);

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)
#define SLAVE_LATENCY                   28


#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(6000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */



NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */

APP_TIMER_DEF(m_qc_timer);

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;



    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
    uint8_t state = 0;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
                 NRF_LOG_INFO("Connect");
        break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            uint8_t reason =
            p_ble_evt->evt.gap_evt.params.disconnected.reason;            
            
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}







/****************** QC GPIO*************************/
static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

    switch(pin)
    {
        case PKEY_STAT_SW_PIN:
            if (nrf_gpio_pin_read(PKEY_STAT_SW_PIN))
            {
                nrf_gpio_pin_clear(LED_BLUE_PIN);
            } 
            else
            {
                nrf_gpio_pin_set(LED_BLUE_PIN);
            } 
        break;
        case VBUS_IN_PIN:
          
        break;
        case LSM_INT_PIN:
            if (nrf_gpio_pin_read(LSM_INT_PIN))
            {
                //NRF_LOG_INFO("LSM_INT_PIN input1\r\n");
                sensor_enable();
            }
        break;
        
    }


}
void app_qc_gpio_init(void)
{

    nrfx_gpiote_in_config_t pek_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    APP_ERROR_CHECK(nrfx_gpiote_in_init(PKEY_STAT_SW_PIN, &pek_config, gpio_handler));
    nrfx_gpiote_in_event_enable(PKEY_STAT_SW_PIN, true);


    nrfx_gpiote_in_config_t vbus_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    vbus_config.pull = NRF_GPIO_PIN_NOPULL;   // 필요시 PULLDOWN으로 변경
    APP_ERROR_CHECK(nrfx_gpiote_in_init(VBUS_IN_PIN, &vbus_config, gpio_handler));
    nrfx_gpiote_in_event_enable(VBUS_IN_PIN, true);
   
    nrfx_gpiote_in_config_t lsm_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    lsm_config.pull = NRF_GPIO_PIN_NOPULL;   // 필요시 PULLDOWN으로 변경
    APP_ERROR_CHECK(nrfx_gpiote_in_init(LSM_INT_PIN, &lsm_config, gpio_handler));
    nrfx_gpiote_in_event_enable(LSM_INT_PIN, true);

    nrf_gpio_cfg_input(BAT_STAT_PIN, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_output(LED_GREEN_PIN);
    nrf_gpio_cfg_output(LED_BLUE_PIN);
    nrf_gpio_cfg_output(LED_RED_PIN);
    nrf_gpio_cfg_output(POWER_HOLD_PIN);
    nrf_gpio_pin_clear(POWER_HOLD_PIN);

    uint32_t start_tick = app_timer_cnt_get();
    uint32_t target_ticks = APP_TIMER_TICKS(2000); // 3000ms를 틱 단위로 변환

    nrf_gpio_pin_set(LED_GREEN_PIN);
    nrf_gpio_pin_set(LED_BLUE_PIN);
    nrf_gpio_pin_set(LED_RED_PIN);
    while (app_timer_cnt_diff_compute(app_timer_cnt_get(), start_tick) < target_ticks)
    {

    }
    nrf_gpio_pin_clear(LED_GREEN_PIN);
    nrf_gpio_pin_clear(LED_BLUE_PIN);
    nrf_gpio_pin_clear(LED_RED_PIN);


}

/****************** QC GPIO*************************/
extern void BLE_Init(void);
uint32_t error_flag = 0;

void error_set_flag(uint32_t flag)
{
  error_flag = flag;
  nrf_gpio_pin_clear(LED_GREEN_PIN);
  nrf_gpio_pin_clear(LED_BLUE_PIN);
  nrf_gpio_pin_clear(LED_RED_PIN);
  switch(error_flag)
  {
    case FLASH_ERROR:
        led_timer_start(false,false,true,100);
    break;
    case ADC_ERROR:
        led_timer_start(true,false,false,100);
    break;
    case MOTION_ERROR:
        led_timer_start(false,true,false,100);
    break;
    case BLE_ERROR:
        led_timer_start(true,true,true,100);
    break;
    default:
    break;
  }
}
uint8_t Motion_gpio_timeout = 5;
void Motion_interrupt(void)
{
    Motion_gpio_timeout = 5;
}

static void sensor_timer_handler(void * p_context)
{

    uint32_t battery = battery_voltage_get();
    
    if(battery < 4300 && battery > 3400)
    {
    
    }
    else
      error_set_flag(ADC_ERROR);
    Motion_gpio_timeout--;
    if(Motion_gpio_timeout == 0)
        error_set_flag(MOTION_ERROR);
}

static void QC_RED_timer_handler(void * p_context)
{
      nrf_gpio_pin_toggle(LED_RED_PIN);
}
static void QC_BLUE_timer_handler(void * p_context)
{
      nrf_gpio_pin_toggle(LED_BLUE_PIN);
}
static void QC_GREEN_timer_handler(void * p_context)
{
      nrf_gpio_pin_toggle(LED_GREEN_PIN);
}

void led_timer_start(bool r, bool g, bool b, uint32_t next_time)
{
  uint32_t timer_tick = APP_TIMER_TICKS(next_time);
  if(r)
  {
    app_timer_stop(m_redled_timer);
    app_timer_start(m_redled_timer, timer_tick, NULL);
  }
  if(g)
  {
    app_timer_stop(m_greenled_timer);
    app_timer_start(m_greenled_timer, timer_tick, NULL);
  }
  if(b)
  {
    app_timer_stop(m_blueled_timer);
    app_timer_start(m_blueled_timer, timer_tick, NULL);
  }
}


void app_qc_timer_create(void)
{
  app_timer_create(&m_redled_timer, APP_TIMER_MODE_REPEATED, QC_RED_timer_handler);
  app_timer_create(&m_blueled_timer, APP_TIMER_MODE_REPEATED, QC_BLUE_timer_handler);
  app_timer_create(&m_greenled_timer, APP_TIMER_MODE_REPEATED, QC_GREEN_timer_handler);
}


void app_qc_mode(void)
{
  ret_code_t err_code;
  app_qc_gpio_init();

  app_qc_timer_create();
  //saadc_init();
  wdt_init();
  if(flash_init() != 0)
    error_set_flag(FLASH_ERROR);


  if(Sensor_init(false) == false)
    error_set_flag(MOTION_ERROR);
    
  saadc_timer_handler(NULL);
  Sensor_update();
  BLE_Init();
  App_advertising_start("T100-QC", 0,0);

  err_code = app_timer_create(&m_sensor_timer,\
                          APP_TIMER_MODE_REPEATED,\
                          sensor_timer_handler);

  APP_ERROR_CHECK(err_code);
  app_timer_start(m_sensor_timer, APP_TIMER_TICKS(1000), NULL);

  while(1)
  {
    Sensor_update();
    idle_state_handle();
  }
}