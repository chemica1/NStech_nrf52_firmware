#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <nrf_saadc.h>

#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "app_timer.h"
#include "app_uart.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_gpiote.h"
#include "gsil_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"


#define DEVICE_NAME                     "GSIL_EB"                       /**< Name of device. Will be included in the advertising data. */

#define LED1_On nrf_gpio_pin_write(13, 1)
#define LED1_Off nrf_gpio_pin_write(13, 0)

#define LED2_On nrf_gpio_pin_write(31, 1)
#define LED2_Off nrf_gpio_pin_write(31, 0)

#define BATTERY_CHECK_PIN               28
#define APP_BLE_CONN_CFG_TAG            1     //< A tag identifying the SoftDevice BLE configuration. The connection to configure.


#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_NUS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_NUS_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x0B //20.08.04 predata : 0x17     /**< Total length of information advertised by the Beacon. */
#define APP_COMPANY_IDENTIFIER          0xABCD                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define ADC12_COUNTS_PER_VOLT           4551

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_TIMEOUT_IN_SECONDS      180                                     /**< The advertising timeout in units of seconds. */

#define DEEP_SLEEP_TIMEOUT_IN_CNT       150                                    // 100 = about 2min
#define BATTERY_CHECK_TIMEOUT_IN_CNT    47
#define ADV_STEP_TIMEOUT_IN_CNT         10   


#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX]={0xAA,};  /**< Buffer for storing an encoded advertising set. */
static uint8_t              m_enc_scanrspdata[BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED];


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    0x03,   
    0x01,     
    0x02,
    0x01,

    0x00,    
    0x00,

    0x08,
         
    0x00,
    0x00,
    0x00,
    0x00
         
};


APP_TIMER_DEF(m_base_timer_id); 
APP_TIMER_DEF(m_btn_timer_id); 



uint16_t get_battery_voltage(void);
void Adc12bitPolledInitialise(void);

uint8_t g_ACC_Data[6];
uint8_t g_PS_Data[6];


void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0
    }
};


static void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                     (const uint8_t *)DEVICE_NAME,
                                      strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t ibeacon_advdata;

    ble_advdata_manuf_data_t manuf_specific_data;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;



      LED2_On;
    nrf_delay_ms(500);
    LED2_Off;
    m_beacon_info[index++] = MSB_16(major_value);


    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif
    
    // Build and set advertising data.
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data        = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size          = APP_BEACON_INFO_LENGTH;
    
    memset(&ibeacon_advdata, 0, sizeof(ibeacon_advdata));

    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; //law data [2]

    ibeacon_advdata.name_type             = BLE_ADVDATA_FULL_NAME;
    ibeacon_advdata.flags                 = flags;  // 04 law data [2] 
    ibeacon_advdata.p_manuf_specific_data = &manuf_specific_data;
   
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = MSEC_TO_UNITS(200, UNIT_0_625_MS);
    //m_adv_params.interval        = MSEC_TO_UNITS(50, UNIT_0_625_MS); //advertising interval
    m_adv_params.duration        = MSEC_TO_UNITS(1000, UNIT_0_625_MS);;       //when to stop advertising

  
    err_code = ble_advdata_encode(&ibeacon_advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, 0, 4);
    APP_ERROR_CHECK(err_code); 
 
}





/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) //init the softdevice(nordic protocol stack) ps) execute software interrupt
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request(); //Function for requesting to enable the SoftDevice. return : NRF_SUCCESS(The process is started.) / NRF_ERROR_INVALID_STATE

    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings. You cannot modify the Softdevice (BLE stack) since it is not open source.
    // Fetch the start address of the application RAM.
    /* The following configurations will be set:
     * Number of peripheral links
     * Number of central links
     * ATT MTU size (for the given connection)
     * Vendor specific UUID count
     * GATTS Attribute table size
     * Service changed
    */

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0; 
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start); // This function configures the BLE stack with the settings specified in the SoftDevice handler BLE configuration. //output :Application RAM start address.
  
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start); //Function for configuring and enabling the BLE stack. [in]Address of the start of application's RAM.

    APP_ERROR_CHECK(err_code);

    //NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */

static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


/******************sensor interrupt handler ************************/

static uint8_t sensor_handler_step = 0;
uint16_t sos_btn_cnt = 0;
uint16_t sos_step = 0;
uint16_t deepsleep_cnt = 1;
uint8_t advertising_handler_step = 0;
uint8_t btn_flag = 0;


static void btn_timeout_handler(void * p_context) // like time interrupt function
{
    bool btn_status;
    btn_status = nrfx_gpiote_in_is_set(BUTTON_1);
    if(btn_status == false)
    {
          btn_flag = 1;
    }

}

/***************advertising interrupt****************/

static void advertising_timeout_handler(void * p_context) // like time interrupt function
{


    if( btn_flag == 1)
    {
    
      advertising_handler_step ++;
      switch( advertising_handler_step )
      {
        case 1:
        
            LED2_On;
      
            advertising_init();
          
            break;

        case 3:
      
            advertising_start(deepsleep_cnt, sos_step);
    
            break; // if app timer 50ms , -> 50 * 60 = 3000ms, 3s 

        case 25:
        
            LED2_Off;
            sd_ble_gap_adv_stop(m_adv_handle);
      
            break;

        case 26:
            m_beacon_info[6]++;
            if(m_beacon_info[6] > 100 ) m_beacon_info[6] = 0;

            break;

        case 30:

            advertising_handler_step = 0; 
            btn_flag = 0;
            break;


        default: break;
      }
    }
}




/**@brief Function for starting advertising.
 */
void advertising_start(uint8_t deepsleep_cnt, uint8_t sos_step)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
       
       printf("sdfg");
            btn_flag = 1;
}

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init(); // preparing using app_timer driver
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_base_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                advertising_timeout_handler); //<-inturpt function name
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_btn_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                btn_timeout_handler); //<-inturpt function name
    APP_ERROR_CHECK(err_code);

}

void timers_start(void)
{
     ret_code_t err_code;

    err_code = app_timer_start(m_base_timer_id, APP_TIMER_TICKS(100), NULL); //attach time interrupt function
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(17);

    err_code = app_timer_start(m_btn_timer_id, APP_TIMER_TICKS(47), NULL); //attach time interrupt function
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;
    uint32_t ret_val;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;
            

            if (data_array[0] != ':')
            {
               index = 0;
               break;
            }

            if ( data_array[index-1] == ',')
            {
           
               for (int i = 0; i < index ; i++)   
               {
                    LED2_On;
                     nrf_delay_ms(100);
                     LED2_Off;
                    printf("hi");   
                    index=0;
               }
               
               break;
            }

            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

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



//dummy function
void change_mac_address(void)
{
    static ble_gap_addr_t m_central_addr;

    m_central_addr.addr_type     = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    m_central_addr.addr[0] = 0xFA; //(uint8_t)NRF_FICR->DEVICEADDR[0];
    m_central_addr.addr[1] = 0x03; //(uint8_t)(NRF_FICR->DEVICEADDR[0] >> 8);
    m_central_addr.addr[2] = 0x00; //(uint8_t)(NRF_FICR->DEVICEADDR[0] >> 16);
    m_central_addr.addr[3] = 0x00; //(uint8_t)(NRF_FICR->DEVICEADDR[0] >> 24);
    m_central_addr.addr[4] = 0x70; //(uint8_t)NRF_FICR->DEVICEADDR[1];
    m_central_addr.addr[5] = (0xFA | 0xC0); //(uint8_t)((NRF_FICR->DEVICEADDR[1] >> 8) | 0xC0); // 2MSB must be set 11

    sd_ble_gap_addr_set(&m_central_addr);
}

void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true);

    err_code = nrf_drv_gpiote_out_init(LED_1, &out_config); //LED1
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(LED_2, &out_config);  //LED2
    APP_ERROR_CHECK(err_code);

    LED1_Off;
    LED2_Off;

    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_HITOLO(true); // vs NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(BUTTON_1, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    //err_code = nrf_drv_gpiote_in_init(LIS_INT_PIN_NUMBER, &in_config, in_pin_handler);
    //APP_ERROR_CHECK(err_code);
    
    nrf_drv_gpiote_in_event_enable(BUTTON_1, true);
    //nrf_drv_gpiote_in_event_enable(LIS_INT_PIN_NUMBER, true);

    nrf_gpio_cfg_sense_input(BUTTON_1, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    //nrf_gpio_cfg_sense_input(LIS_INT_PIN_NUMBER, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW); //this interrupt will wake up the cpu

}


/**
 * @brief Function for application main entry.
 */


int main(void)
{
    ret_code_t err_code;

    // Initialize.
    log_init();
    timers_init();
    uart_init(); 
    leds_init();    
    power_management_init();
    ble_stack_init(); 
    gap_params_init();
    gpio_init();
    //change_mac_address();
    

    
/*
    NRF_SPI0->ENABLE = 0;
    NRF_UART0->TASKS_STOPTX = 1;
    NRF_UART0->TASKS_STOPRX = 1;
    NRF_UART0->ENABLE = 0;


    m_beacon_info[4] = 0x1B;
    m_beacon_info[5] = 0x0C;

    

    *(uint32_t *)(0x40003000 + 0xFFC ) = 0;
    *(volatile uint32_t *)0x40003FFC;
    *(uint32_t *)(0x40003000 + 0xFFC ) = 1;

    *(uint32_t *)(0x40004000 + 0xFFC ) = 0;
    *(volatile uint32_t *)0x40004FFC;
    *(uint32_t *)(0x40004000 + 0xFFC ) = 1;

    */
    timers_start();

    LED1_On;
    nrf_delay_ms(1000);
    LED1_Off;


  
    // Enter main loop.
    for (;; )
    {
        idle_state_handle();
    }
}
