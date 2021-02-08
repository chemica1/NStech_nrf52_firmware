#include "nordic_common.h"
#include "nrf_drv_twi.h"
#include "bsp.h"
#include "gsil_drv_twi.h"
#include "nrf_delay.h"

/* TWI PIN numbers. */
#define SCL_PIN_1           15
#define SDA_PIN_1           14
#define SCL_PIN_2           8
#define SDA_PIN_2           7

/* TWI instance ID. */
#define TWI_INSTANCE_ID_0     0
#define TWI_INSTANCE_ID_1     1



/* TWI instance. */
static const nrf_drv_twi_t m_twi_1 = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID_0);
static const nrf_drv_twi_t m_twi_2 = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID_1);


/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

static volatile enum {TWI_IDLE_1, TWI_TX_1, TWI_RX_1, TWI_JUST_TX_1} m_twi_status_1;
static volatile uint8_t  m_twi_addr_1;
static volatile uint8_t  m_recv_len_1;
static volatile uint8_t* m_recv_buf_1;
static volatile gsil_drv_twi_done_handler_t m_recv_done_handler_1;

static volatile enum {TWI_IDLE_2, TWI_TX_2, TWI_RX_2, TWI_JUST_TX_2} m_twi_status_2;
static volatile uint8_t  m_twi_addr_2;
static volatile uint8_t  m_recv_len_2;
static volatile uint8_t* m_recv_buf_2;
static volatile gsil_drv_twi_done_handler_t m_recv_done_handler_2;

static uint8_t ctrl_reg1_1hz[2] = {0x20, 0x10 | 0x0F  };
static uint8_t ctrl_reg1_10hz[2] = {0x20, 0x20 | 0x0F  };

static uint8_t b0_cmd_pwr[2] = {0x06, 0x61};
static uint8_t b0_cmd_int_cfg[2] = {0x0F, 0x20};
static uint8_t b0_cmd_int_en[2] = {0x10, 0x08};
static uint8_t b2_cmd_wom_en[2] = {0x12, 0x02};
static uint8_t cmd_usr0[2] = {0x7F, 0x00};
static uint8_t cmd_usr2[2] = {0x7F, 0x20};

/* Buffer for samples read from each sensor. */
static uint8_t m_sample;

uint8_t test_buf[16];

//uint8_t test_buf_2[16];

extern uint8_t g_PS_Data[6];   // rx -> m_recv_buf_1(nrf_drv_twi_rx) -> buf(get_register_by_twi_1) -> extern g_ACC_Data
extern uint8_t g_ACC_Data[6];

//nrf_drv_twi_evt_t <- Structure for a TWI event.
void twi_handler_1(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    ret_code_t err_code;
    switch (p_event->type) // nrf_drv_twi_evt_t's type.
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch( p_event->xfer_desc.type )
            {
                case NRF_DRV_TWI_XFER_TX :
                    if( m_twi_status_1 == TWI_TX_1) 
                    {
                      m_twi_status_1 = TWI_RX_1;
                      err_code = nrf_drv_twi_rx(&m_twi_1, m_twi_addr_1, m_recv_buf_1, m_recv_len_1); // rx -> m_recv_buf_1(nrf_drv_twi_rx) -> buf(get_register_by_twi_1) -> extern g_ACC_Data
                      APP_ERROR_CHECK(err_code);
                      return;
                    }
                    m_twi_status_1 = TWI_IDLE_1;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    m_recv_done_handler_1(m_recv_buf_1, m_recv_len_1);
                    
                    break;                
            }
            break;
        default:
            break;
    }
    m_twi_status_1 = TWI_IDLE_1;
}

void twi_handler_2(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    ret_code_t err_code;
    switch (p_event->type) // nrf_drv_twi_evt_t's type.
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch( p_event->xfer_desc.type )
            {
                case NRF_DRV_TWI_XFER_TX :
                    if( m_twi_status_2 == TWI_TX_2) 
                    {
                      m_twi_status_2 = TWI_RX_2;
                      err_code = nrf_drv_twi_rx(&m_twi_2, m_twi_addr_2, m_recv_buf_2, m_recv_len_2);
                      APP_ERROR_CHECK(err_code);
                      return;
                    }
                    m_twi_status_2 = TWI_IDLE_2;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    m_recv_done_handler_2(m_recv_buf_2, m_recv_len_2);
                    
                    break;                
            }
            break;
        default:
            break;
    }
    m_twi_status_2 = TWI_IDLE_2;
}



void gsil_drv_twi_uninit(void)
{

    nrf_drv_twi_disable(&m_twi_1);
    nrf_drv_twi_uninit(&m_twi_1);
}


/**
 * @brief TWI initialization.
 */


void gsil_drv_twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_1_config = {
       .scl                = SCL_PIN_1,
       .sda                = SDA_PIN_1,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
   
    err_code = nrf_drv_twi_init(&m_twi_1, &twi_1_config, twi_handler_1, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi_1);

}


bool get_register_by_twi_1(uint8_t twi_addr, uint8_t reg_addr, uint8_t* buf, uint8_t len, gsil_drv_twi_done_handler_t done_handler)
{
    ret_code_t err_code;
 
    if( m_twi_status_1 != TWI_IDLE_1 ) { 
      //printf(" Status is Not Idle.... ");
      return false; 
    }
    m_twi_addr_1 = twi_addr;
    m_recv_buf_1 = buf;
    m_recv_len_1 = len;
    m_recv_done_handler_1 = done_handler;

    m_twi_status_1 = TWI_TX_1;
    err_code = nrf_drv_twi_tx(&m_twi_1, twi_addr, &reg_addr, 1, true);
    APP_ERROR_CHECK(err_code);

    return true;
}

bool get_register_by_twi_2(uint8_t twi_addr, uint8_t reg_addr, uint8_t* buf, uint8_t len, gsil_drv_twi_done_handler_t done_handler)
{
    ret_code_t err_code;
 
    if( m_twi_status_2 != TWI_IDLE_2 ) { 
      //printf(" Status is Not Idle.... ");
      return false; 
    }
    m_twi_addr_2 = twi_addr;
    m_recv_buf_2 = buf;
    m_recv_len_2 = len;
    m_recv_done_handler_2 = done_handler;

    m_twi_status_2 = TWI_TX_2;
    err_code = nrf_drv_twi_tx(&m_twi_2, twi_addr, &reg_addr, 1, true);
    APP_ERROR_CHECK(err_code);

    return true;
}



bool send_data_by_twi_1(uint8_t twi_addr, uint8_t* send_data, uint8_t len)
{
    ret_code_t err_code;

    if( m_twi_status_1 != TWI_IDLE_1 ) { return false; }
    m_twi_status_1 = TWI_JUST_TX_1;
    err_code = nrf_drv_twi_tx(&m_twi_1, twi_addr, send_data, len, false);
    APP_ERROR_CHECK(err_code);
}

bool send_data_by_twi_2(uint8_t twi_addr, uint8_t* send_data, uint8_t len)
{
    ret_code_t err_code;

    if( m_twi_status_2 != TWI_IDLE_2 ) { return false; }
    m_twi_status_2 = TWI_JUST_TX_2;
    err_code = nrf_drv_twi_tx(&m_twi_2, twi_addr, send_data, len, false);
    APP_ERROR_CHECK(err_code);
}


void print_buf(void *recv_data, uint8_t len)
{

    return;

    /*

    uint8_t i;
    
    printf("(%d): ", len);

    if( len == 0 ) 
    {
        printf(" nothing");
        return;
    }
    
    for(i=0; i< len; i++ ) 
    {
        printf(" %02X", ((uint8_t*)(recv_data))[i]);
    }
    */
}

bool is_twi_ready_1(void)
{
    return m_twi_status_1 == TWI_IDLE_1?true:false;
}

bool is_twi_ready_2(void)
{
    return m_twi_status_2 == TWI_IDLE_2?true:false;
}


/*3-axis sensor*/

void lis2dh12tr_print_who_am_i()
{
    //printf("\nLIS2DH12TR WHO AM  I test(33 is expected)");
    if(!get_register_by_twi_1(LIS2DH12TR_ADDR, 0x0F, test_buf,1,print_buf)) 
    {
      //printf("\nlis2dh12tr print who am i failed!");
    }
} //(uint8_t twi_addr, uint8_t reg_addr, uint8_t* buf, uint8_t len, gsil_drv_twi_done_handler_t done_handler)


static uint8_t reg_data[4];

void lis2dh12tr_set_config()
{
  send_data_by_twi_1(LIS2DH12TR_ADDR, ctrl_reg1_10hz, 2);   while(!is_twi_ready_1());

  reg_data[0]=0x21;//CTRL_REG2
  reg_data[1]=0x01;//  High-pass filter enabled for AOI function on Interrupt 1.
  send_data_by_twi_1(LIS2DH12TR_ADDR, reg_data, 2);   while(!is_twi_ready_1());

  reg_data[0]=0x22;//CTRL_REG3
  reg_data[1]=0x40;//IA1 interrupt on INT1 pin.
  send_data_by_twi_1(LIS2DH12TR_ADDR, reg_data, 2);   while(!is_twi_ready_1());

  reg_data[0]=0x25;//CTRL_REG6
  reg_data[1]=0x02;//INT1 and INT2 pin polarity. active-low
  send_data_by_twi_1(LIS2DH12TR_ADDR, reg_data, 2);   while(!is_twi_ready_1());

  reg_data[0]=0x30;//  INT1_CFG register
  reg_data[1]=0x2A; //0A;//
  send_data_by_twi_1(LIS2DH12TR_ADDR, reg_data, 2);   while(!is_twi_ready_1());

  reg_data[0]=0x32;//Interrupt 1 threshold.
  reg_data[1]=0x25;//
  send_data_by_twi_1(LIS2DH12TR_ADDR, reg_data, 2);   while(!is_twi_ready_1()); 
}

void lis2dh12tr_print_info()
{
    //printf("\r\n info");
    get_register_by_twi_1(LIS2DH12TR_ADDR, 0x80 | 0x1E, test_buf,10,print_buf);
}

void lis2dh12tr_print_data()
{
    //printf("\r\n ACC Read");
    get_register_by_twi_1(LIS2DH12TR_ADDR, 0x80 | 0x28, g_ACC_Data,6,print_buf);

}





/*press sensor*/

void LPS22HB_print_who_am_i()
{
    //printf("\nLPS22HB WHO AM  I test(B1 is expected)");
    get_register_by_twi_2(LPS22HB_ADDR, LPS22HB_WHO_AM_I, test_buf, 1, print_buf);
}

static uint8_t ctrl_1[2] = {LPS22HB_RES_CONF, 0x0 };
static uint8_t ctrl_2[2] = {LPS22HB_CTRL_REG1 , 0x10 };

void LPS22HB_set_config()
{
  send_data_by_twi_2(LPS22HB_ADDR, ctrl_1, 2);   while(!is_twi_ready_2());
  send_data_by_twi_2(LPS22HB_ADDR, ctrl_2, 2);   while(!is_twi_ready_2());
}

void LPS22HB_print_data()
{
    //printf("\r\n PS Read ");
   
    get_register_by_twi_2(LPS22HB_ADDR, 0x80 | LPS22HB_PRES_OUT_XL, g_PS_Data,5,print_buf);

}

