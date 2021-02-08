#include "nordic_common.h"
#include "nrf_drv_twi.h"
#include "bsp.h"
#include "drv_twi_test.h"
#include "nrf_delay.h"
#define TWI_INSTANCE_ID     
//#define TWI_INSTANCE_ID_PRESS     1



/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_doe = false;
static volatile enum {TWI_IDLE, TWI_TX, TWI_RX, TWI_JUST_TX} m_twi_status;
static volatile uint8_t  m_twi_addr;
static volatile uint8_t  m_recv_len;
static volatile uint8_t* m_recv_buf;
static volatile gsil_drv_twi_done_handler_t m_recv_done_handler; //if twi ended, it will execute

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);




////////////////////////////////////attach twi /////////////////////////////////////
void gsil_drv_twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = LPS_SCL_PIN,
       .sda                = LPS_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
    m_twi_status = TWI_IDLE;
}

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

uint8_t test_buf[16];

//nrf_drv_twi_evt_t <- Structure for a TWI event.
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    ret_code_t err_code;
    switch (p_event->type) // nrf_drv_twi_evt_t's type.
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch( p_event->xfer_desc.type )
            {
                case NRF_DRV_TWI_XFER_TX :
                    if( m_twi_status == TWI_TX) 
                    {
                      m_twi_status = TWI_RX;
                      err_code = nrf_drv_twi_rx(&m_twi, m_twi_addr, m_recv_buf, m_recv_len);
                      APP_ERROR_CHECK(err_code);
                      return;
                    }
                    m_twi_status = TWI_IDLE;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    m_recv_done_handler(m_recv_buf, m_recv_len);
                    
                    break;                
            }
            break;
        default:
            break;
    }
    m_twi_status = TWI_IDLE;
}


bool get_register_by_twi(uint8_t twi_addr, uint8_t reg_addr, uint8_t* buf, uint8_t len, gsil_drv_twi_done_handler_t done_handler)
{
    ret_code_t err_code;
 
    if( m_twi_status != TWI_IDLE ) { 
      printf(" Status is Not Idle.... ");
      return false; 
    }
    m_twi_addr = twi_addr;
    m_recv_buf = buf;
    m_recv_len = len;
    m_recv_done_handler = done_handler;

    m_twi_status = TWI_TX;
    err_code = nrf_drv_twi_tx(&m_twi, twi_addr, &reg_addr, 1, true); //3th =/= pointer to a transmit buffer??
    APP_ERROR_CHECK(err_code);

    return true;
}
static uint8_t b0_cmd_pwr[2] = {0x06, 0x61};
static uint8_t b0_cmd_int_cfg[2] = {0x0F, 0x20};
static uint8_t b0_cmd_int_en[2] = {0x10, 0x08};
static uint8_t b2_cmd_wom_en[2] = {0x12, 0x02};
static uint8_t cmd_usr0[2] = {0x7F, 0x00};
static uint8_t cmd_usr2[2] = {0x7F, 0x20};



bool send_data_by_twi(uint8_t twi_addr, uint8_t* send_data, uint8_t len)
{
    ret_code_t err_code;

    if( m_twi_status != TWI_IDLE ) { return false; }
    m_twi_status = TWI_JUST_TX;
    err_code = nrf_drv_twi_tx(&m_twi, twi_addr, send_data, len, false);
    APP_ERROR_CHECK(err_code);
}


void print_buf(void *recv_data, uint8_t len)
{
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
}

static int16_t m_acc_data[3];
void print_acc(void *recv_data, uint8_t len)
{
    uint8_t i;

    printf("\n > %d : ", len);
    if( len != 6 ) { 
      printf("\n print_acc Nothing! ");
      return; 
    }
   
    for(i=0; i< 3; i++ ) 
    {
        printf(" %d", ((int16_t*)(recv_data))[i]);
        m_acc_data[i] = ((int16_t*)(recv_data))[i];
    }
}

int16_t* gsil_get_LPS_data()
{
  
}


int16_t* gsil_get_acc_data()
{
    //lis2dh12tr_print_info();
    //lis2dh12tr_print_data();
    return m_acc_data;
}
bool is_twi_ready(void)
{
    return m_twi_status == TWI_IDLE?true:false;
}

void lis2dh12tr_print_who_am_i()
{
    printf("\nLIS2DH12TR WHO AM  I test(33 is expected)");
    if(!get_register_by_twi(LIS2DH12TR_ADDR, 0x0F, test_buf,1,print_buf)) 
    {
      printf("\nlis2dh12tr print who am i failed!");
    }
}



void lis2dh12tr_print_info()
{
    printf("\ninfo");
    get_register_by_twi(LIS2DH12TR_ADDR, 0x80 | 0x1E, test_buf,10,print_buf);
}

void lis2dh12tr_print_data()
{
    printf("\ndata ! ");
    bool resb = get_register_by_twi(LIS2DH12TR_ADDR, 0x80 | 0x28, test_buf,6,print_acc);
    //printf("\n get_register_by_twi returnd %d",resb);
}

static uint8_t ctrl_reg1_1hz[2] = {0x20, 0x10 | 0x0F  };
static uint8_t ctrl_reg1_10hz[2] = {0x20, 0x20 | 0x0F  };

void lis2dh12tr_set_config()
{
  send_data_by_twi(LIS2DH12TR_ADDR, ctrl_reg1_10hz, 2);   while(!is_twi_ready());
}

void lps22hb_print_who_am_i()
{
    printf("\nLPS22HB WHO AM  I test(B1 is expected)");
    get_register_by_twi(LPS22HB_ADDR, 0x0F, test_buf,1,print_buf);
}

void hdc2010_print_who_am_i()
{
    printf("\nHDC2010 WHO AM  I test(D0 07 are expected)");
    get_register_by_twi(HDC2010_ADDR, 0xFE, test_buf,2,print_buf);
}

void hts221_print_who_am_i()
{
    printf("\nHTS221 WHO AM  I test(?? is expected)");
    get_register_by_twi(HTS221_ADDR, 0x0F, test_buf,1,print_buf);
}

void icm20948_print_who_am_i()
{
    printf("\nICM20948 WHO AM  I test(EA is expected)");
    get_register_by_twi(ICM20948_ADDR, 0x00, test_buf,1,print_buf);
}

void icm20948_set_wakeup_on_motion()
{
    printf("\nRead");
    send_data_by_twi(ICM20948_ADDR, cmd_usr0, 2); while(!is_twi_ready());
    get_register_by_twi(ICM20948_ADDR, 6, test_buf, 11, print_buf);
    while(!is_twi_ready());

    printf("\nSet wakeup motion\n");



    
    send_data_by_twi(ICM20948_ADDR, b0_cmd_pwr, 2);   while(!is_twi_ready());
    send_data_by_twi(ICM20948_ADDR, b0_cmd_int_cfg, 2);   while(!is_twi_ready());
    send_data_by_twi(ICM20948_ADDR, b0_cmd_int_en, 2);   while(!is_twi_ready());

    send_data_by_twi(ICM20948_ADDR, cmd_usr2, 2); while(!is_twi_ready());
    send_data_by_twi(ICM20948_ADDR, b2_cmd_wom_en, 2);  while(!is_twi_ready());
    send_data_by_twi(ICM20948_ADDR, cmd_usr0, 2); while(!is_twi_ready());

    
    
    printf("\nRead");
    get_register_by_twi(ICM20948_ADDR, 6, test_buf, 11, print_buf);
    while(!is_twi_ready());

    for(uint8_t i=0; i<10;i++) 
    {
      printf("\nStatus");
      get_register_by_twi(ICM20948_ADDR, 25, test_buf, 1, print_buf);
      nrf_delay_ms(1000);
    }

}