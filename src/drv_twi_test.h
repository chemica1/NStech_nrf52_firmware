#ifndef GSIL_DRV_TWI_H__
#define GSIL_DRV_TWI_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define LIS2DH12TR_ADDR       (0x32U >> 1) // accelerometer
#define LPS22HB_ADDR          (0xB8U >> 1) // presure sensor
#define HDC2010_ADDR          (0x80U >> 1) // Humidity and Temperature sonsor
#define ICM20948_ADDR         (0xD0U >> 1) // motion sensor
#define HTS221_ADDR           (0xBEU >> 1) // not existed on the G542 board
//#define TES0901_ADDR  ?? no datasheet -_-
//#define CCS811_ADDR ?? not mounted on the G542 board

void gsil_drv_twi_init (void);
bool is_twi_ready(void);

typedef void (* gsil_drv_twi_done_handler_t)(void *recv_data,uint8_t len);

bool get_register_by_twi(uint8_t twi_addr, uint8_t reg_addr, uint8_t* buf, uint8_t len, gsil_drv_twi_done_handler_t done_handler);

void lis2dh12tr_print_who_am_i();
void lis2dh12tr_print_data();
void lis2dh12tr_print_info();
void lis2dh12tr_set_config();

int16_t* gsil_get_acc_data();

void lps22hb_print_who_am_i();
void hdc2010_print_who_am_i();
void hts221_print_who_am_i();
void icm20948_print_who_am_i();


void icm20948_set_wakeup_on_motion();
#endif /* GSIL_DRV_TWI_H__ */