#ifndef GSIL_DRV_TWI_H__
#define GSIL_DRV_TWI_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


/*
////////LPS22HBªÎàâïÒ////////
#define CTRL_REG1_1HZ (0x10)//«Ç?«¿õóÕô«â?«É(1Hz)
#define ctrl_reg1_10hz (0x20)//«Ç?«¿õóÕô«â?«É(10Hz)
#define CTRL_REG1_25HZ 0x30//«Ç?«¿õóÕô«â?«É(35Hz)
#define CTRL_REG1_50HZ 0x40//«Ç?«¿õóÕô«â?«É(50Hz)
#define CTRL_REG1_75HZ 0x50//«Ç?«¿õóÕô«â?«É(75Hz)//akiracing.com
#define CTRL_REG1_LPF_9_1HZ 0x18//«Ç?«¿õóÕô«â?«É(1Hz)//LPF, Freq/9
#define CTRL_REG1_LPF_9_10HZ 0x28//«Ç?«¿õóÕô«â?«É(10Hz)//LPF, Freq/9
#define CTRL_REG1_LPF_9_25HZ 0x38//«Ç?«¿õóÕô«â?«É(35Hz)//LPF, Freq/9
#define CTRL_REG1_LPF_9_50HZ 0x48//«Ç?«¿õóÕô«â?«É(50Hz)//LPF, Freq/9
#define CTRL_REG1_LPF_9_75HZ 0x58//«Ç?«¿õóÕô«â?«É(75Hz)//LPF, Freq/9
#define CTRL_REG1_LPF_20_1HZ 0x1C//«Ç?«¿õóÕô«â?«É(1Hz)//LPF, Freq/20
#define CTRL_REG1_LPF_20_10HZ 0x2C//«Ç?«¿õóÕô«â?«É(10Hz)//LPF, Freq/20
#define CTRL_REG1_LPF_20_25HZ 0x3C//«Ç?«¿õóÕô«â?«É(35Hz)//LPF, Freq/20
#define CTRL_REG1_LPF_20_50HZ 0x4C//«Ç?«¿õóÕô«â?«É(50Hz)//LPF, Freq/20
#define CTRL_REG1_LPF_20_75HZ 0x5C8//«Ç?«¿õóÕô«â?«É(75Hz)//LPF, Freq/20
*/

#define LIS2DH12TR_ADDR       (0x32U >> 1) // accelerometer
#define LPS22HB_ADDR          (0xB8U >> 1) // presure sensor  //1byte shift -> 0x5D
#define HDC2010_ADDR          (0x80U >> 1) // Humidity and Temperature sonsor
#define ICM20948_ADDR         (0xD0U >> 1) // motion sensor
#define HTS221_ADDR           (0xBEU >> 1) // not existed on the G542 board
//#define TES0901_ADDR  ?? no datasheet -_-
//#define CCS811_ADDR ?? not mounted on the G542 board


/* LPS */
#define LPS22HB_WHO_AM_I	0X0F //Who am I
#define LPS22HB_RES_CONF	0X1A //Resolution //resolution??
#define LPS22HB_CTRL_REG1	0X10
#define LPS22HB_CTRL_REG2	0X11
#define LPS22HB_STATUS_REG	0X27
#define LPS22HB_PRES_OUT_XL	0X28 //LSB
#define LPS22HB_PRES_OUT_L	0X29
#define LPS22HB_PRES_OUT_H	0X2A //MSB
#define LPS22HB_TEMP_OUT_L	0X2B //LSB
#define LPS22HB_TEMP_OUT_H	0X2C //MSB
#define LPS22HB_WHO_AM_I_VALUE	0xB1 // Expected return value of WHO_AM_I register


void gsil_drv_twi_init (void);
void gsil_drv_twi_uninit (void);
void gsil_drv_twi_disable(void);
bool is_twi_ready_1(void);
bool is_twi_ready_2(void);


typedef void (* gsil_drv_twi_done_handler_t)(void *recv_data,uint8_t len); // how it work??

bool get_register_by_twi_1(uint8_t twi_addr, uint8_t reg_addr, uint8_t* buf, uint8_t len, gsil_drv_twi_done_handler_t done_handler);
bool get_register_by_twi_2(uint8_t twi_addr, uint8_t reg_addr, uint8_t* buf, uint8_t len, gsil_drv_twi_done_handler_t done_handler);


void lis2dh12tr_print_who_am_i();
void lis2dh12tr_print_data();
void lis2dh12tr_print_info();
void lis2dh12tr_set_config();


void LPS22HB_print_who_am_i();
void LPS22HB_set_config();
void LPS22HB_print_data();



void hdc2010_print_who_am_i();
void hts221_print_who_am_i();
void icm20948_print_who_am_i();


void icm20948_set_wakeup_on_motion();
#endif /* GSIL_DRV_TWI_H__ */