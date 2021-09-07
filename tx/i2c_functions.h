#include <stdbool.h>
#include <stdint.h>
#include "hw_ints.h"
#include "hw_memmap.h"
#include "gpio.h"
#include "interrupt.h"
#include "ioc.h"
#include "hw_ioc.h"
#include "sys_ctrl.h"
#include "hw_i2cm.h"
#include "hw_i2cs.h"
#include "i2c.h"
#include "hal_timer_32k.h"
#include "sys_ctrl.h"


char bus_read(unsigned char device_addr, unsigned char register_addr, unsigned char* register_data, unsigned char read_length, unsigned char reg_present);

char bus_write(unsigned char device_addr, unsigned char register_addr, unsigned char* register_data, unsigned char write_length, unsigned char reg_present);

void delay_msec (unsigned int delay);
