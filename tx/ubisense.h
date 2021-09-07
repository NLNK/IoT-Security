/*  
 * Date: 12-Aug-2015
 * Revision: 3.0
 *
 * Copyright (C) 2015 Centre for Development of Advanced Computing (DCBA), Bengaluru.
 *
 * UbiSense API
 *
 * Usage:  Application Programming Interface for UbiSense configuration and data read out
 *
 * Author: Thajudheen K <thajudheenk@DCBA.in>   
 *
 */

#include <math.h>
#include <stdint.h>

#ifndef __UBISENSE_H__
#define __UBISENSE_H__

#define UBI_SUCCESS 1
#define UBI_FAIL 0
#define UBI_YES 1
#define UBI_NO 0

/* Define for used read and write macros
 * Define the calling convention of YOUR bus communication routine.
 * note This includes types of parameters. This example shows the
 * configuration for an SPI bus link.*/

/* defines the return parameter type of the BMP180_WR_FUNCTION */
#define I2C_BUS_WR_RETURN_TYPE char

/* defines the calling parameter types of the BMP180_WR_FUNCTION */
#define I2C_BUS_WR_PARAM_TYPES unsigned char, unsigned char,\
	unsigned char *, unsigned char, unsigned char

/* links the order of parameters defined in BMP180_BUS_WR_PARAM_TYPE
 * to function calls used inside the API */
#define I2C_BUS_WR_PARAM_ORDER (device_addr, register_addr,\
		register_data, write_length, reg_present)

/* never change this line  */
#define I2C_BUS_WRITE_FUNC(device_addr, register_addr,\
		register_data, write_length, reg_present)\
bus_write(device_addr, register_addr, register_data, write_length, reg_present)

/* defines the return parameter type of the BMP180_WR_FUNCTION */
#define I2C_BUS_RD_RETURN_TYPE char

/* defines the calling parameter types of the BMP180_WR_FUNCTION */
#define I2C_BUS_RD_PARAM_TYPES unsigned char, unsigned char,\
	unsigned char *, unsigned char, unsigned char

/* links the order of parameters defined in BMP180_BUS_WR_PARAM_TYPE
 * to function calls used inside the API*/
#define I2C_BUS_RD_PARAM_ORDER (device_addr, register_addr,\
		register_data, read_length, reg_present)

/* never change this line */
#define I2C_BUS_READ_FUNC(device_addr, register_addr,\
		register_data, read_length, reg_present)\
bus_read(device_addr, register_addr, register_data, read_length, reg_present)

/* register write and read delays */

#define I2C_MDELAY_DATA_TYPE unsigned int
#define I2C_MDELAY_RETURN_TYPE  void

#endif   /* __UBISENSE_H__ */

/***********************************************************************************************************************
 *************************************  SHT21 - Temperature & Relative Humidity  ***************************************
 ***********************************************************************************************************************/
#ifndef __SHT21_H__
#define __SHT21_H__

#define SHT21_I2C_ADDR                	0x40

#define SHT21_TRIG_TEMP_HOLD		0xE3
#define SHT21_TRIG_HUMI_HOLD		0xE5
#define SHT21_TRIG_TEMP_NOHOLD		0xF3
#define SHT21_TRIG_HUMI_NOHOLD		0xF5

#define SHT21_WRITE_USER_REG		0xE6
#define SHT21_READ_USER_REG		0xE7
#define SHT21_SOFT_RESET		0xFE

struct sht21_t {
	unsigned char dev_addr;
	unsigned char resolution;

	I2C_BUS_WR_RETURN_TYPE(*bus_write)(I2C_BUS_WR_PARAM_TYPES);
	I2C_BUS_RD_RETURN_TYPE(*bus_read)(I2C_BUS_RD_PARAM_TYPES);
	I2C_MDELAY_RETURN_TYPE(*delay_msec)(I2C_MDELAY_DATA_TYPE);
};

int sht21_init(struct sht21_t *);

uint16_t sht21_get_raw_humidity(void);
uint16_t sht21_get_raw_temperature(void);

float sht21_get_caliberated_humidity(uint16_t );
float sht21_get_caliberated_temperature(uint16_t );

#endif   /* __SHT21_H__ */

/***********************************************************************************************************************
 *****************************************   TSL26711 - Proximity Detector  ********************************************
 ***********************************************************************************************************************/
#ifndef __TSL26711_H__
#define __TSL26711_H__

#define TSL26711_I2C_ADDR              	0x39

#define	TSL26711_REG_ENABLE	0x00
#define	TSL26711_REG_PTIME	0x02
#define	TSL26711_REG_WTIME	0x03
#define	TSL26711_REG_PILTL	0x08
#define	TSL26711_REG_PILTH	0x09
#define	TSL26711_REG_PIHTL	0x0A
#define	TSL26711_REG_PIHTH	0x0B
#define	TSL26711_REG_PERS	0x0C
#define	TSL26711_REG_CONFIG	0x0D
#define	TSL26711_REG_PPCOUNT	0x0E
#define	TSL26711_REG_CONTROL	0x0F
#define	TSL26711_REG_ID		0x12
#define	TSL26711_REG_STATUS	0x13
#define	TSL26711_REG_PDATAL	0x18
#define	TSL26711_REG_PDATAH	0x19

#define TSL26711_COMMAND		0x80
#define TSL26711_TYPE_AUTOINC		0x20
#define TSL26711_PROX_INT_CLEAR		0x05

#define TSL26711_ENABLE_PIEN 		0x20
#define TSL26711_ENABLE_WEN 		0x08
#define TSL26711_ENABLE_PEN 		0x06
#define TSL26711_ENABLE_PON 		0x01

#define TSL26711_PTIMER_272		0xFF

#define TSL26711_WTIMER_1		0xFF
#define TSL26711_WTIMER_74		0xB6
#define TSL26711_WTIMER_256		0x00

/* Interrupt registers not added */

#define TSL26711_CONFIG_WLONG		0x02

#define TSL26711_PPCOUNT_PPULSE		0x0A /* 10 Pulses */

#define TSL26711_CTRL_PDRIVE_100	0x00 /* LED Strength 100mA */
#define TSL26711_CTRL_PDRIVE_50		0x40 /* LED Strength 50mA */
#define TSL26711_CTRL_PDRIVE_25		0x80 /* LED Strength 25mA */
#define TSL26711_CTRL_PDRIVE_125	0xC0 /* LED Strength 12.5mA */
 
#define TSL26711_CTRL_PDIODE_CH0        0x10 /* Diode Channel 0 */
#define TSL26711_CTRL_PDIODE_CH1        0x20 /* Diode Channel 1 */
#define TSL26711_CTRL_PDIODE_BOTH       0x30 /* Both Diode */

#define TSL26711_ID			0x00 /* 0x00 = TSL26711 and TSL26715 and 0x09 = TSL26713 and TSL26717 */

struct tsl26711_t {
        unsigned char dev_addr;
        unsigned char pulses;

        I2C_BUS_WR_RETURN_TYPE(*bus_write)(I2C_BUS_WR_PARAM_TYPES);
        I2C_BUS_RD_RETURN_TYPE(*bus_read)(I2C_BUS_RD_PARAM_TYPES);
        I2C_MDELAY_RETURN_TYPE(*delay_msec)(I2C_MDELAY_DATA_TYPE);
};

int tsl26711_init(struct tsl26711_t *);

uint16_t tsl26711_get_proximity(void);

#endif   /* __TSL26711_H__ */

/***********************************************************************************************************************
 ****************************   BMP180 - Barometric Pressure & Temperature Sensor  *************************************
 ***********************************************************************************************************************/

#ifndef __BMP180_H__
#define __BMP180_H__

/*        CHIP_TYPE CONSTANTS */
#define BMP180_CHIP_ID                      0x55
#define BOSCH_PRESSURE_BMP180   85

/*        BMP180 I2C Address */
#define BMP180_I2C_ADDR         (0xEE>>1)

/*        SMB380 API error codes */
#define E_BMP_NULL_PTR                      (char)  (-127)
#define E_BMP_COMM_RES                     (char) (-1)
#define E_BMP_OUT_OF_RANGE              (char) (-2)
#define E_SENSOR_UBI_NOT_DETECTED        (char) 0

/*     register definitions     */
#define BMP180_PROM_START__ADDR         0xaa
#define BMP180_PROM_DATA__LEN             22

#define BMP180_CHIP_ID_REG                      0xD0
#define BMP180_VERSION_REG                      0xD1

#define BMP180_CTRL_MEAS_REG            0xF4
#define BMP180_ADC_OUT_MSB_REG          0xF6
#define BMP180_ADC_OUT_LSB_REG          0xF7

#define BMP180_SOFT_RESET_REG           0xE0

#define BMP180_T_MEASURE        0x2E  /* temperature measurent */
#define BMP180_P_MEASURE        0x34  /* pressure measurement */

#define BMP180_TEMP_CONVERSION_TIME  5 /* TO be spec'd by GL or SB */

#define PARAM_MG      3038        /*calibration parameter */
#define PARAM_MH     -7357        /*calibration parameter */
#define PARAM_MI      3791        /*calibration parameter */

/* this structure holds all device specific calibration parameters */
struct bmp180_calibration_param_t{
	short ac1;
	short ac2;
	short ac3;
	unsigned short ac4;
	unsigned short ac5;
	unsigned short ac6;
	short b1;
	short b2;
	short mb;
	short mc;
	short md;
};
/* BMP180 image registers data structure */
struct bmp180_t {
	struct bmp180_calibration_param_t cal_param;
	unsigned char mode;
	unsigned char chip_id, ml_version, al_version;
	unsigned char dev_addr;
	unsigned char sensortype;

	long param_b5;
	int number_of_samples;
	short oversampling_setting;
	short sw_oss;
	I2C_BUS_WR_RETURN_TYPE(*bus_write)(I2C_BUS_WR_PARAM_TYPES);
	I2C_BUS_RD_RETURN_TYPE(*bus_read)(I2C_BUS_RD_PARAM_TYPES);
	I2C_MDELAY_RETURN_TYPE(*delay_msec)(I2C_MDELAY_DATA_TYPE);
};
/* bit slice positions in registers */

#define BMP180_CHIP_ID__POS             0
#define BMP180_CHIP_ID__MSK             0xFF
#define BMP180_CHIP_ID__LEN             8
#define BMP180_CHIP_ID__REG             BMP180_CHIP_ID_REG

#define BMP180_ML_VERSION__POS          0
#define BMP180_ML_VERSION__LEN          4
#define BMP180_ML_VERSION__MSK          0x0F
#define BMP180_ML_VERSION__REG          BMP180_VERSION_REG

#define BMP180_AL_VERSION__POS          4
#define BMP180_AL_VERSION__LEN          4
#define BMP180_AL_VERSION__MSK          0xF0
#define BMP180_AL_VERSION__REG          BMP180_VERSION_REG

/* DATA REGISTERS
 * LG/HG thresholds are in LSB and depend on RANGE setting
 * no range check on threshold calculation*/
#define BMP180_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMP180_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))
/* General Setup Functions */
/* BMP180_init */

/*   input :      Pointer to bmp180_t
 *   output:  -
 *   return:  result of communication function
 *   notes : */

int bmp180_init(struct bmp180_t *);

long bmp180_get_temperature(unsigned long ut);

long bmp180_get_pressure(unsigned long up);

unsigned long bmp180_get_ut(void);
unsigned long  bmp180_get_up(void);

int bmp180_get_cal_param(void);

#endif   /* __BMP180_H__ */

/***********************************************************************************************************************
 ****************************************   TSL2561 - Ambient Light Sensor  ********************************************
 ***********************************************************************************************************************/
#ifndef __TSL2561_H__
#define __TSL2561_H__

#define	TSL2561_I2C_ADDR		0x29

/* Registers*/
#define	TSL2561_REG_CONTROL		0x00
#define	TSL2561_REG_TIMING		0x01
#define	TSL2561_REG_THRESHLOWLOW	0x02
#define	TSL2561_REG_THRESHLOWHIGH	0x03
#define	TSL2561_REG_THRESHHIGHLOW	0x04
#define	TSL2561_REG_THRESHHIGHHIGH	0x05
#define	TSL2561_REG_INTERRUPT		0x06
#define	TSL2561_REG_CRC			0x08
#define	TSL2561_REG_ID			0x0A
#define	TSL2561_REG_DATA0LOW		0x0C
#define	TSL2561_REG_DATA0HIGH		0x0D
#define	TSL2561_REG_DATA1LOW		0x0E
#define	TSL2561_REG_DATA1HIGH		0x0F

/* Commands */
#define TSL2561_COMMAND_CMD 	0x80
#define TSL2561_COMMAND_CLEAR 	0x40
#define TSL2561_COMMAND_WORD	0x20
#define TSL2561_COMMAND_BLOCK	0x10

#define TSL2561_POWERON 0x03
#define TSL2561_POWEROFF 0x00

#define TSL2561_INTEGRATIONTIME_13MS 0x00    // 13.7ms
#define TSL2561_INTEGRATIONTIME_101MS 0x01    // 101ms
#define TSL2561_INTEGRATIONTIME_402MS 0x02     // 402ms

#define TSL2561_LUX_LUXSCALE      (14)      // Scale by 2^14
#define TSL2561_LUX_RATIOSCALE    (9)       // Scale ratio by 2^9
#define TSL2561_LUX_CHSCALE       (10)      // Scale channel values by 2^10
#define TSL2561_LUX_CHSCALE_TINT0 (0x7517)  // 322/11 * 2^TSL2561_LUX_CHSCALE
#define TSL2561_LUX_CHSCALE_TINT1 (0x0FE7)  // 322/81 * 2^TSL2561_LUX_CHSCALE

#define TSL2561_GAIN_0X		(0x00)    // No gain
#define TSL2561_GAIN_16X	(0x10)    // 16x gain

// T, FN and CL package values
#define TSL2561_LUX_K1T           (0x0040)  // 0.125 * 2^RATIO_SCALE
#define TSL2561_LUX_B1T           (0x01f2)  // 0.0304 * 2^LUX_SCALE
#define TSL2561_LUX_M1T           (0x01be)  // 0.0272 * 2^LUX_SCALE
#define TSL2561_LUX_K2T           (0x0080)  // 0.250 * 2^RATIO_SCALE
#define TSL2561_LUX_B2T           (0x0214)  // 0.0325 * 2^LUX_SCALE
#define TSL2561_LUX_M2T           (0x02d1)  // 0.0440 * 2^LUX_SCALE
#define TSL2561_LUX_K3T           (0x00c0)  // 0.375 * 2^RATIO_SCALE
#define TSL2561_LUX_B3T           (0x023f)  // 0.0351 * 2^LUX_SCALE
#define TSL2561_LUX_M3T           (0x037b)  // 0.0544 * 2^LUX_SCALE
#define TSL2561_LUX_K4T           (0x0100)  // 0.50 * 2^RATIO_SCALE
#define TSL2561_LUX_B4T           (0x0270)  // 0.0381 * 2^LUX_SCALE
#define TSL2561_LUX_M4T           (0x03fe)  // 0.0624 * 2^LUX_SCALE
#define TSL2561_LUX_K5T           (0x0138)  // 0.61 * 2^RATIO_SCALE
#define TSL2561_LUX_B5T           (0x016f)  // 0.0224 * 2^LUX_SCALE
#define TSL2561_LUX_M5T           (0x01fc)  // 0.0310 * 2^LUX_SCALE
#define TSL2561_LUX_K6T           (0x019a)  // 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6T           (0x00d2)  // 0.0128 * 2^LUX_SCALE
#define TSL2561_LUX_M6T           (0x00fb)  // 0.0153 * 2^LUX_SCALE
#define TSL2561_LUX_K7T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7T           (0x0018)  // 0.00146 * 2^LUX_SCALE
#define TSL2561_LUX_M7T           (0x0012)  // 0.00112 * 2^LUX_SCALE
#define TSL2561_LUX_K8T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8T           (0x0000)  // 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8T           (0x0000)  // 0.000 * 2^LUX_SCALE

/* Interrupt related needs to be updated */

struct tsl2561_t {
	unsigned char chip_id, partno, revno;
	unsigned char dev_addr;
	unsigned char gain;
	unsigned char integration;

	I2C_BUS_WR_RETURN_TYPE(*bus_write)(I2C_BUS_WR_PARAM_TYPES);
	I2C_BUS_RD_RETURN_TYPE(*bus_read)(I2C_BUS_RD_PARAM_TYPES);
	I2C_MDELAY_RETURN_TYPE(*delay_msec)(I2C_MDELAY_DATA_TYPE);
};

int tsl2561_init(struct tsl2561_t *);

int tsl2561_get_raw_light_intensity(uint8_t *);

uint32_t tsl2561_get_lux_light_intensity(uint8_t *);

#endif   /* __TSL2561_H__ */

/* End of file */
