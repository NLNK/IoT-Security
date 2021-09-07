/*
 *
 */

#include "ubisense.h"

/***********************************************************************************************************************
 *************************************  SHT21 - Temperature & Relative Humidity  ***************************************
 ***********************************************************************************************************************/

struct sht21_t *p_sht21;

/*
Function initailises sht21 chip. 
Arguments -- struct sht21_t. Fill the I2C read, I2C write and delay funtion pointers in the structure before passing into this function.

Returns 1 upon successful initialisation.
*/
int sht21_init(struct sht21_t *sht21_t)
{
	uint8_t command,ret;
        p_sht21 = sht21_t;
        p_sht21->dev_addr = SHT21_I2C_ADDR;
        command = SHT21_SOFT_RESET;	
        ret = p_sht21->I2C_BUS_WRITE_FUNC(p_sht21->dev_addr, 0, &command, 1, UBI_NO); /* Enable the chip */
	if(ret) {}
		
	return UBI_SUCCESS;
}

/*
Function reads raw humidity from sht21 chip. 

Arguments -- None.

Returns raw humidity(uint16_t) upon successful completion.
*/
uint16_t sht21_get_raw_humidity(void)
{
	uint8_t raw_data[2],command,ret;
	uint16_t raw_humi;
	
	command = SHT21_SOFT_RESET;
        ret = p_sht21->I2C_BUS_WRITE_FUNC(p_sht21->dev_addr, 0, &command, 1, UBI_NO); /* Enable the chip */
        p_sht21->delay_msec(15);

        command = SHT21_TRIG_HUMI_NOHOLD;
	ret = p_sht21->I2C_BUS_WRITE_FUNC(p_sht21->dev_addr, 0, &command, 1, UBI_NO);
        p_sht21->delay_msec(100);
        ret = p_sht21->I2C_BUS_READ_FUNC(p_sht21->dev_addr, 0, raw_data, 2, UBI_NO); /* Read raw humidity */
	
	raw_humi = raw_data[0];
	raw_humi = ((raw_humi << 8) | (raw_data[1] & 0x0F));
	if(ret) {}


	return raw_humi;
}

/*
Function reads raw temperature from sht21 chip. 
Arguments -- None.

Returns raw temperature(uint16_t) upon successful completion.
*/
uint16_t sht21_get_raw_temperature(void)
{
	uint8_t raw_data[2],command,ret;
        uint16_t raw_temp;
	
	command = SHT21_SOFT_RESET;
        ret = p_sht21->I2C_BUS_WRITE_FUNC(p_sht21->dev_addr, 0, &command, 1, UBI_NO); /* Enable the chip */
        p_sht21->delay_msec(15);

        command = SHT21_TRIG_TEMP_NOHOLD;    
        ret = p_sht21->I2C_BUS_WRITE_FUNC(p_sht21->dev_addr, 0, &command, 1, UBI_NO);
        p_sht21->delay_msec(100);
        ret = p_sht21->I2C_BUS_READ_FUNC(p_sht21->dev_addr, 0, raw_data, 2, UBI_NO); /* Read raw temperature */

	raw_temp = raw_data[0];
        raw_temp = ((raw_temp << 8) | (raw_data[1] & 0x03));
	if(ret) {}

        return raw_temp;

}

/*
Function converts raw humidity into humidity in percentage(%). 

Arguments -- uint16_t raw_data. Calculations will be based on the value in raw_data. Expects the output of function sht21_get_raw_humidity() as the input to this funtion.

Returns humidity in percentage(%)(float) upon successful completion.

NOTE: THIS FUNTION USES FLOAT OPERATIONS. IF YOUR MICROCONTROLLER/COMPILER DOESNOT SUPPORT FLOAT OPERATIONS PLEASE DONT CALL THIS FUNTION. INSTEAD WORK ON RAW DATA.
*/
float sht21_get_caliberated_humidity(uint16_t raw_data)
{
	return (-6.0 + 125.0 / 65536.0 * (float)(raw_data));
}

/*
Function converts raw temperature into temperature in deg celsius. 

Arguments -- uint16_t raw_data. Calculations will be based on the value in raw_data. Expects the output of function sht21_get_raw_temperature() as the input to this funtion.

Returns temperature in deg celsius(float) upon successful completion.

NOTE: THIS FUNTION USES FLOAT OPERATIONS. IF YOUR MICROCONTROLLER/COMPILER DOESNOT SUPPORT FLOAT OPERATIONS PLEASE DONT CALL THIS FUNTION. INSTEAD WORK ON RAW DATA.

*/
float sht21_get_caliberated_temperature(uint16_t raw_data)
{
	return (-46.85 + 175.72 / 65536.0 * (float)(raw_data));
}

/***********************************************************************************************************************
 *****************************************   TSL26711 - Proximity Detector  ********************************************
 ***********************************************************************************************************************/

struct tsl26711_t *p_tsl26711;

/*

Function initailises tsl26711 chip. 
Arguments -- struct tsl26711_t. Fill the I2C read, I2C write and delay funtion pointers in the structure before passing into this function.

Returns 1 upon successful initialisation.
*/
int tsl26711_init(struct tsl26711_t *tsl26711_t)
{
	uint8_t data=0,command,ret;
        p_tsl26711 = tsl26711_t;
        p_tsl26711->dev_addr = TSL26711_I2C_ADDR;
        p_tsl26711->pulses = TSL26711_PPCOUNT_PPULSE; /* set to 10 pulses now. Comment this line if you are setting manually*/
        
	command = TSL26711_COMMAND + TSL26711_REG_PTIME;
        data = TSL26711_PTIMER_272;
        ret = p_tsl26711->I2C_BUS_WRITE_FUNC(p_tsl26711->dev_addr, command, &data, 1, UBI_YES); /* Set Proximity timer to 2.72ms */

	command = TSL26711_COMMAND + TSL26711_REG_PPCOUNT;
        data = p_tsl26711->pulses;
        ret = p_tsl26711->I2C_BUS_WRITE_FUNC(p_tsl26711->dev_addr, command, &data, 1, UBI_YES); /*  proximity pulses transmitted */

	command = TSL26711_COMMAND + TSL26711_REG_CONTROL;
        data = TSL26711_CTRL_PDRIVE_100 + TSL26711_CTRL_PDIODE_BOTH;
        ret = p_tsl26711->I2C_BUS_WRITE_FUNC(p_tsl26711->dev_addr, command, &data, 1, UBI_YES); /* 100mA with both diodes */

	command = TSL26711_COMMAND + TSL26711_REG_ENABLE;
        data = TSL26711_ENABLE_PON;
        ret = p_tsl26711->I2C_BUS_WRITE_FUNC(p_tsl26711->dev_addr, command, &data, 1, UBI_YES); /* Power ON */
	p_tsl26711->delay_msec(5);

	command = TSL26711_COMMAND + TSL26711_REG_ID;
        data = 0xFF;
        ret = p_tsl26711->I2C_BUS_READ_FUNC(p_tsl26711->dev_addr, command, &data, 1, UBI_YES); /* Reading ID */
	if(data != TSL26711_ID)
	{
		return UBI_FAIL;		
	}

	command = TSL26711_COMMAND + TSL26711_REG_ENABLE;
        data = TSL26711_ENABLE_PEN + TSL26711_ENABLE_PON;
        ret = p_tsl26711->I2C_BUS_WRITE_FUNC(p_tsl26711->dev_addr, command, &data, 1, UBI_YES); /* Proximity ON */
	if(ret) {}

	return UBI_SUCCESS;
}

/*
Function get proximity value from tsl26711 chip. 
Arguments -- None.

Returns proximity value(uint16_t) upon successful initialisation.

NOTE: Proximity value ranges from 0 to 1024.
*/
uint16_t tsl26711_get_proximity(void)
{
	uint8_t data[2],command,ret;
	uint16_t proximity = 0;
	command = TSL26711_COMMAND + TSL26711_TYPE_AUTOINC + TSL26711_REG_PDATAL;
        ret = p_tsl26711->I2C_BUS_READ_FUNC(p_tsl26711->dev_addr, command, data, 2, UBI_YES); /* Reading ID */
	
	proximity = data [1];
	proximity = ((proximity << 8) | data[0]);
	if(ret) {}

	return proximity;
}

/***********************************************************************************************************************
 ****************************   BMP180 - Barometric Pressure & Temperature Sensor  *************************************
 ***********************************************************************************************************************/

struct bmp180_t *p_bmp180; 

/* 
Initialize BMP180. This function initializes the BMP180 pressure sensor. The function automatically detects the sensor type and storesthis for all future communication and calculation steps param *bmp180_t pointer to bmp180 device data structure.
Arguments -- struct bmp180_t. Fill the I2C read, I2C write and delay funtion pointers in the structure before passing into this function.

Returns result of communication routines 
*/
int bmp180_init(struct bmp180_t *bmp180)
{
	char comres = 0;
	unsigned char data;

	p_bmp180 = bmp180;         
	p_bmp180->sensortype = E_SENSOR_UBI_NOT_DETECTED;
	p_bmp180->dev_addr = BMP180_I2C_ADDR;   /* preset BMP180 I2C_addr */

	comres += p_bmp180->I2C_BUS_READ_FUNC(p_bmp180->dev_addr, BMP180_CHIP_ID__REG, &data, 1, UBI_YES);  /* read Chip Id */

	p_bmp180->chip_id = BMP180_GET_BITSLICE(data, BMP180_CHIP_ID);
	p_bmp180->number_of_samples = 1;
	p_bmp180->oversampling_setting = 0;
	p_bmp180->sw_oss = 0;
	if (p_bmp180->chip_id == BMP180_CHIP_ID) {
		/* get bitslice */
		p_bmp180->sensortype = BOSCH_PRESSURE_BMP180;
		/* read Version reg */
		comres += p_bmp180->I2C_BUS_READ_FUNC(p_bmp180->dev_addr, BMP180_VERSION_REG, &data, 1, UBI_YES);


		/* get ML Version */
		p_bmp180->ml_version = BMP180_GET_BITSLICE(data, BMP180_ML_VERSION);
		/* get AL Version */
		p_bmp180->al_version = BMP180_GET_BITSLICE(data, BMP180_AL_VERSION);
		bmp180_get_cal_param(); /*readout bmp180 calibparam structure*/
	}
	return comres;
}

/** 
Reads out parameters cal_param from BMP180 memory
Arguments -- None.

Returns result of communication routines
*/
int bmp180_get_cal_param(void)
{
	int comres;
	unsigned char data[22];
	comres = p_bmp180->I2C_BUS_READ_FUNC(p_bmp180->dev_addr, BMP180_PROM_START__ADDR, data, BMP180_PROM_DATA__LEN, UBI_YES);
	//comres = bus_read(p_bmp180->dev_addr, BMP180_PROM_START__ADDR, data, BMP180_PROM_DATA__LEN);

	/*parameters AC1-AC6*/
	p_bmp180->cal_param.ac1 =  (data[0] << 8) | data[1];
	p_bmp180->cal_param.ac2 =  (data[2] << 8) | data[3];
	p_bmp180->cal_param.ac3 =  (data[4] << 8) | data[5];
	p_bmp180->cal_param.ac4 =  (data[6] << 8) | data[7];
	p_bmp180->cal_param.ac5 =  (data[8] << 8) | data[9];
	p_bmp180->cal_param.ac6 = (data[10] << 8) | data[11];

	/*parameters B1,B2*/
	p_bmp180->cal_param.b1 =  (data[12] << 8) | data[13];
	p_bmp180->cal_param.b2 =  (data[14] << 8) | data[15];

	/*parameters MB,MC,MD*/
	p_bmp180->cal_param.mb =  (data[16] << 8) | data[17];
	p_bmp180->cal_param.mc =  (data[18] << 8) | data[19];
	p_bmp180->cal_param.md =  (data[20] << 8) | data[21];
	return comres;
}

/* 
Calculates temperature from ut. ut was read from the device via I2C and fed into the right calc path for BMP180.
Arguments -- unsigned long ut. ut read from device by funtion bmp180_get_ut().

Returns temperature(long) in steps of 0.1 deg celsius
*/
long bmp180_get_temperature(unsigned long ut)
{
	long temperature;
	long x1, x2;
	if (p_bmp180->sensortype == BOSCH_PRESSURE_BMP180) {
		x1 = (((long) ut - (long) p_bmp180->cal_param.ac6) * (long) p_bmp180->cal_param.ac5) >> 15;
		x2 = ((long) p_bmp180->cal_param.mc << 11) / (x1 + p_bmp180->cal_param.md);
		p_bmp180->param_b5 = x1 + x2;
	}
	temperature = ((p_bmp180->param_b5 + 8) >> 4);  /* temperature in 0.1 deg C*/
	return temperature;
}

/*
Calculates pressure from up. up was read from the device via I2C and fed into the right calc path for BMP180. In case of BMP180 averaging is done through oversampling by the sensor IC
Arguments -- unsigned long up. up read from device by funtion bmp180_get_up().

Returns pressure(long) in steps of 1.0 Pa
*/
long bmp180_get_pressure(unsigned long up)
{
	long pressure, x1, x2, x3, b3, b6;
	unsigned long b4, b7;

	b6 = p_bmp180->param_b5 - 4000;
	/*****calculate B3************/
	x1 = (b6*b6) >> 12;
	x1 *= p_bmp180->cal_param.b2;
	x1 >>= 11;

	x2 = (p_bmp180->cal_param.ac2*b6);
	x2 >>= 11;

	x3 = x1 + x2;

	b3 = (((((long)p_bmp180->cal_param.ac1)*4 + x3) << p_bmp180->oversampling_setting)+2) >> 2;

	/*****calculate B4************/
	x1 = (p_bmp180->cal_param.ac3 * b6) >> 13;
	x2 = (p_bmp180->cal_param.b1 * ((b6*b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (p_bmp180->cal_param.ac4 * (unsigned long) (x3 + 32768)) >> 15;

	b7 = ((unsigned long)(up - b3) * (50000>>p_bmp180->oversampling_setting));
	if (b7 < 0x80000000)
		pressure = (b7 << 1) / b4;
	else
		pressure = (b7 / b4) << 1;

	x1 = pressure >> 8;
	x1 *= x1;
	x1 = (x1 * PARAM_MG) >> 16;
	x2 = (pressure * PARAM_MH) >> 16;
	pressure += (x1 + x2 + PARAM_MI) >> 4;/* pressure in Pa*/
	return pressure;
}

/*
Read out ut(raw temperature value) for temperature conversion.
Arguments -- None.

Returns ut(unsigned short) that represents the uncompensated temperature sensors conversion value
*/
unsigned long bmp180_get_ut(void)
{
	unsigned long ut;
	unsigned char data[2];
	unsigned char ctrl_reg_data;
	int wait_time;
	int comres;
	if (p_bmp180->chip_id == BMP180_CHIP_ID  /* get bitslice */) {
		ctrl_reg_data = BMP180_T_MEASURE;
		wait_time = BMP180_TEMP_CONVERSION_TIME;
	}
	comres = p_bmp180->I2C_BUS_WRITE_FUNC(p_bmp180->dev_addr, BMP180_CTRL_MEAS_REG, &ctrl_reg_data, 1, UBI_YES);

	p_bmp180->delay_msec(wait_time);
	comres += p_bmp180->I2C_BUS_READ_FUNC(p_bmp180->dev_addr, BMP180_ADC_OUT_MSB_REG, data, 2, UBI_YES);

	ut = (data[0] << 8) | data[1];
	return ut;
}

/*
Read out up(raw pressure value) for pressure conversion depending on the oversampling ratio setting up can be 16 to 19 bit
Arguments -- None.

Returns up(unsigned long) that represents the uncompensated pressure value
*/
unsigned long bmp180_get_up(void)
{
	int j;         /* j included for loop*/
	unsigned long up = 0;
	unsigned long sum = 0; /* get the calculated pressure data*/
	unsigned char data[3];
	unsigned char ctrl_reg_data;
	int comres = 0;
	if (p_bmp180->chip_id == BMP180_CHIP_ID && p_bmp180->sw_oss == 1 && p_bmp180->oversampling_setting == 3) {
		for (j = 0 ; j < 3; j++) {
			/* 3 times getting pressure data*/
			ctrl_reg_data = BMP180_P_MEASURE + (p_bmp180->oversampling_setting << 6);
			comres = p_bmp180->I2C_BUS_WRITE_FUNC(p_bmp180->dev_addr, BMP180_CTRL_MEAS_REG, &ctrl_reg_data, 1, UBI_YES);
			p_bmp180->delay_msec(2 + (3 << (p_bmp180->oversampling_setting)));
			comres += p_bmp180->I2C_BUS_READ_FUNC(p_bmp180->dev_addr, BMP180_ADC_OUT_MSB_REG, data, 3, UBI_YES);
			sum = (((unsigned long) data[0] << 16) | ((unsigned long) data[1] << 8) |	(unsigned long) data[2]) >> (8-p_bmp180->oversampling_setting);
			p_bmp180->number_of_samples = 1;
			up = up + sum;  /*add up with dummy var*/
		}
		up = up / 3;   /* averaging*/
	}
	else    {
		if (p_bmp180->chip_id == BMP180_CHIP_ID && p_bmp180->sw_oss == 0) {
			ctrl_reg_data = BMP180_P_MEASURE + (p_bmp180->oversampling_setting << 6);
			comres = p_bmp180->I2C_BUS_WRITE_FUNC(p_bmp180->dev_addr, BMP180_CTRL_MEAS_REG, &ctrl_reg_data, 1, UBI_YES);
			p_bmp180->delay_msec(2 + (3 << (p_bmp180->oversampling_setting)));
			comres += p_bmp180->I2C_BUS_READ_FUNC(p_bmp180->dev_addr, BMP180_ADC_OUT_MSB_REG, data, 3, UBI_YES);

			up = (((unsigned long) data[0] << 16) | ((unsigned long) data[1] << 8) | (unsigned long) data[2]) >> (8-p_bmp180->oversampling_setting);
			p_bmp180->number_of_samples = 1;
		}
	}
	return up;
}

/***********************************************************************************************************************
 ****************************************   TSL2561 - Ambient Light Sensor  ********************************************
 ***********************************************************************************************************************/

struct tsl2561_t *p_tsl2561;

/*
Function initailises tsl2561 chip. 
Arguments -- struct tsl2561_t. Fill the I2C read, I2C write and delay funtion pointers in the structure before passing into this function.
This function also sets gain =0 and integration time = 402ms. If you want to set different values for this set gain and integration feild in the struct tsl2561_t before passing to this funtion. Also comment setting defualt lines mentioned below. 

Returns 1 upon successful initialisation.
*/
int tsl2561_init(struct tsl2561_t *tsl2561_t)
{
	uint8_t data=0,command,ret;
	p_tsl2561 = tsl2561_t;
	p_tsl2561->dev_addr = TSL2561_I2C_ADDR;
	command = TSL2561_COMMAND_CMD + TSL2561_REG_CONTROL;
	data = TSL2561_POWERON;
	ret = p_tsl2561->I2C_BUS_WRITE_FUNC(p_tsl2561->dev_addr, command, &data, 1, UBI_YES); /* Enable the chip */
	/* read control register */
/*	ret = p_tsl2561->I2C_BUS_READ_FUNC(p_tsl2561->dev_addr, command, &data, 1, UBI_YES);
	if((data & 0x03) != TSL2561_POWERON)
	{
		return UBI_FAIL;
	}	
	command = TSL2561_COMMAND_CMD + TSL2561_REG_ID;
	ret = p_tsl2561->I2C_BUS_READ_FUNC(p_tsl2561->dev_addr, command, &data, 1, UBI_YES);
	
	p_tsl2561->chip_id = data;
	p_tsl2561->partno = (data >> 4);
	p_tsl2561->revno = (data & 0x0F);*/
	p_tsl2561->gain = TSL2561_GAIN_0X; /* Setting default value. If you want to pass different value at time of init comment this line */
	p_tsl2561->integration = TSL2561_INTEGRATIONTIME_402MS;/* Setting default value. If you want to pass different value at time of init comment this line */

	command = TSL2561_COMMAND_CMD + TSL2561_REG_TIMING;
        data =  (p_tsl2561->gain) + (p_tsl2561->integration);
        ret = p_tsl2561->I2C_BUS_WRITE_FUNC(p_tsl2561->dev_addr, command, &data, 1, UBI_YES); /* Setting Gain and integration value */
	
	if(ret) {}
	return UBI_SUCCESS;
}

/*
Function gets raw light intesity from tsl2561 chip. 
Arguments -- uint8_t *raw_data (data pointer). Fetched four bytes of data will be filled in the memory pointed by raw_data. First two bytes are of channel 0 and next two bytes are of channel 1

Returns 1 upon successful initialisation.
*/
int tsl2561_get_raw_light_intensity(uint8_t *raw_data)
{
	uint8_t command,ret;
/*
	command = TSL2561_COMMAND_CMD + TSL2561_REG_DATA0LOW;
	ret = p_tsl2561->I2C_BUS_READ_FUNC(p_tsl2561->dev_addr, command, &raw_data[0], 1, UBI_YES);
	command = TSL2561_COMMAND_CMD + TSL2561_REG_DATA0HIGH;
        ret = p_tsl2561->I2C_BUS_READ_FUNC(p_tsl2561->dev_addr, command, &raw_data[1], 1, UBI_YES);
	command = TSL2561_COMMAND_CMD + TSL2561_REG_DATA1LOW;
	ret = p_tsl2561->I2C_BUS_READ_FUNC(p_tsl2561->dev_addr, command, &raw_data[2], 1, UBI_YES);
	command = TSL2561_COMMAND_CMD + TSL2561_REG_DATA1HIGH;
	ret = p_tsl2561->I2C_BUS_READ_FUNC(p_tsl2561->dev_addr, command, &raw_data[3], 1, UBI_YES);
*/
	command = TSL2561_COMMAND_CMD + TSL2561_COMMAND_BLOCK + TSL2561_REG_DATA0LOW;
	ret = p_tsl2561->I2C_BUS_READ_FUNC(p_tsl2561->dev_addr, command, raw_data, 4, UBI_YES);
	if(ret) {}

	return UBI_SUCCESS;
}

/*
Function converts raw light intesity into lux. 
Arguments -- uint8_t *raw_data (data pointer). Calculations will be based on data pointer by raw_data pointer. Expects the output of function tsl2561_get_raw_light_intensity() as the input to this funtion.

Returns lux value(uint32_t) upon successful completion.
*/
uint32_t tsl2561_get_lux_light_intensity(uint8_t *raw_data)
{
	uint16_t ch_0=0,ch_1=0;
	//	float ch_div=0;

	unsigned long chScale, channel1, channel0;
	unsigned long ratio, ratio1, temp;
	unsigned int b, m;
	uint32_t lux;

	ch_0 = raw_data[1];
	ch_0 = (( ch_0 << 8 ) | raw_data[0] );
	ch_1 = raw_data[3];
	ch_1 = (( ch_1 << 8 ) | raw_data[2] );
	//        ch_div = ((float)ch_1 / ch_0);

	chScale = (1 << TSL2561_LUX_CHSCALE); // No scaling ... default integration time = 402ms

	switch (p_tsl2561->integration)
	{
		case TSL2561_INTEGRATIONTIME_13MS:
			chScale = TSL2561_LUX_CHSCALE_TINT0;
			break;
		case TSL2561_INTEGRATIONTIME_101MS:
			chScale = TSL2561_LUX_CHSCALE_TINT1;
			break;
		default: // No scaling ... integration time = 402ms
			chScale = (1 << TSL2561_LUX_CHSCALE);
			break;
	}

	// Scale for gain (1x or 16x)
	if (!(p_tsl2561->gain)) chScale = (chScale << 4);

	// scale the channel values
	channel0 = (ch_0 * chScale) >> TSL2561_LUX_CHSCALE;
	channel1 = (ch_1 * chScale) >> TSL2561_LUX_CHSCALE;

	// find the ratio of the channel values (Channel1/Channel0)
	ratio1 = 0;
	if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;

	// round the ratio value
	ratio = ((ratio1 + 1) >> 1 );

	if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
	{b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
	else if (ratio <= TSL2561_LUX_K2T)
	{b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
	else if (ratio <= TSL2561_LUX_K3T)
	{b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
	else if (ratio <= TSL2561_LUX_K4T)
	{b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
	else if (ratio <= TSL2561_LUX_K5T)
	{b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
	else if (ratio <= TSL2561_LUX_K6T)
	{b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
	else if (ratio <= TSL2561_LUX_K7T)
	{b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
	else if (ratio > TSL2561_LUX_K8T)
	{b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}

	temp = ((channel0 * b) - (channel1 * m));

	// do not allow negative lux value
	if (temp < 0) temp = 0;

	// round lsb (2^(LUX_SCALE-1))
	temp += (1 << (TSL2561_LUX_LUXSCALE-1));

	// strip off fractional portion
	lux = temp >> TSL2561_LUX_LUXSCALE;

	// Signal I2C had no errors
	return lux;

	/*
	   if((ch_div > 0) && (ch_div <= 0.50))
	   {
	   return ((0.0304*(float)ch_0)-(0.062*(float)ch_0)*( pow(ch_div, 1.4) ));
	   if((ch_div>0) && (ch_div<=0.125)) { return ((0.0304*(float)ch_0)-((0.0272*(float)ch_0)*ch_div)); }
	   if((ch_div>0.125) && (ch_div<=0.250)) { return ((0.0325*(float)ch_0)-((0.0440*(float)ch_0)*ch_div)); }
	   if((ch_div>0.250) && (ch_div<=0.375)) { return ((0.0351*(float)ch_0)-((0.0544*(float)ch_0)*ch_div)); }
	   if((ch_div>0.375) && (ch_div<=0.50)) { return ((0.0381*(float)ch_0)-((0.0624*(float)ch_0)*ch_div)); } 
	   }
	   if((ch_div > 0.50) && (ch_div <= 0.61)) { return ((0.0224*(float)ch_0)-(0.031*(float)ch_1)); }
	   if((ch_div > 0.61) && (ch_div <= 0.80)) { return ((0.0128*(float)ch_0)-(0.0153*(float)ch_1)); }
	   if((ch_div > 0.80) && (ch_div <= 1.30)) { return ((0.00146*(float)ch_0)-(0.00112*(float)ch_1)); }
	   if(ch_div > 1.30)	{ return 0; }

	   return 0;
	 */
}

/* End of file */
