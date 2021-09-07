
#include "bsp.h"
#include "bsp_key.h"
#include "hal_int.h"
#include "basic_rf.h"
#include "hal_rf.h"
#include "hal_timer_32k.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "bsp_led.h"
#include "uartstdio.h"
#include "hw_ioc.h"
#include "ioc.h"
#include "string.h"
#include "sys_ctrl.h"

#include "present.h"
#include "rotate.h"


#include "i2c_functions.h"
#include "ubisense.h"


typedef struct
{
    unsigned char mode;             //!< PER test mode. [RX|TX]
    unsigned char channel;          //!< PER test IEEE channel [11,26]
    unsigned char txPower;          //!< PER test TX power
    unsigned char gainMode;         //!< Gain mode
} moteCfg_t;


#define MOTE_MODE_TX             0
#define MOTE_MODE_RX             1
#define CHANNEL			        0x12

#define MOTE_GAIN_MODE_LO        0       // Same value as in hal_rf
#define MOTE_GAIN_MODE_HI        1       // Same value as in hal_rf
#define MOTE_GAIN_MODE_NONE      42
#define MAX_PAYLOAD		103

static u8 sbox[] = {0xc, 0x5, 0x6, 0xb, 0x9, 0x0, 0xa, 0xd, 0x3, 0xe, 0xf, 0x8, 0x4, 0x7, 0x1, 0x2};
static u8 invsbox[] = {0x5, 0xe, 0xf, 0x8, 0xC, 0x1, 0x2, 0xD, 0xB, 0x4, 0x6, 0x3, 0x0, 0x7, 0x9, 0xA};
     

moteCfg_t moteConfig = {0};
//basicRfCfg_t basicRfConfig;
volatile unsigned char timer_experied = 0;
//uint8_t data[]="Hi.. Welcome to SETS";

//uint8_t rxByte[] = "aaaabbbb";  

//static uint8_t *rxByte;

uint8_t payload[MAX_PAYLOAD];
static void transmit(void);
static void UARTlibinit(void);
static void delay(uint16_t mSec);
static void timerIsr(void);


static void present_64_80_test(void);


#define EXAMPLE_PIN_I2C_SCL             GPIO_PIN_3
#define EXAMPLE_PIN_I2C_SDA             GPIO_PIN_5
#define EXAMPLE_GPIO_I2C_BASE           GPIO_B_BASE

#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_UART_BASE          GPIO_A_BASE

#define TEMP_STRT_BYTE  0xAA
#define PRESS_STRT_BYTE 0xBB
#define PRXY_STRT_BYTE  0xCC
#define LGHT_STRT_BYTE  0xDD
#define HUMID_STRT_BYTE 0xEE

#define PAN_ID                  0xDCBA
#define TX_ADDR                 0xCCEB
#define RX_ADDR                 0x2538

#define PER_GAIN_MODE_LO        0       // Same value as in hal_rf
#define PER_GAIN_MODE_HI        1       // Same value as in hal_rf
#define PER_GAIN_MODE_NONE      42

typedef struct
{
    unsigned char mode;             //!< PER test mode. [RX|TX]
    unsigned char state;            //!< PER test state (idle, transmit)
    unsigned char channel;          //!< PER test IEEE channel [11,26]
    unsigned char txPower;          //!< PER test TX power
    unsigned long burstSize;        //!< Number of packets to send in TX
    unsigned long pktRate;          //!< Number of packets per second
    unsigned char gainMode;         //!< Gain mode (CC2591 PA/LNA)
} perCfg_t;

enum sensor_data_position {
  TEMP_POS      =       0,
  PRESS_POS     =       5,
  LGHT_POS      =       10,
  HUMID_POS     =       15,
  PRXY_POS      =       20
};

perCfg_t perConfig = {0};
static basicRfCfg_t basicRfConfig;
static uint8_t rxByte[127]; 



    struct bmp180_t bmp180_data;
    struct tsl2561_t tsl2561_data;
    struct sht21_t sht21_data;
    struct tsl26711_t tsl26711_data;



int main(void){

	// Initialize board
	bspInit(BSP_SYS_CLK_SPD);

	// Initialize keys and key interrupts
	bspKeyInit(BSP_KEY_MODE_ISR);
	bspKeyIntEnable(BSP_USER_KEY);

	UARTlibinit();

	moteConfig.mode = MOTE_MODE_RX;
    	moteConfig.channel = CHANNEL;
    	moteConfig.txPower = 7;          		// Index 0. Max output
    	moteConfig.gainMode = MOTE_GAIN_MODE_HI; 	// No PA/LNA

	// Enable interrupts
    	halIntOn();

	// Config basicRF
    	basicRfConfig.panId = PAN_ID;
    	basicRfConfig.ackRequest = false;

		//present_64_80_test();


    	while(1)
	{

	//UARTprintf("\nWelcome");
	transmit();

	delay(1000);
	}




























	
}

/*
 * Key schedule for 80-bit
 * key: master key
 * roundKeys: round keys

 */

static void present_64_80_key_schedule( const u8 *key, u8 *roundKeys) {
	u64 keylow = *(const u64*)key;
	u16 highBytes = *(const u16*)(key + 8);
	u64 keyhigh = ((u64)(highBytes) << 48) | (keylow >> 16);
	u64 *rk = (u64*)roundKeys;
	rk[0] = keyhigh;

	u64 temp;
	u8 i;
	

	for (i = 0; i < PRESENT_ROUNDS; i++) {
		/* 61-bit left shift */
		temp = keyhigh;
		keyhigh <<= 61;
		keyhigh |= (keylow << 45);
		keyhigh |= (temp >> 19);
		keylow = (temp >> 3) & 0xFFFF;

		/* S-Box application */
		temp = sbox[keyhigh >> 60];
		keyhigh &= 0x0FFFFFFFFFFFFFFF;
		keyhigh |= temp << 60;

		/* round counter addition */
		keylow ^= (((u64)(i + 1) & 0x01) << 15);
		keyhigh ^= ((u64)(i + 1) >> 1);

		rk[i+1] = keyhigh;
	}
}

/*
 * Key schedule for 128-bit
 * key: master key
 * roundKeys: round keys
 */

static void present_64_128_key_schedule( const u8 *key, u8 *roundKeys) {
	u64 keylow = *(const u64*)key;
	u64 keyhigh = *((const u64*)key+1);
	u64 *rk = (u64*)roundKeys;
	rk[0] = keyhigh;

	u64 temp;
	u8 i;

	for (i = 0; i < PRESENT_ROUNDS; i++) {
		/* 61-bit left shift */
		temp = ( (keyhigh<<61) | (keylow>>3) );
		keylow = ( (keylow<<61) | (keyhigh>>3) );
		keyhigh = temp;

		/* S-Box application */
		temp = (sbox[keyhigh>>60]<<4) ^ (sbox[(keyhigh>>56)&0xf]);
		keyhigh &= 0x00FFFFFFFFFFFFFF;
		keyhigh |= temp<<56;

		/* round counter addition */
		temp = ((keyhigh<<2) | (keylow>>62)) ^ (u64)(i + 1);
		keyhigh = (keyhigh & 0xFFFFFFFFFFFFFFF8) ^ (temp & 0x7);
		keylow = (keylow & 0x3FFFFFFFFFFFFFFF) ^ (temp << 62);

		rk[i+1] = keyhigh;
	}
}

/*
 * one block is encrypted
 * rxByte: one block of plain text
 * roundKeys: round keys
 *
 */
static void present_encrypt(u8 *rxByte, const u8 *roundKeys) {
	u64 state = *(u64*)rxByte;
	const u64* rk = (const u64*)roundKeys;
	u64 result;
	u8 sInput; // every nibble of sbox
	u8 pLayerIndex; // the output position of every bit in pLayer
	u64 stateBit; // the input value of every bit in pLayer
	u8 i; // rounds
	u16 k;
	
	for (i = 0; i < PRESENT_ROUNDS; i++) {		
		state ^= rk[i];

		/* sbox */
		for (k = 0; k < PRESENT_BLOCK_SIZE/4; k++) {
			sInput = state & 0xF;
			state &= 0xFFFFFFFFFFFFFFF0; 
			state |= sbox[sInput];
			state = ror64(state, 4); 
		}
		
		/* pLayer */
		result = 0;
		for (k = 0; k < PRESENT_BLOCK_SIZE; k++) {
			stateBit = state & 0x1;
			state = state >> 1;
			if ( 0 != stateBit ) {
				pLayerIndex = (16 * k) % 63;
				if (63 == k) {
					pLayerIndex = 63;
				}
				/*
				 * result |= 0x1 << pLayerIndex;
				 * 0x1 is 32-bit by default
				 */
				result |= stateBit << pLayerIndex; 
			}
		}
		state = result;
	}

	state ^= rk[i];
	*(u64*)rxByte = state;
}

static void present_64_80_test(void) {
	/*
	 * PRESENT 64/80
	 */
	u8 inputKey[10] = {0x00};
	u8 keys[PRESENT_KEY_SIZE_80/8*(PRESENT_ROUNDS+1)];
	 


	present_64_80_key_schedule(inputKey, keys);
	
	
	
    int i, counter, len,chars;  
    len = sizeof(rxByte)/sizeof(uint8_t);  
    chars = len;  
     
    uint8_t c[chars-1];  

               
        for(i = 0; i < len; i = i+chars) 
        {  
                counter = 0;  
                //Dividing rxByteing in n equal part using subrxByteing()  
                while (counter < chars) {  
                    rxByte[counter] = rxByte[i + counter];  
                    counter++;  
                }  
                rxByte[counter]='\0';  
                //UARTprintf("%s",rxByte);  


				UARTprintf("\nrxByte: ");
				int k=0;
				while(rxByte[k]!=0)
				{
				 UARTprintf("%c ",rxByte[k]);
				 k++;
				}

				


				present_encrypt(rxByte,keys);
				
				
				
				/*UARTprintf("\nAfter encryption: ");
				
				k=0;
				while(rxByte[k]!=0)
				{
				 UARTprintf("%x ",rxByte[k]);
				 k++;
				}*/

		}

}


static void transmit(void){
	
    	basicRfConfig.myAddr = TX_ADDR;
    	basicRfConfig.channel = moteConfig.channel;
	
	if(basicRfInit(&basicRfConfig) == FAILED){
		while(1);
    	}
	
	// Turn receiver off
    basicRfReceiveOff();
	
	halRfSetTxPower(moteConfig.txPower);


unsigned char raw_data[4],i;
    uint32_t lux;
    int raw_humi,raw_temp,proximity;
    float fhumi,ftemp;
    
    short temperature;
    unsigned long ut;
    long pressure;  
    unsigned long up;
    uint32_t ui32Val;
    
    // Set the clocking to run directly from the external crystal/oscillator. (no ext 32k osc, no internal osc)
    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_16MHZ);
    //SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);

    // Set IO clock to the same as system clock
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_16MHZ);    
   // SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);    

    
    //  The I2C peripheral must be enabled before use.
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_I2C);
    
    // Do reset of I2C module
    SysCtrlPeripheralReset(SYS_CTRL_PERIPH_I2C);

    // Configure I2C pins
    GPIOPinTypeI2C(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SCL);
    GPIOPinTypeI2C(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SDA);  

    // Configure pins as peripheral input and output        
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SCL,IOC_MUX_OUT_SEL_I2C_CMSSCL);
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SDA,IOC_MUX_OUT_SEL_I2C_CMSSDA);
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SCL, IOC_I2CMSSCL);
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SDA, IOC_I2CMSSDA);
    
    // If false the data rate is set to 100kbps and if true the data rate will be set to 400kbps.  
    I2CMasterInitExpClk(SysCtrlClockGet(), false); 

    I2CSlaveDisable();
   
    bmp180_data.bus_write = bus_write;
    bmp180_data.bus_read = bus_read;
    bmp180_data.delay_msec = delay_msec;
    bmp180_init(&bmp180_data);
    
    tsl2561_data.bus_write = bus_write;
    tsl2561_data.bus_read = bus_read;
    tsl2561_data.delay_msec = delay_msec;
    tsl2561_init(&tsl2561_data);
    
    sht21_data.bus_write = bus_write;
    sht21_data.bus_read = bus_read;
    sht21_data.delay_msec = delay_msec;
    sht21_init(&sht21_data);
    
    tsl26711_data.bus_write = bus_write;
    tsl26711_data.bus_read = bus_read;
    tsl26711_data.delay_msec = delay_msec;
    tsl26711_init(&tsl26711_data);    
   
    bspLedInit();

	//
    // Set default configuration (used by SmartRF06 batteryboard)
    //
 	perConfig.channel = 26;
    perConfig.txPower = 22;          // Index 0. Max output
    perConfig.gainMode = PER_GAIN_MODE_HI; //PA/LNA

	//
    // Enable interrupts
    //
    halIntOn();

    //
    // Config basicRF
    //
    basicRfConfig.panId = PAN_ID;
    basicRfConfig.ackRequest = false;
	basicRfConfig.myAddr = TX_ADDR;
    basicRfConfig.channel = perConfig.channel;
	basicRfConfig.panId = PAN_ID;
	if(basicRfInit(&basicRfConfig)==FAILED) {
       	while(1) {
       	}
    }

	if(perConfig.gainMode != PER_GAIN_MODE_NONE) {
       	//
       	// Set gain mode
       	//
       	halRfSetGain(perConfig.gainMode);
    }
	halRfSetTxPower(perConfig.txPower);
	
	//
    // Turn receiver off
    //
    basicRfReceiveOff();
    //IntMasterEnable();
    
		ut = bmp180_get_ut();
        temperature= bmp180_get_temperature(ut);
        up = bmp180_get_up();
        pressure = bmp180_get_pressure(up);
		rxByte[PRESS_POS] = PRESS_STRT_BYTE;
    	for(i=0;i<4;i++){
      		rxByte[PRESS_POS+1+i] = (((pressure)>>(i*8)) & 0x000000FF);
    	}
		
        UARTprintf("\nPressure= %d",pressure);
		
		tsl2561_get_raw_light_intensity(raw_data);
        lux = tsl2561_get_lux_light_intensity (raw_data);
        UARTprintf("\nLight= %d",lux);
		rxByte[LGHT_POS] = LGHT_STRT_BYTE;
    	for(i=0;i<4;i++){
      		rxByte[LGHT_POS+1+i] = (((lux)>>(i*8)) & 0x000000FF);
    	}
        raw_humi = sht21_get_raw_humidity();
        raw_temp = sht21_get_raw_temperature();
        fhumi=sht21_get_caliberated_humidity(raw_humi);
        ftemp=sht21_get_caliberated_temperature(raw_temp);
	
		rxByte[TEMP_POS ] = TEMP_STRT_BYTE;
    	for(i=0;i<4;i++){
      		rxByte[TEMP_POS +1+i] = ((((uint32_t)(ftemp*100))>>(i*8)) & 0x000000FF);
    	}
		rxByte[HUMID_POS] = HUMID_STRT_BYTE;
    	for(i=0;i<4;i++){
      		rxByte[HUMID_POS+1+i] = ((((uint32_t)(fhumi*100))>>(i*8)) & 0x000000FF);
    	}
        UARTprintf("\nHumi= %d.%d",((int)fhumi),((int)(fhumi*100)%100));
        UARTprintf("\nTemperature= %d.%d",((int)ftemp),((int)(ftemp*100)%100));
		
		if(tsl26711_init(&tsl26711_data)){	
			proximity = tsl26711_get_proximity();
			UARTprintf("\nProximity= %d",proximity);
		}
		bspLedToggle(BSP_LED_ALL);
		//delay_msec(1000);

		
		//present_64_80_test();

		//while(1)
		//{

			present_64_80_test();
		
		//basicRfSendPacket(RX_ADDR, rxByte, 20);


	int len = sizeof(rxByte);
	
	memcpy(&payload[0],rxByte, len);
	
	while(1){

				int k=0;
				UARTprintf("\nAfter encryption: ");
				while(payload[k]!=0)
				{
				 UARTprintf("%x ",payload[k]);
				 k++;
				}
		
		bspLedClear(BSP_LED_ALL);
		basicRfSendPacket(RX_ADDR, (unsigned char *)payload, (payload[0]));
		
		bspLedSet(BSP_LED_ALL);
    


}




























	
	
		
		//delay(1000);
	//}
}

static void UARTlibinit(void){

    	// Map UART signals to the correct GPIO pins and configure them as
    	// hardware controlled.
    	IOCPinConfigPeriphOutput(GPIO_A_BASE, GPIO_PIN_1, 
                             IOC_MUX_OUT_SEL_UART0_TXD);
    	GPIOPinTypeUARTOutput(GPIO_A_BASE, GPIO_PIN_1);
    
    	IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_0, 
                            IOC_UARTRXD_UART0);
    	GPIOPinTypeUARTInput(GPIO_A_BASE, GPIO_PIN_0);
     
    	// Initialize the UART (UART0) for console I/O.
    	UARTStdioInit(0);
}

static void delay(uint16_t mSec) {
    
	timer_experied = 0;
	IntPrioritySet(INT_SMTIM, 0x80);        // Reduce timer interrupt priority
	halTimer32kInit((uint16_t)(((float)(32768/1000.0))*mSec));
    halTimer32kIntConnect(&timerIsr);    // Connect ISR
   	halTimer32kIntEnable();                 // Enable interrupts
	while(!(timer_experied == 1));
}

static void timerIsr(void){

	timer_experied = 1;
	halTimer32kIntDisable();
}

