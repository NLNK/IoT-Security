#include "ubimotesez_config.h"
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
#include "bsp_led.h"
#include "gpio.h"
#include "uart.h"
#include "present.h"
#include "rotate.h"

#include <stdint.h>


static u8 sbox[] = {0xc, 0x5, 0x6, 0xb, 0x9, 0x0, 0xa, 0xd, 0x3, 0xe, 0xf, 0x8, 0x4, 0x7, 0x1, 0x2};
static u8 invsbox[] = {0x5, 0xe, 0xf, 0x8, 0xC, 0x1, 0x2, 0xD, 0xB, 0x4, 0x6, 0x3, 0x0, 0x7, 0x9, 0xA};



volatile unsigned char timer_experied = 0;

moteCfg_t moteConfig = {0};
basicRfCfg_t basicRfConfig;


u8 inputKey[10] = {0x00};
u8 keys[PRESENT_KEY_SIZE_80/8*(PRESENT_ROUNDS+1)];





static void receive(void);

static void UARTlibinit(void);
static void present_64_80_test(void);
static void delay(uint16_t mSec);
static void timerIsr(void);

int main(void){

	// Initialize board
	bspInit(BSP_SYS_CLK_SPD);


	UARTlibinit();


	present_64_80_test();


	// Initialize keys and key interrupts
	bspKeyInit(BSP_KEY_MODE_ISR);
	bspKeyIntEnable(BSP_USER_KEY);

	UARTlibinit();

		moteConfig.mode = MOTE_MODE_RX;
    	moteConfig.channel = CHANNEL;
    	moteConfig.txPower = 5;          		// Index 0. Max output
    	moteConfig.gainMode = MOTE_GAIN_MODE_NONE; 	// No PA/LNA

	// Enable interrupts
    	halIntOn();

	// Config basicRF
    	basicRfConfig.panId = PAN_ID;
    	basicRfConfig.ackRequest = false;
	
	receive();


	return 0;

}

static void receive(void){

		signed short ssRssi;
		int i;

		basicRfConfig.myAddr = RX_ADDR;
    	basicRfConfig.channel = moteConfig.channel;
    	if(basicRfInit(&basicRfConfig)==FAILED){
		UARTprintf("rf init failed\n\r");
		while(1);
    	}
	
	if(moteConfig.gainMode != MOTE_GAIN_MODE_NONE){

        	// Set gain mode
        	halRfSetGain(moteConfig.gainMode);
    	}

	// Start RX
	basicRfReceiveOn();
	
	while(1) {
        	
		while(!basicRfPacketIsReady());

				uint8_t payload[48];
	        	if(basicRfReceive((unsigned char*)payload, 103, &ssRssi) > 0) {
            		
				//bspLedToggle(BSP_LED_ALL);

        		int k=0;
				UARTprintf("\nEncrypted data: ");
				while(payload[k]!=0)
				{
				 UARTprintf("%x ",payload[k]);
				 k++;
				}
        		
        		present_64_80_decrypt(payload,keys);


				UARTprintf("\n\r");
        	}
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
 * plainText: one block of plain text
 * roundKeys: round keys
 *
 */

/*
 * one block is decrypted
 * cipherText: one block of cipher text
 * roundKeys: round keys
 */
static void present_decrypt(u8 *cipherText, const u8 *roundKeys) {
	u64 state = *(u64*)cipherText;
	const u64* rk = (const u64*)roundKeys;
	u64 result;
	u8 sInput; // every nibble of sbox
	u8 pLayerIndex; // the output position of every bit in pLayer
	u64 stateBit; // the input value of every bit in pLayer
	u8 i; // rounds
	u16 k;
	
	for (i = PRESENT_ROUNDS; i > 0; i--){
		state ^= rk[i];

		/* pLayer */
		result = 0;
		for (k = 0; k < PRESENT_BLOCK_SIZE; k++) {
			stateBit = state & 0x1;
			state = state >> 1;
			if ( 0 != stateBit ) {
				pLayerIndex = (4 * k) % 63;
				if (63 == k) {										
					pLayerIndex = 63;
				}
				result |= stateBit << pLayerIndex; 
			}
		}
		state = result;

		/* sbox */
		for (k = 0; k < PRESENT_BLOCK_SIZE/4; k++) {
			sInput = state & 0xF;
			state &= 0xFFFFFFFFFFFFFFF0; 
			state |= invsbox[sInput];
			state = ror64(state, 4); 
		}

	}
	
	state ^= rk[i];
	*(uint64_t*)cipherText = state;
}


static void present_64_80_decrypt (u8 * plainText, const u8 * keys)
{
	/*
	 * PRESENT 64/80
	 */
	 

	
 	present_decrypt(plainText, keys);

	int k=0;
	UARTprintf("\nAfter decryption: ");
	while(plainText[k]!=0)
	{
	 UARTprintf("%c ",plainText[k]);
	 k++;
	}
	UARTprintf("\n");
 
               
}  
static void present_64_80_test(void) {
	/*
	 * PRESENT 64/80
	 */
	
	present_64_80_key_schedule(inputKey, keys);                 
         
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
