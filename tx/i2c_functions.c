#include "i2c_functions.h"

//volatile int timer_experied;


char bus_read(unsigned char device_addr, unsigned char register_addr, unsigned char* register_data, unsigned char read_length, unsigned char reg_present)
{
    uint8_t recv = 0;
        
    if((read_length == 0) || register_data == '\0') {
        return -1;
    }

    if(reg_present) 
    {
        I2CMasterSlaveAddrSet(device_addr, false);
        I2CMasterDataPut(register_addr);
        I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_START);
        while(I2CMasterBusy());
        
        if(read_length == 1) {
    
            uint32_t temp;
    
            I2CMasterSlaveAddrSet(device_addr, true);
            I2CMasterControl(I2C_MASTER_CMD_SINGLE_RECEIVE);
    
            while(I2CMasterBusy());
            temp = I2CMasterErr();
            if(temp == I2C_MASTER_ERR_NONE) {
                *register_data = I2CMasterDataGet();
            }          
        }
        else
        {
            I2CMasterSlaveAddrSet(device_addr, true);
            I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_START);
            while(I2CMasterBusy());
            if(I2CMasterErr() == I2C_MASTER_ERR_NONE) 
            {
                register_data[0] = I2CMasterDataGet();
                /* If we got 2 or more bytes pending to be received, keep going*/
                for(recv = 1; recv <= (read_length - 2); recv++) {
                    I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                    while(I2CMasterBusy());
                    register_data[recv] = I2CMasterDataGet();
                }
                /* This should be the last byte, stop receiving */
                I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                while(I2CMasterBusy());
                register_data[read_length - 1] = I2CMasterDataGet();
            }
        }
    }
    else
    {
        if(read_length == 1) 
        {
    
            uint32_t temp;
    
            I2CMasterSlaveAddrSet(device_addr, true);
            I2CMasterControl(I2C_MASTER_CMD_SINGLE_RECEIVE);
    
            while(I2CMasterBusy());
            temp = I2CMasterErr();
            if(temp == I2C_MASTER_ERR_NONE) {
                *register_data = I2CMasterDataGet();
            }
            //return temp;           
        }
        else
        {
            I2CMasterSlaveAddrSet(device_addr, true);
            I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_START);
            while(I2CMasterBusy());
            if(I2CMasterErr() == I2C_MASTER_ERR_NONE) {
                register_data[0] = I2CMasterDataGet();
                /* If we got 2 or more bytes pending to be received, keep going*/
                for(recv = 1; recv <= (read_length - 2); recv++) {
                    I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                    while(I2CMasterBusy());
                    register_data[recv] = I2CMasterDataGet();
                }
                /* This should be the last byte, stop receiving */
                I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                while(I2CMasterBusy());
                register_data[read_length - 1] = I2CMasterDataGet();
            }
        }
    }
    // return I2CMasterErr();
    return read_length;
}

char bus_write(unsigned char device_addr, unsigned char register_addr, unsigned char* register_data, unsigned char write_length, unsigned char reg_present)
{
    uint8_t sent;
   
    if((write_length == 0) || (register_data == '\0')) {
        return -1;
    }

    if(reg_present)
    {
        I2CMasterSlaveAddrSet(device_addr, false);
        I2CMasterDataPut(register_addr);
        I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_START);
        while(I2CMasterBusy());
        if(I2CMasterErr() == I2C_MASTER_ERR_NONE) {
            for(sent = 0; sent <= (write_length - 2); sent++) {
                I2CMasterDataPut(register_data[sent]);
                I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_CONT);
                while(I2CMasterBusy());
            }
            /* This should be the last byte, stop sending */
            I2CMasterDataPut(register_data[write_length - 1]);
            I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_FINISH);
            while(I2CMasterBusy());
        }
    }
    else
    {
        if(write_length == 1) {

            I2CMasterSlaveAddrSet(device_addr, false);
            I2CMasterDataPut(register_data[0]);
            I2CMasterControl(I2C_MASTER_CMD_SINGLE_SEND);
            while(I2CMasterBusy());

            /* Return the STAT register of I2C module if error occured, I2C_MASTER_ERR_NONE otherwise */
            //return I2CMasterErr();
            return write_length;
        }
        I2CMasterSlaveAddrSet(device_addr, false);
        I2CMasterDataPut(register_data[0]);
        I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_START);
        while(I2CMasterBusy());
        if(I2CMasterErr() == I2C_MASTER_ERR_NONE) {
            for(sent = 1; sent <= (write_length - 2); sent++) {
                I2CMasterDataPut(register_data[sent]);
                I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_CONT);
                while(I2CMasterBusy());
            }
            /* This should be the last byte, stop sending */
            I2CMasterDataPut(register_data[write_length - 1]);
            I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_FINISH);
            while(I2CMasterBusy());
        }
    }
    //return I2CMasterErr();
    return write_length;
}

void delay_msec (unsigned int delay)
{
    int i;
    for(i=0;i<delay;++i)
    { SysCtrlDelay(10666/2); }
}

/*
   void delay_msec (unsigned int delay)
   {
   int i,j;
//for(i=0;i<32;++i);
for(j=0;j<32000000;++j) { __asm(" NOP");}
}*/
