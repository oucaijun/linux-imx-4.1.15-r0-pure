#ifndef _ADS_1256_H_
#define _ADS_1256_H_

#include "spique.h"


struct ads1256_drv{
        //欧才俊传入
	char drv_name[32];
       int  (*irq_call_back)(AdcData *data);
        //int n_ch;
        //uint32_t *cs_gpio_index;
		//韦海盛传出
    	int (*read)(int ch, char* data, int cnt);
    	int (*write)(int ch, char* data, int cnt);
    	int (*write_then_read)(int ch, char* inbuf, int inbytes, char* outbuf, int outbytes);
    	int (*ioctl)(uint32_t cmd, void* arg);
};


enum 
{
    CMD_START_C,
    CMD_STOP_C,
    CMD_ASYNC_READ_C,
    CMD_ASYNC_SET_SAMP_STAMP,
    CMD_READ_DATA_SYNC,
	CMD_GET_STATUS,
    CMD_MODE,
    CMD_HZ,
    CMD_BITS
 };

#define MXC_SPI_CS(no)	((no) - 32)

//extern int ads1256_register_driver(struct ads1256_drv* drv);
//extern int ads1256_unregister_driver(struct ads1256_drv* drv);

#endif

