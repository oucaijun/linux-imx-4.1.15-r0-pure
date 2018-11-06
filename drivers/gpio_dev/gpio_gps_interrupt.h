
#ifndef __GPIO_GPS_INTERRUPT_H__
#define __GPIO_GPS_INTERRUPT_H__


#include <linux/module.h>  
#include <linux/device.h>  
#include <linux/fs.h>  
#include <linux/miscdevice.h>  
#include <linux/sched.h>  
#include <linux/wait.h>  
#include <linux/irqreturn.h>  
#include <asm/uaccess.h>  

#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#define CROSS_CHECK_IRQ_CNT 0xA0
#define SET_NEXT_SEC_STAMP  0xA1
#define GET_RECENT_OFFSET  0xA2
#define GET_PRE_ADJUST_TIME  0xA3
#define SETUP_ADJUST_TIME  0xA4
#define SET_SIG_VALUE  0xA5

#define GPS_CK__SIG 0xAF

struct gps_settime_info
{
	unsigned long usr_set_seconds;//设置的秒数
};

struct gps_sig_info
{
	int signal;	//当前信号强度
};

typedef void (*gi_callback)(void) ;
typedef int  (*giadc_callback)(bool valid,int32_t signal,uint64_t tick);
void registerAdcCallback(giadc_callback fun);

void registerGpsCallback(gi_callback fun);

#endif

