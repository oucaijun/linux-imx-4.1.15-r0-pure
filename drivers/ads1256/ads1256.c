/*
 * Copyright 2004-2007, 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 Juergen Beisert
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation
 * 51 Franklin Street, Fifth Floor
 * Boston, MA  02110-1301, USA.
 */
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/ioctl.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>

#include <linux/platform_data/dma-imx.h>
#include <linux/platform_data/spi-imx.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <asm/uaccess.h>

#include "ads1256.h"
#include "../gpio_dev/gpio_gps_interrupt.h"
#include <linux/moduleparam.h>


#define DRIVER_NAME "ads1256"

#define AD_TEST_EN 0

#define DATA_PACK_COUNT 1000

#define ADC_CH_ALL 0x07


#define SPI_CMD_OPEN 0xAA000000
#define SPI_CMD_CLOSE 0xAA000001

///$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#define DEV_NAME                    "ads1256"

#define MDEBUG  (printk("XXX MDEBUG %s (%d) <%s> ",__FILE__,__LINE__,__FUNCTION__),printk)
#define MWARN  (printk("XXX MWARN %s (%d) <%s>" ,__FILE__,__LINE__,__FUNCTION__),printk)

#define ESPI1_IRQ 63

#define MAX_DEVNAME_LEN 30
#define GI_COUNT 1

#define SET_REG_MDELAY 100

#define SAMPLE_RATE_DEF 1000



struct gi_desc_struct
{
    int grp_index;
    int entry_index;
    unsigned int irq;
    irq_handler_t irq_handler;
    unsigned long flags;
    char name[MAX_DEVNAME_LEN];
};


struct adc_cali_info
{
    int32_t ofc[3];
    int32_t fsc[3];
};

#pragma pack(push,1)
struct adc_data_pack
{
    uint8_t flag;
    uint8_t datax[3];
    uint8_t datay[3];
    uint8_t dataz[3];
};
#pragma pack(pop)

extern long fpga_spi_cfg(unsigned int cmd, unsigned long arg);
extern int fpga_spi_open(void);
extern int fpga_spi_read(char  *buf, size_t count);
extern int fpga_spi_write(char *buf, size_t count);
extern int fpga_write_then_read(uint8_t  *buf_in,
                                size_t byte_in,
                                uint8_t *buf_out,
                                size_t byte_out);
extern int fpga_spi_status(void);

static uint8_t mSpiMode = 1;
static uint8_t mSpiBits = 8;
static uint32_t mSpiSpeed = 1000000;
static uint16_t mSpiDelay = 7;

static int gi_triggered[GI_COUNT];

static int register_ads1256_drdy_irq(void);
static irqreturn_t ads1256_drdy_isr(int irq,void *dev_id);

static struct class *ads1256_class;

static struct gi_desc_struct adc_ready_irq_map[GI_COUNT] =
{
    {4,5,-1,ads1256_drdy_isr,IRQF_TRIGGER_FALLING, "ads1256_ready" },
};

static char* param = "normal";
module_param(param,charp,S_IRUSR);

//adc gpio pins
#define N_GPIO 6
#define IMX_GPIO_NR(bank, nr) (((bank) - 1) * 32 + (nr))
struct gpio_idx_struct
{
    int grp_idx;
    int entry_idx;
    char dev_name[MAX_DEVNAME_LEN];
    int init_value; //GPIOF_OUT_INIT_HIGH   GPIOF_OUT_INIT_LOW
};
static struct gpio_idx_struct gpio_array[N_GPIO] =
{
#if AD_TEST_EN
    {6, 7, "FPGA_SYNC", GPIOF_OUT_INIT_HIGH  }, //test
#else
    {5, 23, "FPGA_SYNC", GPIOF_OUT_INIT_HIGH  },
#endif
    {5, 22, "ADC_RESET", GPIOF_OUT_INIT_HIGH  }, // low to reset ads1256
    {4, 6, "FPGA_POWEN", GPIOF_OUT_INIT_HIGH  },
    {1, 8, "FPGA_RESET", GPIOF_OUT_INIT_HIGH  },

    {5, 30, "FPGA_DATA_FULL", GPIOF_IN  },
    {5, 19, "FPGA_DATA_NOT_EMPTY", GPIOF_IN  },
};

#if AD_TEST_EN
static unsigned int fpga_sync = IMX_GPIO_NR(6, 7);
#else
static unsigned int fpga_sync = IMX_GPIO_NR(5, 23);
#endif
static unsigned int adc_reset = IMX_GPIO_NR(5, 22);
static unsigned int fpga_pow_en = IMX_GPIO_NR(4, 6);
static unsigned int fpga_reset = IMX_GPIO_NR(1, 8);
static unsigned int fpga_data_full = IMX_GPIO_NR(5, 30);
static unsigned int fpga_data_not_empy = IMX_GPIO_NR(5, 19);


static DECLARE_WAIT_QUEUE_HEAD(gi_ready_wait);
static int ready_flag = 0;
static volatile unsigned long _adc_irq_sum = 0; //for check
static int _adc_started = 0;
static int32_t _adc_ready_irq_registered = 0;
static struct fasync_struct *fasync;
static DEFINE_MUTEX(adc_queue_lock);
static DECLARE_WAIT_QUEUE_HEAD(in_queue_wait);
static volatile unsigned long _adc_read_sum = 0; //for check

#define ADC_DATAPACK_COUNT	100

typedef enum SYNC_STATE_
{
    SYNC_NULL,
    SYNC_SYNCING,
    SYNC_PULSE_OK,
    SYNC_TIMEOUT
} SYNC_STATE;

struct adc_data_info
{
    int mSpiFd;
    int sample_intern;
    int sample_rate;
    int32_t read_idx;
    int32_t write_idx;
    int32_t write_pos;
    int32_t full_count;
    int64_t stamp;
    int64_t pre_gps_pulse_stamp;
    int64_t pre_data_pulse_stamp;
    int64_t pre_dready_stamp;
    int64_t dready_stamp;
    int64_t read_stamp;
    uint32_t totl_count;
    uint32_t err_count;
    uint32_t err2_count;
    uint32_t after_pulse_count;
    uint32_t dready_count;
    uint32_t read_count;
    uint32_t usr_read_count;
    uint8_t pre_flag;
    uint8_t pre_id;
    int32_t gpsok;
    int32_t gps_signal;
    AdcData adc_pingpong_data[DATA_PACK_COUNT][ADC_DATAPACK_COUNT];
    bool need_cabli;//输出数据是否需要校正
    int32_t ofc[3];
    int32_t fsc[3];
    bool ad_init;
    bool time_sync;
    SYNC_STATE sync_state;
};

static int _in_que_flag = 0;
static struct adc_data_info ads1256_data_info;


static uint8_t rate2regdata(int rate)
{
    uint8_t ret = 0xFF;
    switch(rate)
    {
    case 30000:
        ret = DRATE_30000;
        break;
    case 15000:
        ret = DRATE_15000;
        break;
    case 7500:
        ret = DRATE_7500 ;
        break;
    case 3750:
        ret = DRATE_3750;
        break;
    case 2000:
        ret = DRATE_2000;
        break;
    case 1000:
        ret = DRATE_1000;
        break;
    case 500:
        ret = DRATE_500;
        break;
    case 100:
        ret = DRATE_100;
        break;
    case 60:
        ret = DRATE_60;
        break;
    case 50:
        ret = DRATE_50 ;
        break;
    case 30:
        ret = DRATE_30;
        break;
    case 25:
        ret = DRATE_25;
        break;
    case 15:
        ret = DRATE_15;
        break;
    case 10:
        ret = DRATE_10;
        break;
    case 5:
        ret = DRATE_5;
        break;
//    	  case 2d5:////这个特别注意，暂时不支持
//	  	ret = DRATE_2d5;
//		break;
    default:
        break;
    }

    return ret;
}

static inline void adc_sleep(unsigned sec)
{
    __set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(sec * HZ);
}

//adc_msleep: schedule_timeout at least delay 1/HZ s (10 ms)
static inline void adc_msleep(unsigned ms)
{
    __set_current_state(TASK_INTERRUPTIBLE);

    if (ms < 10)
    {
        ms = 10;
    }
    schedule_timeout(ms * HZ / 1000);
}

static int request_gpio(void)
{
    int error=0;
    unsigned int gpio_idx ;
    int i;
    for (i = 0; i < N_GPIO; i++)
    {
        gpio_idx = IMX_GPIO_NR(gpio_array[i].grp_idx, gpio_array[i].entry_idx);
        if (gpio_is_valid(gpio_idx))
        {
            error = gpio_request_one(gpio_idx, gpio_array[i].init_value, "input gpio out dev");
            if (error < 0)
            {
                printk("Failed to request GPIO %d(i=%d), error %d\n", gpio_idx,i, error);
                return error;
            }
        }
    }
    return error;
}

static void free_gpio(void)
{
    int i;
    for (i = 0; i < N_GPIO; i++)
    {
        gpio_free(IMX_GPIO_NR(gpio_array[i].grp_idx, gpio_array[i].entry_idx));
    }
}

static int register_ads1256_drdy_irq(void)
{
    int ret,i;

    for(i=0; i<GI_COUNT; i++)
    {
        adc_ready_irq_map[i].irq = gpio_to_irq(IMX_GPIO_NR(adc_ready_irq_map[i].grp_index,adc_ready_irq_map[i].entry_index));

        /*gpio request and config*/
        if(gpio_is_valid(IMX_GPIO_NR(adc_ready_irq_map[i].grp_index,adc_ready_irq_map[i].entry_index)))
        {
            gpio_request_one(IMX_GPIO_NR(adc_ready_irq_map[i].grp_index,adc_ready_irq_map[i].entry_index), GPIOF_DIR_IN, adc_ready_irq_map[i].name);
        }
        /*irq request*/
        ret = request_irq(adc_ready_irq_map[i].irq, adc_ready_irq_map[i].irq_handler, adc_ready_irq_map[i].flags, adc_ready_irq_map[i].name, &gi_triggered[i]);
        if (ret)
        {
            return ret;
        }
    }

    return 0;
}

static void fpga_power_on(void)
{
    MDEBUG("FPGA POWER ON.\n");
    gpio_set_value(fpga_pow_en, 1);
}

static void fpga_power_off(void)
{
    MDEBUG("FPGA POWER OFF.\n");
    gpio_set_value(fpga_pow_en, 0);
}


static void unregister_ads1256_drdy_irq(void)
{
    int i;

    MDEBUG("unregistering ads1256_drdy_irq...\n");
    for(i=0; i<GI_COUNT; i++)
    {
        gpio_free(IMX_GPIO_NR(adc_ready_irq_map[i].grp_index,adc_ready_irq_map[i].entry_index));
        free_irq(adc_ready_irq_map[i].irq, &gi_triggered[i]);
    }
    MDEBUG("unregister ads1256_drdy_irq ok\n");
}

static int wait_dready(uint32_t jiffies)
{
    int ret;
    if(jiffies > 10 * HZ)
    {
        MWARN("too large timeout value, > 10s !\n");
        return -1;
    }
    ret = wait_event_timeout(gi_ready_wait, ready_flag, jiffies); // jiffies / HZ = s
    // wait_event_interruptible(gi_ready_wait, ready_flag); // jiffies / HZ = s
    ready_flag = 0;
    return 0;
}

void adc_callback(void)
{
#if 0
    ads1256_spi_async_read();
#endif
}

static uint64_t get_timestamp(void)
{
    struct timeval now;
    uint64_t ts;
    do_gettimeofday(&now);
    ts = now.tv_sec;
    ts *= 1000000;
    ts += now.tv_usec;
    return ts;
}

#include <linux/random.h>

uint8_t tempbuf[1200];
char strbuf[1000];
bool tempbuf_en = false;
static int adc_in_queue(void)
{
    int32_t retval=0,i,j;
    AdcData data,test;
    AdcOrgData *org = NULL;
    bool sync = ads1256_data_info.time_sync;//记录上次同步状态

    if(!fpga_spi_status())
    {
        MWARN(" ads1256 devs not prepared\n");
        return  -1;
    }
    /*
    mutex_lock(&adc_queue_lock);
    int q_full = queue_full(ads1256_queue);
    mutex_unlock(&adc_queue_lock);
    if(q_full)
    {
        retval =  -1;
        break;
    }*/

    data.timestamp = get_timestamp();

    org = (AdcOrgData *)kmalloc(ADC_DATAPACK_COUNT*sizeof(AdcOrgData), GFP_KERNEL);
    retval = fpga_spi_read((uint8_t *)org,ADC_DATAPACK_COUNT*sizeof(AdcOrgData));
    ads1256_data_info.read_stamp = get_timestamp();
    ads1256_data_info.read_count++;

    for(i = 0; i < ADC_DATAPACK_COUNT; i++)
    {
        uint32_t* tmpbuf =  (uint32_t*)data.data;
        uint8_t id;
        //数据处理
        tmpbuf[0] = ((uint32_t)org[i].datax[0] << 16) & 0x00FF0000;
        tmpbuf[0] |= ((uint32_t)org[i].datax[1] << 8);
        tmpbuf[0] |= (uint32_t)org[i].datax[2] ;
        if (tmpbuf[0] & 0x800000)
        {
            tmpbuf[0] |= 0xFF000000;
        }
        tmpbuf[1] = ((int32_t)org[i].datay[0] << 16) & 0x00FF0000;
        tmpbuf[1] |= ((int32_t)org[i].datay[1] << 8);
        tmpbuf[1] |= (int32_t)org[i].datay[2] ;
        if (tmpbuf[1] & 0x800000)
        {
            tmpbuf[1] |= 0xFF000000;
        }
        tmpbuf[2] = ((int32_t)org[i].dataz[0] << 16) & 0x00FF0000;
        tmpbuf[2] |= ((int32_t)org[i].dataz[1] << 8);
        tmpbuf[2] |= (int32_t)org[i].dataz[2] ;
        if (tmpbuf[2] & 0x800000)
        {
            tmpbuf[2] |= 0xFF000000;
        }
        //测试用
#if 0
        if(org[i].flagx&0x80)
        {
            data.data[2] = 1000000;
        }
        else
        {
            data.data[2] = 0;
        }
#endif

        data.info.linfo_ = 0;
        data.gps_sig = ads1256_data_info.gps_signal;

        //时间处理
        ads1256_data_info.stamp+=ads1256_data_info.sample_intern;
        ads1256_data_info.totl_count++;
        ads1256_data_info.after_pulse_count++;//上一个秒边缘到此的点数
        data.timestamp = ads1256_data_info.stamp;//每一个数据时间是上个数据时间累计的结果
        data.timestamp_sys = ads1256_data_info.dready_stamp+(i-100)*ads1256_data_info.sample_intern;		//数据的系统时间是以这包dready系统时间加上偏移的结果
        if((!(ads1256_data_info.pre_flag&0x80))&& (org[i].flagx&0x80))
        {
            int pulse_cap;
            data.info.sinfo_.is_edge_ = 1;
            ads1256_data_info.pre_data_pulse_stamp+=1000000;
            pulse_cap = (int)(ads1256_data_info.pre_data_pulse_stamp-ads1256_data_info.pre_gps_pulse_stamp);
            if(!ads1256_data_info.time_sync)
            {
                if(ads1256_data_info.pre_gps_pulse_stamp>1509593270705977) //限制条件
                {
                    ads1256_data_info.pre_data_pulse_stamp = ads1256_data_info.stamp = ads1256_data_info.pre_gps_pulse_stamp;
                    ads1256_data_info.time_sync = true;
                    printk("time sync ok, stamp:%lld\n",ads1256_data_info.stamp);
                }
                else
                {
                    printk("time sync error, pulse stamp:%lld\n",ads1256_data_info.pre_gps_pulse_stamp);
                }
            }
            else if((abs(pulse_cap)>500000)&&ads1256_data_info.gpsok)
            {
                printk("===pulse time error cap=%d,my:%lld,gps:%lld\n",pulse_cap,ads1256_data_info.pre_data_pulse_stamp,ads1256_data_info.pre_gps_pulse_stamp);
                if(ads1256_data_info.pre_gps_pulse_stamp>1509593270705977) //限制条件
                {
                    ads1256_data_info.pre_data_pulse_stamp = ads1256_data_info.stamp = ads1256_data_info.pre_gps_pulse_stamp;
                    ads1256_data_info.time_sync = true;
                    printk("==time sync ok, stamp:%lld\n",ads1256_data_info.stamp);
                }
                //else
                //{
                //	ads1256_data_info.pre_data_pulse_stamp+=1000;
                //     printk("==time sync error, pulse stamp:%lld\n",ads1256_data_info.pre_gps_pulse_stamp);
                // }
            }
            //// else
            //  {
            //      ads1256_data_info.pre_data_pulse_stamp+=1000;
            //   }
            data.timestamp = ads1256_data_info.pre_data_pulse_stamp;//如果发现数据中有秒边缘,则直接更新这个数据点时间为下一秒时间

            if(ads1256_data_info.after_pulse_count>(ads1256_data_info.sample_rate+1)||ads1256_data_info.after_pulse_count<(ads1256_data_info.sample_rate-1))
            {
                //可能是有问题了，两个边缘秒脉冲不是1000点，怎么处理?
                //printk("after_pulse_count:%d\n",ads1256_data_info.after_pulse_count);
                ///ads1256_data_info.gpsok = 0;//??????
                data.info.sinfo_.data_count_err_ = 1;
            }

            //判定计算的上次数据秒边缘时间与GPS秒边缘时间是否一致
            if(ads1256_data_info.pre_data_pulse_stamp!=ads1256_data_info.pre_gps_pulse_stamp)
            {
                //printk("pre_data_pulse_stamp(%lld)!=pre_gps_pulse_stamp(%lld)\n",ads1256_data_info.pre_data_pulse_stamp,ads1256_data_info.pre_gps_pulse_stamp);
            }
            ads1256_data_info.after_pulse_count = 0;
        }
        if(0==i)
        {
            test = data;
        }
        id = org[i].flagx&0x7F;

        if(ads1256_data_info.totl_count>1)
        {
            uint8_t cap = (id+128-ads1256_data_info.pre_id)&0x7F;
            if(cap!=1) //应该出现了丢包和丢点情况
            {
                //printk("adc data id lost,pre id:%d,cur id:%d,cap:%d\n",ads1256_data_info.pre_id,id,cap);
                ads1256_data_info.err_count++;
                data.info.sinfo_.lost_err_ = 1;
            }
        }
        if(((org[i].flagy&0x7F)!=id)||((org[i].flagz&0x7F)!=id))
        {
            ads1256_data_info.err2_count++;
            if(!(ads1256_data_info.err2_count%1000))
            {
                printk("adc index error: x:%d, y:%d, z:%d\n",org[i].flagx&0x7F,org[i].flagy&0x7F,org[i].flagz&0x7F);
            }
            data.info.sinfo_.index_err_ = 1;
        }
        data.info.sinfo_.adjusted_ = (ads1256_data_info.sync_state==SYNC_PULSE_OK)&&ads1256_data_info.time_sync;
        data.info.sinfo_.data_valid_ = data.info.sinfo_.adjusted_&&(!data.info.sinfo_.index_err_)&&(!data.info.sinfo_.lost_err_)&&(!data.info.sinfo_.data_count_err_);
        data.info.sinfo_.sync_state_ = (uint8_t)ads1256_data_info.sync_state;
        //printk("ad data id%d,pulse%d,x:%d,y:%d,z:%d\n",id,org[i].flagx&0x80,data.data[0],data.data[1],data.data[2]);
        ads1256_data_info.pre_id = id ;
        ads1256_data_info.pre_flag = org[i].flagx ;
        if(sync) //只有在之前包里同步过，才能输出
        {
            ads1256_data_info.adc_pingpong_data[ads1256_data_info.write_idx][i] = data;
        }
    }

    if(sync) //只有在之前包里同步过，才能输出
    {
        ads1256_data_info.write_idx = (ads1256_data_info.write_idx+1)%DATA_PACK_COUNT;
        if(ads1256_data_info.full_count<DATA_PACK_COUNT)
        {
            ads1256_data_info.full_count++;
        }
    }
#if 0
    if(!(ads1256_data_info.read_count%10))
    {
        printk("sync:%d,signal:%d,%lld----ads1256,pulse tick:%lld,read tick %lld,data tick:%lld,data sys tick:%lld,dly:%d,cap:%d(isr:%d,read:%d,err:%d,err2:%d)(seq=%02x,x=%02x %02x %02x (%d),y=%02x %02x %02x (%d),z=%02x %02x %02x  (%d))\n",
               sync,test.gps_sig,get_timestamp(),test.timestamp,test.timestamp_sys,
               ads1256_data_info.dready_stamp,ads1256_data_info.read_stamp,
               (int)(ads1256_data_info.read_stamp-ads1256_data_info.dready_stamp),
               (int)(ads1256_data_info.dready_stamp-ads1256_data_info.pre_dready_stamp),
               ads1256_data_info.dready_count,ads1256_data_info.read_count,
               ads1256_data_info.err_count,ads1256_data_info.err2_count,
               org[0].flagx,org[0].datax[0],org[0].datax[1],org[0].datax[2],test.data[0],
               org[0].datay[0],org[0].datay[1],org[0].datay[2],test.data[1],
               org[0].dataz[0],org[0].dataz[1],org[0].dataz[2],test.data[2]);

    }
#endif

    ads1256_data_info.pre_dready_stamp = ads1256_data_info.dready_stamp;
    kfree(org);

    return retval;
}

static irqreturn_t ads1256_drdy_isr(int irq,void *dev_id)
{
    if(_adc_started &&(fpga_spi_status()>0))
    {
        volatile int *happend = (volatile int *)dev_id;
        *happend = *happend + 1;
        _adc_irq_sum++;
        ads1256_data_info.dready_stamp = get_timestamp();
        ads1256_data_info.dready_count++;
        adc_callback();
        wake_up(&in_queue_wait);
        _in_que_flag = 1;
        //printk(" _in_que_flag = 1\n");
    }
    ready_flag = 1;
    //printk(" ready_flag = 1,_adc_started:%d,fpga_spi_status:%d\n",_adc_started,fpga_spi_status());
    wake_up(&gi_ready_wait);


    kill_fasync(&fasync, SIGIO, POLL_IN);
    return IRQ_HANDLED;
}

#if 1
static struct task_struct * Adc_Thread = NULL;
static int proc_cap_max = 0;
static int adc_th_func(void *arg)
{
    printk("adc_th_func start.\n");
    do
    {
        long ret =  wait_event_interruptible_timeout(in_queue_wait, _in_que_flag,2 * HZ);
        if((ret!=-ERESTARTSYS)&&(ret!=0))
        {
            int64_t tick0 = get_timestamp();
            int cap = 0;
            //printk("adc_th_func ready\n");
            adc_in_queue();
            _in_que_flag = 0;
            _adc_read_sum++;
            cap = (int)(get_timestamp()-ads1256_data_info.dready_stamp);
            if((cap>proc_cap_max)||(cap>50000))
            {
                proc_cap_max = cap;
                printk("cap:%d,proc_cap_max:%d,dreadycount:%d,read count:%d\n",cap,proc_cap_max,_adc_irq_sum,_adc_read_sum);
            }

        }
    }
    while(!kthread_should_stop());

    return 0;
}
#endif



#if 1
static struct workqueue_struct *adc_read_wq;
struct work_struct adc_read_work;
static void adc_read_worker(struct work_struct *work)
{
    while(1)
    {
        wait_event_interruptible(in_queue_wait, _in_que_flag);
        adc_in_queue();
        _in_que_flag = 0;
        _adc_read_sum++;
    }
}
#endif


static struct ads1256_spi_config spi_config_def =
{
    .speed_hz = 1000000,
    .bpw = 8,
    .mode = 1,
//	.cs =
};



static void xque_init(void)
{
#if 1
    MDEBUG("kthread_create ..\n");

    Adc_Thread = kthread_run(adc_th_func,NULL,"adc_kthread");
    if(IS_ERR(Adc_Thread))
    {
        MWARN("kthread_create error\n");
    }
    else
    {
        MDEBUG("kthread_create ok\n");
    }
#endif

#if  0
    adc_read_wq = create_workqueue("qat_adc_read_wq");
    if(	adc_read_wq )
    {
        INIT_WORK(&adc_read_work, adc_read_worker);
        queue_work(adc_read_wq, &adc_read_work);
    }
#endif
}

static void xque_del(void)
{
#if 1
    if (!IS_ERR(Adc_Thread))
    {
        MDEBUG("Stop... Kernel Thread\n");
        kthread_stop(Adc_Thread);
        MDEBUG("Stop Kernel Thread OK\n");
    }
#endif

#if  0
    if (adc_read_wq)
        destroy_workqueue(adc_read_wq);
    adc_read_wq = NULL;
#endif
}



static int waitForAdcReady(void)
{
    uint32_t jiffies = 2*HZ;
    adc_msleep(200);
    return wait_dready(jiffies);
}


//0xAA 0xAA 0xAA  0xAA ch cmd 0xAF
static void send_adc_cmd(uint8_t adid,uint8_t *buf, size_t count)
{
    int pos = 0,i;
    uint8_t cmds[32];

    cmds[pos++] = 0xAA;
    cmds[pos++] = 0xAA;
    cmds[pos++] = 0xAA;
    cmds[pos++] = 0xAA;
    cmds[pos++] = adid;

    for(i = 0; i < count; i++)
    {
        cmds[pos++] = buf[i];
    }

    cmds[pos++] = 0xAF;

#if 0
    printk("send_adc_cmd:");
    for(i = 0; i < pos; i++)
    {
        printk("\t%02x",cmds[i]);
    }
    printk("\n");
#endif
    fpga_spi_write(cmds, pos);
}


static int read_adc_reg_cmd(uint8_t adid,uint8_t  *buf_in,
                            size_t byte_in,
                            uint8_t *buf_out,
                            size_t byte_out)
{
    int ret;
    send_adc_cmd(adid,buf_in,byte_in);
    ret = waitForAdcReady();
    if(ret!=0)
    {
        printk("read_adc_reg_cmd wait err :%d\n",ret);
        return -1;
    }
    return fpga_spi_read(buf_out,byte_out);
}

static void writeByteToReg(uint8_t adid, uint8_t register_id, uint8_t value)
{
    uint8_t cmds[3];
    cmds[0] = CMD_WREG | register_id;
    cmds[1] = 0x00;
    cmds[2] = value;
    //fpga_spi_write(cmds, 3);
    send_adc_cmd(adid,cmds,3);
}

static void adc_reset_hw(void)
{
#if 1
    MDEBUG("adc reseting....\n");
    gpio_set_value(adc_reset, 1);
    adc_msleep(20);
    gpio_set_value(adc_reset, 0);
    adc_msleep(100);
    gpio_set_value(adc_reset, 1);
//   adc_msleep(100);
    MDEBUG("adc resset ok\n");
#endif
}

static void fpga_reset_hw(void)
{
#if 1
    MDEBUG("fpga reseting....\n");
    //gpio_set_value(fpga_reset, 1);
    //adc_msleep(20);
    gpio_set_value(fpga_reset, 0);
    adc_msleep(1000);
    gpio_set_value(fpga_reset, 1);
    adc_msleep(1000);
    MDEBUG("fpga resset ok\n");
#endif
}

//adc同步允许
static void adc_sync_hw(void)
{
    int count = 0;
    gpio_set_value(fpga_sync, 0);
    adc_msleep(200);
    MDEBUG("adc syncing....\n");
    _adc_started = 1;
    gpio_set_value(fpga_sync, 1);
    ads1256_data_info.sync_state = SYNC_SYNCING;
    while(SYNC_SYNCING==ads1256_data_info.sync_state)
    {
        adc_msleep(10);
        count++;
        if(count>=150)
        {
            ads1256_data_info.sync_state = SYNC_TIMEOUT;
            MWARN("adc_sync_hw timeout!!!\n");
            break;
        }
    }
    adc_msleep(10);
    gpio_set_value(fpga_sync, 0);
    if(SYNC_PULSE_OK==ads1256_data_info.sync_state)
    {
        MDEBUG("adc sync ok.gps_pulse_stamp:%lld\n",ads1256_data_info.pre_gps_pulse_stamp);
    }
}

static void SyncStartReadC(void)
{
    uint8_t cmd = CMD_RDATAC;
    int ret;
    MDEBUG("adc Sync Starting ReadC...\n");
    send_adc_cmd(ADC_CH_ALL,&cmd, 1);

    mutex_lock(&adc_queue_lock);
    ads1256_data_info.read_idx = 0;
    ads1256_data_info.write_idx = 0;
    ads1256_data_info.write_pos = 0;
    ads1256_data_info.full_count = 0;
    ads1256_data_info.stamp = 0;
    ads1256_data_info.pre_gps_pulse_stamp = 0;
    ads1256_data_info.pre_data_pulse_stamp = 0;
    ads1256_data_info.dready_stamp = 0;
    ads1256_data_info.totl_count = 0;
    ads1256_data_info.err_count = 0;
    ads1256_data_info.err2_count = 0;
    ads1256_data_info.read_count = 0;
    ads1256_data_info.usr_read_count = 0;
    ads1256_data_info.dready_count = 0;
    ads1256_data_info.after_pulse_count = 0;
    ads1256_data_info.gpsok = 0;
    ads1256_data_info.gps_signal = 0;
    ads1256_data_info.ad_init = true;
    ads1256_data_info.time_sync = false;
    ads1256_data_info.pre_flag = -1;
    ads1256_data_info.sync_state = SYNC_NULL;
    mutex_unlock(&adc_queue_lock);

    //adc_sleep(1);
    adc_sync_hw();
    //adc_sleep(1);

    ret = waitForAdcReady();
    if(ret!=0)
    {
        MWARN(" ADC_START_DATA_STREAM TIMEROUT\n");
        return;
    }
    MDEBUG(" ADC_START_DATA_STREAM OK\n");
    adc_sleep(3);
}

static void StopReadC(void)
{
    uint8_t cmd = CMD_SDATAC;
    // Stop continuous mode.
    int ret =  waitForAdcReady();
    if(ret)
    {
        MWARN(" StopReadC  TIMEROUT\n");
    }

    mutex_lock(&adc_queue_lock);
    ads1256_data_info.read_idx = 0;
    ads1256_data_info.write_idx = 0;
    ads1256_data_info.write_pos = 0;
    ads1256_data_info.full_count = 0;
    ads1256_data_info.stamp = 0;
    ads1256_data_info.pre_gps_pulse_stamp = 0;
    ads1256_data_info.pre_data_pulse_stamp = 0;
    ads1256_data_info.dready_stamp = 0;
    ads1256_data_info.totl_count = 0;
    ads1256_data_info.err_count = 0;
    ads1256_data_info.err2_count = 0;
    ads1256_data_info.read_count = 0;
    ads1256_data_info.usr_read_count = 0;
    ads1256_data_info.dready_count = 0;
    ads1256_data_info.after_pulse_count = 0;
    ads1256_data_info.ad_init = false;
    ads1256_data_info.time_sync = false;
    ads1256_data_info.gpsok = 0;
    ads1256_data_info.gps_signal = 0;
    ads1256_data_info.pre_flag = -1;
    ads1256_data_info.sync_state = SYNC_NULL;
    _adc_started = 0;
    mutex_unlock(&adc_queue_lock);

    adc_msleep(1000);

    //fpga_spi_write( &cmd, 1);
    send_adc_cmd(ADC_CH_ALL,&cmd, 1);
    adc_msleep(10);
    send_adc_cmd(ADC_CH_ALL,&cmd, 1);//多发一次，确保成功
}

static void SyncChannels(void)
{
    adc_sync_hw();
    adc_msleep(SET_REG_MDELAY);
}

static int readRegs(uint8_t adid, uint8_t start, uint8_t* outbuf, uint8_t len)
{

    uint8_t cmds[2];

    if(outbuf == NULL)
    {
        MDEBUG("outbuf ptr cannot be null !\n");
        return -2;
    }

    cmds[0] = CMD_RREG | start;
    cmds[1] = len-1;// bytes_to_read - 1;
    return read_adc_reg_cmd(adid,cmds, 2, outbuf, len);
}

static int readAllRegs(uint8_t adid, uint8_t start, uint8_t* outbuf, uint8_t len)
{

    uint8_t cmds[2];

    if(outbuf == NULL)
    {
        MDEBUG("outbuf ptr cannot be null !\n");
        return -2;
    }

    cmds[0] = CMD_RREG | start;
    cmds[1] = 0;// bytes_to_read - 1;
    return read_adc_reg_cmd(adid,cmds, 2, outbuf, len);
}

static int32_t setBuffer(uint8_t adid, bool val)
{

    uint8_t cmd[3];

    cmd[0]= CMD_WREG | REG_STATUS;
    cmd[1] = 0x00;
    cmd[2]= (0 << 3) | (1 << 2) | (val << 1);

// fpga_spi_write(cmd, 2);
    send_adc_cmd(adid,cmd,3);
    adc_msleep(SET_REG_MDELAY);
    return 0;
}
static int32_t setBuffers(bool val)
{
    return setBuffer(ADC_CH_ALL,val);
}
static int32_t setPGA(uint8_t adid, uint8_t pga)
{
    uint8_t cmds[3];
    cmds[0] = CMD_WREG | REG_ADCON;

    cmds[1] = 0x00;
    cmds[2] = pga;

    send_adc_cmd(adid,cmds, 3);
    adc_msleep(SET_REG_MDELAY);
    return 0;
}

static int32_t setPGAs(uint8_t pga)
{
    return setPGA(ADC_CH_ALL,pga);
}

static int32_t setSampleRate(uint8_t adid,uint32_t v)
{
    uint8_t rate = rate2regdata(v);
    uint8_t cmds[4]= {CMD_WREG|REG_DRATE,0x01,rate,0x00};

    if(rate!=0xFF)
    {
        send_adc_cmd(adid,cmds, 4);
        adc_msleep(SET_REG_MDELAY);
    }
    else
    {
        MWARN("setSampleRate:rate error %d",v);
        return -1;
    }
    return 0;
}

static int32_t setSampleRates( uint32_t v)
{
    return setSampleRate(ADC_CH_ALL,v);
}

static uint8_t readSampleRateReg(uint8_t adid)
{
    uint8_t cmds[2];
    uint8_t rx;

    cmds[0] = CMD_RREG | REG_DRATE;
    cmds[1] = 0x00;// bytes_to_read - 1;
    read_adc_reg_cmd(adid,cmds, 2, &rx, 1);

    return rx;
}

static uint8_t readAllSampleRateReg(uint8_t *regs)
{
    return readAllRegs(ADC_CH_ALL,REG_DRATE,regs,3);
}

static uint8_t readStatusReg(uint8_t adid)
{
    uint8_t cmds[2];
    uint8_t rx;
    int ret;

    cmds[0] = CMD_RREG | REG_STATUS;
    cmds[1] = 0x00;// bytes_to_read - 1;

    ret = read_adc_reg_cmd(adid,cmds, 2, &rx, 1);
    //printk("readStatusReg ad%d:%02x,ret=%d\n ",rx,ret);
    return rx;
}

static int readAllStatusReg(uint8_t *regs)
{
    return readAllRegs(ADC_CH_ALL,REG_STATUS,regs,3);
}

static int32_t setDiffChannel(uint8_t adid,uint8_t val)
{

    uint8_t cmd[2];

    cmd[0]= CMD_WREG | REG_MUX;
    cmd[1] = 0x00;
    cmd[2]= val;

    send_adc_cmd(adid,cmd,3);
    adc_msleep(SET_REG_MDELAY);

    return 0;
}

static int32_t setDiffChannels(uint8_t val)
{
    return setDiffChannel(ADC_CH_ALL,val);
}

static int readAllDiffReg(uint8_t *regs)
{
    return readAllRegs(ADC_CH_ALL,REG_MUX,regs,3);
}

static int32_t setSelfCal(uint8_t adid)
{
    uint8_t cmd = CMD_SELFCAL;
    int ret;

    send_adc_cmd(adid,&cmd,1);
    ret =  waitForAdcReady();
    if(ret!=0)
    {
        MWARN("adc setSelfCal %d TIMEROUT\n",adid);
    }
    MDEBUG("adc setSelfCal %d ok.\n",adid);
    return ret;
}

static int32_t setSelfCals(void)
{
    return setSelfCal(ADC_CH_ALL);
}



static int32_t readOFC(uint8_t adid)
{

    uint8_t cmds[2];
    uint8_t rx[3];
    int32_t readv;

    cmds[0] = CMD_RREG | REG_OFC0; //05h
    cmds[1] = 0x02;// bytes_to_read - 1;
    read_adc_reg_cmd(adid,cmds, 2, rx, 3);

    /*
    OFC is a Binary Two Complement number that can range from -8388608 to 8388607
    */
    readv = rx[2];
    readv = (readv<<8) + rx[1];
    readv = (readv<<8)+ rx[0];

    if((rx[2]&0x80) != 0)
    {
        readv |= 0xff000000;
    }

    return readv;
}

static int32_t readAllOFC(int32_t *ofc)
{
    uint8_t cmds[2];
    uint8_t rx[9];
    int32_t readv = 0,i;
    uint32_t *tmpbuf = (uint32_t *)ofc;

    cmds[0] = CMD_RREG | REG_OFC0; //05h
    cmds[1] = 0x02;// bytes_to_read - 1;
    read_adc_reg_cmd(ADC_CH_ALL,cmds, 2, rx, 9);

    /*
    OFC is a Binary Two Complement number that can range from -8388608 to 8388607
    */
    MDEBUG("adc ofc org data:");
    for(i = 0; i < 3; i++)
    {
        tmpbuf[i] = ((uint32_t)rx[6+i] << 16) & 0x00FF0000;
        tmpbuf[i] |= ((uint32_t)rx[3+i] << 8);
        tmpbuf[i] |= (uint32_t) rx[i];
        if (tmpbuf[i] & 0x800000)
        {
            tmpbuf[i] |= 0xFF000000;
        }
        printk("%02x %02x %02x | ",rx[i], rx[3+i],rx[6+i]);
    }
    printk("\n");

    return readv;
}


static int readFSC(uint8_t adid)
{
    uint8_t cmds[2];
    uint8_t rx[3];
    int32_t readv;

    cmds[0] = CMD_RREG | REG_FSC0; //08h
    cmds[1] = 0x02;// bytes_to_read - 1;
    read_adc_reg_cmd(adid,cmds, 2, rx, 3); //OKOK

    readv = ((int32_t)rx[2] << 16) & 0x00FF0000;
    readv |= ((int32_t)rx[1] << 8);
    readv |= rx[0];

    return readv;
}

static int32_t readAllFSC(int32_t *fsc)
{
    uint8_t cmds[2];
    uint8_t rx[9];
    int32_t readv = 0,i;
    uint32_t *tmpbuf = (uint32_t *)fsc;

    cmds[0] = CMD_RREG | REG_FSC0; //05h
    cmds[1] = 0x02;// bytes_to_read - 1;
    read_adc_reg_cmd(ADC_CH_ALL,cmds, 2, rx, 9);
    MDEBUG("adc fsc org data:");
    /*
    OFC is a Binary Two Complement number that can range from -8388608 to 8388607
    */
    for(i = 0; i < 3; i++)
    {
        tmpbuf[i] = ((uint32_t)rx[6+i] << 16) & 0x00FF0000;
        tmpbuf[i] |= ((uint32_t)rx[3+i] << 8);
        tmpbuf[i] |= (uint32_t) rx[i];
        if (tmpbuf[i] & 0x800000)
        {
            tmpbuf[i] |= 0xFF000000;
        }
        printk("%02x %02x %02x | ",rx[i], rx[3+i],rx[6+i]);
    }
    printk("\n");

    return readv;
}

static int32_t readAllOneData(uint32_t *tmpbuf)
{
    uint8_t cmds[2];
    uint8_t rx[9];
    int32_t readv = 0,i;

    cmds[0] = 0x01; //05h
    read_adc_reg_cmd(ADC_CH_ALL,cmds, 1, rx, 9);

    /*
    OFC is a Binary Two Complement number that can range from -8388608 to 8388607
    */
    for(i = 0; i < 3; i++)
    {
        tmpbuf[i] = ((uint32_t)rx[3*i] << 16) & 0x00FF0000;
        tmpbuf[i] |= ((uint32_t)rx[3*i+1] << 8);
        tmpbuf[i] |= (uint32_t) rx[3*i+2];
        if (tmpbuf[i] & 0x800000)
        {
            tmpbuf[i] |= 0xFF000000;
        }
    }

    return readv;
}

static uint8_t setTestData(void)
{
    uint8_t cmds[]= {0xAA,0xAA,0xAA,0xAA,0x07,0xFF,0xAF};

    fpga_spi_write(cmds, sizeof(cmds));
    adc_msleep(SET_REG_MDELAY);

    return 0;
}


static int initChipsForSample(uint32_t dRate)
{
    int i,ret = 0;
    uint8_t regs[3] = {0xFF,0xFF,0xFF};
    uint8_t positiveCh=0;
    uint8_t negativeCh=1;
    uint32_t dat[3];
    uint8_t rate;

    //adc_reset_hw();
    fpga_reset_hw();
    gpio_set_value(fpga_sync, 0);

#if 1
    MDEBUG("adc set status register\n");
    setBuffers(0);


    ret = readAllStatusReg( regs);
    MDEBUG("adc read status reg(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);

    //setPGAs(PGA_GAIN1); //榛樿灏辨槸 PGA=1

    MDEBUG("adc set sample_rate register\n");
    rate = rate2regdata(dRate);
    if(0xFF==rate)
    {
        MWARN("sample rate %d error!!,use default.\n");
        dRate = SAMPLE_RATE_DEF;
    }
    setSampleRates(dRate);
    ads1256_data_info.sample_rate = dRate;
    ads1256_data_info.sample_intern = 1000000/dRate;
    MDEBUG("adc sample intern:%d\n",ads1256_data_info.sample_intern);

    ret = readAllSampleRateReg(regs);
    MDEBUG("adc read sample_rate reg(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);

    MDEBUG("adc set DiffChannels reg\n");
    setDiffChannels(positiveCh << 4 | negativeCh);
    ret = readAllDiffReg(regs);
    printk("Diff(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);

    MDEBUG("adc setSelfCals\n");
    setSelfCals();
    printk("initChipsForSample ok\n");

    readAllOFC(ads1256_data_info.ofc);
    MDEBUG("adc read ofc result:%d,%d,%d\n",ads1256_data_info.ofc[0],ads1256_data_info.ofc[1],ads1256_data_info.ofc[2]);
    readAllFSC(ads1256_data_info.fsc);
    MDEBUG("adc read fsc result:%d,%d,%d\n",ads1256_data_info.fsc[0],ads1256_data_info.fsc[1],ads1256_data_info.fsc[2]);
#endif

    return 0;
}

static int  gi_ads1256_callback(bool valid,int32_t signal,uint64_t tick)
{
    if(tick>=1509593270705977)
    {
#if 0
        int cap = (int)(tick-ads1256_data_info.pre_gps_pulse_stamp);
        if((ads1256_data_info.sync_state==SYNC_PULSE_OK)
                &&((cap<800000)||(cap>1200000))) {
            printk("pre_gps_pulse_stamp=%lld(%lld),cap:%d\n",ads1256_data_info.pre_gps_pulse_stamp,get_timestamp(),cap);
        }
#endif
        ads1256_data_info.pre_gps_pulse_stamp = tick;
        if(SYNC_SYNCING==ads1256_data_info.sync_state)
        {
            ads1256_data_info.sync_state = SYNC_PULSE_OK;
        }
    }
    else if(ads1256_data_info.sync_state>SYNC_SYNCING) //无信号超时时候这样做
    {
        ads1256_data_info.pre_gps_pulse_stamp = get_timestamp();//如果传递进来的时间是无效时间，直接使用系统时间
    }
    if(signal>=3)
    {
        ads1256_data_info.gpsok = 1;
    }
    else
    {
        ads1256_data_info.gpsok = 0;
    }
    ads1256_data_info.gps_signal = signal;

    return 0;
}

static void init_data_info(void)
{
    ads1256_data_info.read_idx = 0;
    ads1256_data_info.write_idx = 0;
    ads1256_data_info.write_pos = 0;
    ads1256_data_info.full_count = 0;
    ads1256_data_info.stamp = 0;
    ads1256_data_info.pre_gps_pulse_stamp = 0;
    ads1256_data_info.pre_data_pulse_stamp = 0;
    ads1256_data_info.dready_stamp = 0;
    ads1256_data_info.totl_count = 0;
    ads1256_data_info.err_count = 0;
    ads1256_data_info.err2_count = 0;
    ads1256_data_info.read_count = 0;
    ads1256_data_info.usr_read_count = 0;
    ads1256_data_info.dready_count = 0;
    ads1256_data_info.ad_init = false;
    ads1256_data_info.time_sync = false;
    ads1256_data_info.after_pulse_count = 0;
    ads1256_data_info.gpsok = 0;
    ads1256_data_info.gps_signal = 0;
    ads1256_data_info.need_cabli = false;
    ads1256_data_info.sync_state = SYNC_NULL;
}


static int init_spi_param(void)
{
    int ret =  fpga_spi_cfg(SPI_CMD_OPEN,0);
    if(ret!=0)
    {
        MWARN("mSpiFd open error%d\n",ret);
        return -1;
    }
    else
    {
        printk("mSpiFd open ok\n");
    }

    MDEBUG("spi mode: 0x%x\n", mSpiMode);
    fpga_spi_cfg(SPI_IOC_WR_MODE, (unsigned long)(&mSpiMode));
    fpga_spi_cfg(SPI_IOC_RD_MODE, (unsigned long)(&mSpiMode));
    MDEBUG("bits per word: %d\n", mSpiBits);
    fpga_spi_cfg(SPI_IOC_WR_BITS_PER_WORD, (unsigned long)(&mSpiBits));
    fpga_spi_cfg(SPI_IOC_RD_BITS_PER_WORD, (unsigned long)(&mSpiBits));
    MDEBUG("max speed: %d Hz (%d KHz)\n", mSpiSpeed, mSpiSpeed/1000);
    fpga_spi_cfg(SPI_IOC_WR_MAX_SPEED_HZ, (unsigned long)(&mSpiSpeed));
    fpga_spi_cfg(SPI_IOC_RD_MAX_SPEED_HZ, (unsigned long)(&mSpiSpeed));

    return ret;
}

static int ads1256_basic_init(void)
{
    register_ads1256_drdy_irq();
    _adc_ready_irq_registered = 1;
    init_data_info();

    xque_init();
    if(request_gpio()!=0)
    {
        MWARN("request_gpio fail.\n");
        return -1;
    }
    registerAdcCallback(gi_ads1256_callback);

    if(init_spi_param()<0)
    {
        MWARN("spi init fail.\n");
        return -2;
    }
    fpga_power_on();
    adc_msleep(1000);
    StopReadC();
    adc_reset_hw();
    fpga_reset_hw();

    return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int ads1256_open(struct inode * inode, struct file * file)
{
    uint8_t regs[3] = {0xFF,0xFF,0xFF};
    int ret = 0,i;
    uint32_t dat[3];

    //adc_msleep(1000);
    if(ads1256_basic_init()<0)
    {
        return -1;
    }
#if 0
    if(!strcmp(param,"test"))
    {
        MDEBUG("********************start all test proc***********************\n");
        ///test/////
        initChipsForSample(SAMPLE_RATE_DEF);
        SyncStartReadC();
    }
    else if(!strcmp(param,"reset"))
    {
        MDEBUG("********************start reset test***********************\n");
        adc_reset_hw();
        gpio_set_value(fpga_sync, 0);
        printk("reset ok\n");
    }
    else if(!strcmp(param,"status"))
    {
        MDEBUG("********************start status reg test***********************\n");
        setBuffers(0);
        printk("status ok\n");
        ret = readAllStatusReg( regs);
        printk("status(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);
    }
    else if(!strcmp(param,"rate"))
    {
        MDEBUG("********************start sample rate test***********************\n");

        setSampleRates(SAMPLE_RATE_DEF);
        ads1256_data_info.sample_rate = SAMPLE_RATE_DEF;
        ads1256_data_info.sample_intern = 1000000/SAMPLE_RATE_DEF;
        printk("set sample rate ok\n");
        ret = readAllSampleRateReg(regs);
        printk("sample rate(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);
    }
    else if(!strcmp(param,"diff"))
    {
        uint8_t positiveCh=0;
        uint8_t negativeCh=1;

        MDEBUG("********************start diff reg test***********************\n");
        setDiffChannels(positiveCh << 4 | negativeCh);
        printk("setDiffChannels ok\n");
        ret = readAllDiffReg(regs);
        printk("Diff(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);
    }
    else if(!strcmp(param,"calib"))
    {
        MDEBUG("********************start calib test***********************\n");
        printk("setSelfCals\n");
        setSelfCals();
        printk("calib ok\n");

        readAllOFC(ads1256_data_info.ofc);
        printk("ofc:%d,%d,%d\n",ads1256_data_info.ofc[0],ads1256_data_info.ofc[1],ads1256_data_info.ofc[2]);
        readAllFSC(ads1256_data_info.fsc);
        printk("fsc:%d,%d,%d\n",ads1256_data_info.fsc[0],ads1256_data_info.fsc[1],ads1256_data_info.fsc[2]);
    }
    else if(!strcmp(param,"start"))
    {
        MDEBUG("********************start snyc test***********************\n");
        ads1256_data_info.sample_rate = SAMPLE_RATE_DEF;
        ads1256_data_info.sample_intern = 1000000/SAMPLE_RATE_DEF;
        SyncStartReadC();
        printk("start sample ok\n");
    }
#endif

    printk("================ads1256 open ok===================\n");
    return 0;
}

static  int ads1256_release(struct inode *inode, struct file *filp)
{
    StopReadC();
    xque_del();
    unregister_ads1256_drdy_irq();
    _adc_ready_irq_registered = 0;
    fpga_spi_cfg(SPI_CMD_CLOSE,0);
    registerAdcCallback(NULL);

    adc_reset_hw();
    free_gpio();
    MDEBUG("================ads1256 release ok===================\n");

    return 0;
}

static long
ads1256_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int i;
    long retval = 0;
    //printk("=============ads1256_ioctl:%d=========\n",cmd);
    switch(cmd)
    {
    case ADS1256_CFG_SPI:

        break;
    case ADS1256_CFG_AD://初始化配置
        initChipsForSample(arg);
        MDEBUG("ADS1256_CFG_AD OK\n");
        retval = 1;
        break;
    case ADS1256_START://启动采集
    case 	1234://启动采集
        MDEBUG("ADS1256_START\n");

#if 1
        if(arg) //需要判定输出数据是否需要校正
        {
            ads1256_data_info.need_cabli = true;
        }
        else
        {
            ads1256_data_info.need_cabli = false;
        }
        StopReadC();//先停止连续传输，再校正
        SyncStartReadC();
#endif
        retval = 1;
        break;
    case ADS1256_STOP://停止采集
        StopReadC();
        retval = 1;
        break;
    case ADS1256_CALIBRATION:
    {
        struct adc_cali_info info;
        void __user *buf = (void __user *)arg;

        setSelfCals();
        for(i = 0; i < 3; i++)
        {
            info.ofc[i] = readOFC(i);
            info.fsc[i] = readFSC(i);
        }
        if(copy_to_user(buf, &info, sizeof(struct adc_cali_info)))
        {
            MWARN("ADS1256_CALIBRATION, copy error\n");
            retval = -EFAULT;
        }
        retval = 1;
    }
    break;
    case ADS1256_READ_CALIB://读取校正参数
    {
        struct adc_cali_info info;
        void __user *buf = (void __user *)arg;

        for(i = 0; i < 3; i++)
        {
            info.ofc[i] =ads1256_data_info.ofc[i];//readOFC(i);
            info.fsc[i] = ads1256_data_info.fsc[i];//readFSC(i);
        }
        if(copy_to_user(buf, &info, sizeof(struct adc_cali_info)))
        {
            MWARN("ADS1256_READ_CALIB, copy error\n");
            retval = -EFAULT;
        }
        retval = 1;
    }
    break;
    case ADS1256_READDATA://读取采集数据
        if(ads1256_data_info.full_count>0)
        {
            void __user *buf = (void __user *)arg;
            retval = ADC_DATAPACK_COUNT;
            if(copy_to_user(buf, ads1256_data_info.adc_pingpong_data[ads1256_data_info.read_idx], ADC_DATAPACK_COUNT*sizeof(AdcData)))
            {
                MWARN("ADC_READ_FIFO, copy error\n");
                retval = -EFAULT;
            }
            ads1256_data_info.usr_read_count++;
#if 1
            if(!(ads1256_data_info.usr_read_count%600))
            {
                MDEBUG("adc period state(id:%08d,signal:%d,stamp:%lld, stamp_sys:%lld, up data:%d,%d,%d---lost err:%d,pos err:%d---isr:%d,read:%d,usr read:%d)\n",
                       ads1256_data_info.read_idx,
                       ads1256_data_info.adc_pingpong_data[ads1256_data_info.read_idx][0].gps_sig,
                       ads1256_data_info.adc_pingpong_data[ads1256_data_info.read_idx][0].timestamp,
                       ads1256_data_info.adc_pingpong_data[ads1256_data_info.read_idx][0].timestamp_sys,
                       ads1256_data_info.adc_pingpong_data[ads1256_data_info.read_idx][0].data[0],
                       ads1256_data_info.adc_pingpong_data[ads1256_data_info.read_idx][0].data[1],
                       ads1256_data_info.adc_pingpong_data[ads1256_data_info.read_idx][0].data[2],
                       ads1256_data_info.err_count,ads1256_data_info.err2_count,
                       ads1256_data_info.dready_count,ads1256_data_info.read_count,ads1256_data_info.usr_read_count
                      );
            }
#endif
            ads1256_data_info.read_idx=(ads1256_data_info.read_idx+1)%DATA_PACK_COUNT;
            ads1256_data_info.full_count--;
            //MWARN("ADS1256_READDATA OK, ads1256_data_info.full_count = %d\n", ads1256_data_info.full_count);
        }
        else
        {
            //MWARN("ads1256_data_info.full_count<=0, NO DATA!!!!\n");
        }
        break;
    case ADS1256_RESET:
        adc_reset_hw();
        retval = 1;
        break;
    default:
        break;
    }
    return retval;
}

static const struct file_operations ads1256_fops =
{
    .owner =	THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    //.write =	spidev_write,
    //.read =		spidev_read,
    .unlocked_ioctl = ads1256_ioctl,
    //.compat_ioctl = spidev_compat_ioctl,
    .open =		ads1256_open,
    .release =	ads1256_release
    //.fasync = gi_adc_ready_fasync,
    //.llseek =	no_llseek,
};

static struct miscdevice ads1256_dev =
{
    .minor		= MISC_DYNAMIC_MINOR,
    .name		= DEV_NAME,
    .fops  		= &ads1256_fops,
};

static int helloworld_init(void);
static void helloworld_exit(void);

static int __init ads1256_init(void)
{
    uint8_t regs[3] = {0xFF,0xFF,0xFF};
    int ret = 0,i;
    uint32_t dat[3];
	helloworld_init();
    //IO鍒濆鍖?   //涓柇鍒濆鍖?   //SPI鐩稿叧瀵勫瓨鍣ㄥ垵濮嬪寲
    MDEBUG("ads1256_init\n");
    ret = misc_register(&ads1256_dev);
    if(0 != ret)
    {
        MWARN("Kernel: register ads1256 device failed!\n");
        return -1;
    }
    MDEBUG("ads1256 register ok\n");
    adc_msleep(1000);
#if 0
    adc_msleep(1000);
    register_ads1256_drdy_irq();
    _adc_ready_irq_registered = 1;
    ads1256_data_info.read_idx = 0;
    ads1256_data_info.write_idx = 0;
    ads1256_data_info.write_pos = 0;
    ads1256_data_info.full_count = 0;
    ads1256_data_info.stamp = 0;
    ads1256_data_info.pre_gps_pulse_stamp = 0;
    ads1256_data_info.pre_data_pulse_stamp = 0;
    ads1256_data_info.dready_stamp = 0;
    ads1256_data_info.totl_count = 0;
    ads1256_data_info.err_count = 0;
    ads1256_data_info.err2_count = 0;
    ads1256_data_info.read_count = 0;
    ads1256_data_info.usr_read_count = 0;
    ads1256_data_info.dready_count = 0;
    ads1256_data_info.ad_init = false;
    ads1256_data_info.time_sync = false;
    ads1256_data_info.after_pulse_count = 0;
    ads1256_data_info.gpsok = 0;
    ads1256_data_info.need_cabli = false;
    ads1256_data_info.sync_state = SYNC_NULL;

    ret =  fpga_spi_cfg(SPI_CMD_OPEN,0);
    if(ret!=0)
    {
        MWARN("mSpiFd open error%d\n",ret);
        return -1;
    }
    else
    {
        printk("mSpiFd open ok\n");
    }
    xque_init();
    if(request_gpio()!=0)
    {
        MWARN("request_gpio fail.\n");
        return -1;
    }
    registerAdcCallback(gi_ads1256_callback);

    MDEBUG("spi mode: 0x%x\n", mSpiMode);
    fpga_spi_cfg(SPI_IOC_WR_MODE, (unsigned long)(&mSpiMode));
    fpga_spi_cfg(SPI_IOC_RD_MODE, (unsigned long)(&mSpiMode));
    MDEBUG("bits per word: %d\n", mSpiBits);
    fpga_spi_cfg(SPI_IOC_WR_BITS_PER_WORD, (unsigned long)(&mSpiBits));
    fpga_spi_cfg(SPI_IOC_RD_BITS_PER_WORD, (unsigned long)(&mSpiBits));
    MDEBUG("max speed: %d Hz (%d KHz)\n", mSpiSpeed, mSpiSpeed/1000);
    fpga_spi_cfg(SPI_IOC_WR_MAX_SPEED_HZ, (unsigned long)(&mSpiSpeed));
    fpga_spi_cfg(SPI_IOC_RD_MAX_SPEED_HZ, (unsigned long)(&mSpiSpeed));

    if(!strcmp(param,"test"))
    {
        MDEBUG("********************start all test proc***********************\n");
        ///test/////
        initChipsForSample(SAMPLE_RATE_DEF);
        SyncStartReadC();
    }
    else if(!strcmp(param,"reset"))
    {
        MDEBUG("********************start reset test***********************\n");
        adc_reset_hw();
        gpio_set_value(fpga_sync, 0);
        printk("reset ok\n");
    }
    else if(!strcmp(param,"status"))
    {
        MDEBUG("********************start status reg test***********************\n");
        setBuffers(0);
        printk("status ok\n");
        ret = readAllStatusReg( regs);
        printk("status(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);
    }
    else if(!strcmp(param,"rate"))
    {
        MDEBUG("********************start sample rate test***********************\n");

        setSampleRates(SAMPLE_RATE_DEF);
        ads1256_data_info.sample_rate = SAMPLE_RATE_DEF;
        ads1256_data_info.sample_intern = 1000000/SAMPLE_RATE_DEF;
        printk("set sample rate ok\n");
        ret = readAllSampleRateReg(regs);
        printk("sample rate(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);
    }
    else if(!strcmp(param,"diff"))
    {
        uint8_t positiveCh=0;
        uint8_t negativeCh=1;

        MDEBUG("********************start diff reg test***********************\n");
        setDiffChannels(positiveCh << 4 | negativeCh);
        printk("setDiffChannels ok\n");
        ret = readAllDiffReg(regs);
        printk("Diff(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);
    }
    else if(!strcmp(param,"calib"))
    {
        MDEBUG("********************start calib test***********************\n");
        printk("setSelfCals\n");
        setSelfCals();
        printk("calib ok\n");

        readAllOFC(ads1256_data_info.ofc);
        printk("ofc:%d,%d,%d\n",ads1256_data_info.ofc[0],ads1256_data_info.ofc[1],ads1256_data_info.ofc[2]);
        readAllFSC(ads1256_data_info.fsc);
        printk("fsc:%d,%d,%d\n",ads1256_data_info.fsc[0],ads1256_data_info.fsc[1],ads1256_data_info.fsc[2]);
    }
    else if(!strcmp(param,"start"))
    {
        MDEBUG("********************start snyc test***********************\n");
        ads1256_data_info.sample_rate = SAMPLE_RATE_DEF;
        ads1256_data_info.sample_intern = 1000000/SAMPLE_RATE_DEF;
        SyncStartReadC();
        printk("start sample ok\n");
    }
#endif


    if(!strcmp(param,"test"))
    {
        adc_msleep(1000);
        MDEBUG("********************start all test proc***********************\n");
        if(ads1256_basic_init()<0)
        {
            return -1;
        }
        ///test/////
        initChipsForSample(SAMPLE_RATE_DEF);
        SyncStartReadC();
    }
    else if(!strcmp(param,"reset"))
    {
        MDEBUG("********************start reset test***********************\n");
        if(ads1256_basic_init()<0)
        {
            return -1;
        }
        fpga_reset_hw();
        gpio_set_value(fpga_sync, 0);
        printk("reset ok\n");
    }
    else if(!strcmp(param,"status"))
    {
        MDEBUG("********************starting status reg test***********************\n");
        if(ads1256_basic_init()<0)
        {
            return -1;
        }
        fpga_reset_hw();
        setBuffers(0);
        printk("status ok---\n");
        while(1)
        {
            ret = readAllStatusReg( regs);
            printk("status(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);
            adc_msleep(10000);
        }
    }
    else if(!strcmp(param,"rate"))
    {
        MDEBUG("********************start sample rate test***********************\n");
        if(ads1256_basic_init()<0)
        {
            return -1;
        }
        fpga_reset_hw();
        setSampleRates(SAMPLE_RATE_DEF);
        ads1256_data_info.sample_rate = SAMPLE_RATE_DEF;
        ads1256_data_info.sample_intern = 1000000/SAMPLE_RATE_DEF;
        printk("set sample rate ok\n");
        ret = readAllSampleRateReg(regs);
        printk("sample rate(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);
    }
    else if(!strcmp(param,"diff"))
    {
        uint8_t positiveCh=0;
        uint8_t negativeCh=1;

        if(ads1256_basic_init()<0)
        {
            return -1;
        }
        fpga_reset_hw();
        MDEBUG("********************start diff reg test***********************\n");
        setDiffChannels(positiveCh << 4 | negativeCh);
        printk("setDiffChannels ok\n");
        ret = readAllDiffReg(regs);
        printk("Diff(ret=%d):%02x,%02x,%02x\n",ret,regs[0],regs[1],regs[2]);
    }
    else if(!strcmp(param,"calib"))
    {
        MDEBUG("********************start calib test***********************\n");
        if(ads1256_basic_init()<0)
        {
            return -1;
        }
        fpga_reset_hw();
        printk("setSelfCals\n");
        setSelfCals();
        printk("calib ok\n");

        readAllOFC(ads1256_data_info.ofc);
        printk("ofc:%d,%d,%d\n",ads1256_data_info.ofc[0],ads1256_data_info.ofc[1],ads1256_data_info.ofc[2]);
        readAllFSC(ads1256_data_info.fsc);
        printk("fsc:%d,%d,%d\n",ads1256_data_info.fsc[0],ads1256_data_info.fsc[1],ads1256_data_info.fsc[2]);
    }
    else if(!strcmp(param,"start"))
    {
        MDEBUG("********************start snyc test***********************\n");
        if(ads1256_basic_init()<0)
        {
            return -1;
        }
        fpga_reset_hw();
        ads1256_data_info.sample_rate = SAMPLE_RATE_DEF;
        ads1256_data_info.sample_intern = 1000000/SAMPLE_RATE_DEF;
        SyncStartReadC();
        printk("start sample ok\n");
    }

    return ret;
}
module_init(ads1256_init);

static void __exit ads1256_exit(void)
{
	helloworld_exit();
    if(_adc_ready_irq_registered)
    {
        StopReadC();
        xque_del();
        unregister_ads1256_drdy_irq();
        fpga_spi_cfg(SPI_CMD_CLOSE,0);
        registerAdcCallback(NULL);

        adc_reset_hw();
        free_gpio();
        _adc_ready_irq_registered = 0;
    }

    misc_deregister(&ads1256_dev);
    //fpga_power_off();
    MDEBUG("ads1256_exit\n");
}
module_exit(ads1256_exit);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>

static char* version="meihuan, ads1256, V1.0, Wed Sep 19 17:27:11, by whs";
static char* objname="ads1256ver";

static ssize_t version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
   return sprintf(buf, "%s\n", version);
}

static struct kobj_attribute hello_value_attribute = __ATTR_RO(version);
static struct kobject *helloworld_kobj;

static int helloworld_init(void)  //加到module_init
{
   int retval;
   helloworld_kobj = kobject_create_and_add(objname, kernel_kobj);

   if (!helloworld_kobj)
             return -ENOMEM;

   retval = sysfs_create_file(helloworld_kobj, &hello_value_attribute);

   if (retval)
      kobject_put(helloworld_kobj);

   return retval;
}

static void helloworld_exit(void)  //加到module_exit
{
   kobject_put(helloworld_kobj);
}
/////

MODULE_DESCRIPTION("SPI Master Controller driver");
MODULE_AUTHOR("Meihuan ads1256 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ads1256");
