#include <linux/module.h>  
#include <linux/device.h>  
#include <linux/fs.h>  
#include <linux/miscdevice.h>  
#include <linux/sched.h>  
#include <linux/wait.h>  
#include <linux/irqreturn.h>  
#include <asm/uaccess.h>  
// #include <sys/time.h>
#include <linux/time.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include "gpio_gps_interrupt.h"

#define IMX_GPIO_NR(bank, nr)		(((bank) - 1) * 32 + (nr))

MODULE_AUTHOR("oucaijun@huania.com");
MODULE_DESCRIPTION("gps interrupt");
MODULE_LICENSE("GPL");

#define MAX_DEVNAME_LEN 30
struct gi_desc_struct
{
    int grp_index;
    int entry_index;
    unsigned int irq; //irq = gpio_to_irq(IMX_GPIO_NR(grp_index,entry_index));
    irq_handler_t irq_handler;
    unsigned long flags;
    char name[MAX_DEVNAME_LEN]; 
};

struct gi_pulse_callback
{
	giadc_callback adc_func;
	gi_callback gps_func;
};


struct gps_adjust_stuct
{
	unsigned long usr_settime_tick;//us
	unsigned long usr_set_seconds;
	unsigned long cur_seconds;
	struct timespec SetTime;
	unsigned long pre_pulse_tick;
	unsigned long pre_adjustok_tick;//us
	int offset;//us
	int signal;
	bool enable;
	bool valid;
	unsigned long sysHZ;
	unsigned long adjust_count;
};

#define GI_NAMESPACE "gi_gps_pusle"

static struct cdev* gi_cdevp;
static struct class* gi_cls;
static dev_t gi_major;
#define GI_COUNT 1
static int gi_triggered[GI_COUNT];
unsigned long _gps_irq_sum=0; //for check

static struct gi_pulse_callback pulse_callback;
static struct gps_adjust_stuct gps_adjust_info;
/*irq variable*/
static DECLARE_WAIT_QUEUE_HEAD(gi_read_wait);  
static int wait_flag = 0;  

static irqreturn_t gi_irq(int irq, void *dev_id)  
{  
    volatile int *press_cnt = (volatile int *)dev_id;
    *press_cnt = *press_cnt + 1; 
    //printk("gpio irq happend! *press_cnt=%d\n", *press_cnt); 

	if(pulse_callback.gps_func!=NULL){
		//printk("pulse_callback.adc_func ()===\n"); 
		pulse_callback.gps_func ();
	}
	if(pulse_callback.adc_func!=NULL){
		//printk("pulse_callback.adc_func ()===\n"); 
		pulse_callback.adc_func (gps_adjust_info.valid,gps_adjust_info.signal,(uint64_t)(gps_adjust_info.cur_seconds)*1000000);
	}	
    wait_flag = 1;      
    wake_up(&gi_read_wait); 
    _gps_irq_sum++;

    return IRQ_HANDLED;  
}  

static struct gi_desc_struct gi_map[GI_COUNT] = {
    {1, 12, -1, gi_irq, IRQF_TRIGGER_RISING, GI_NAMESPACE },
};

#include <linux/fs.h>
static struct fasync_struct *fasync;
static int gi_gps_fasync(int fd, struct file *fp, int on)
{
	return fasync_helper(fd, fp, on, &fasync);
}

static ssize_t gpio_int_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret;
	int cnt = (count<=sizeof(gi_triggered)) ? count : sizeof(gi_triggered);
	
    wait_event_interruptible(gi_read_wait, wait_flag);
	kill_fasync(&fasync, SIGIO, POLL_IN);
    wait_flag = 0;
   
    // always return 4*GI_COUNT bytes
	if (copy_to_user(buf, (void *)gi_triggered, cnt)) {
		ret = -EFAULT;
	} else {
		ret = cnt;
	}	
	
	memset(gi_triggered, 0, sizeof(gi_triggered));
	return ret;
}

static void 
gps_adjust_proc(void)
{
	int64_t sec = gps_adjust_info.usr_set_seconds;
    time_t timep1;
    struct tm *Tlocal;
    struct  timeval SysTimep;
    int64_t dif;
    uint8_t NeedSet=0;//是否需要校准
    uint8_t NeedPrint=0;

    do_gettimeofday(&SysTimep);
	gps_adjust_info.offset = SysTimep.tv_usec;
    if(SysTimep.tv_sec>sec)//如果当前系统时间大于要同步的时间
    {
        NeedSet=1;
        NeedPrint=1;
    }else if(SysTimep.tv_sec==sec)
    {
        if(SysTimep.tv_usec>=200)
        {
            NeedSet=1;
        }
        if(SysTimep.tv_usec>=200)
        {
            NeedPrint=1;
        }
    }else
    {
        dif=sec-SysTimep.tv_sec;
        if(dif>1)
        {
            NeedPrint=1;
            NeedSet=1;
        }
        else
        {
            if(SysTimep.tv_usec<=999800)NeedSet=1;
            if(SysTimep.tv_usec<=999800)NeedPrint=1;
        }
    }

    if(NeedSet&&gps_adjust_info.enable)
//	&&((unsigned long)times(NULL)-gps_adjust_info.usr_settime_tick )<gps_adjust_info.HZ*2)
    {
        gps_adjust_info.SetTime.tv_sec=sec;
         gps_adjust_info.SetTime.tv_nsec=0;
	//printk("==SetTime:%u,%u\n",gps_adjust_info.SetTime.tv_sec,gps_adjust_info.SetTime.tv_nsec);
	
        do_settimeofday(& gps_adjust_info.SetTime); //设置系统时间,差1秒??
        gps_adjust_info.adjust_count++;
	gps_adjust_info.valid =(gps_adjust_info.signal>=3);

	do_gettimeofday(&SysTimep);
	//printk("adjust time:%d--%ld(%d)\n",sec,SysTimep.tv_sec,SysTimep.tv_usec);
#if 0
        if(NeedPrint)
        {
            printf("校准之前微秒:%ld  ",SysTimep.tv_usec);
            printf("校准之前秒:%ld  ",SysTimep.tv_sec);
            printf("秒:%lld\n",sec);
        }

        gettimeofday(&SysTimep,NULL);
        time(&timep1);
        Tlocal= localtime(&timep1);
        if(NeedPrint)
        printf("有校准:秒:%d ,分:%d,时:%d,天:%d,月:%d,年:%d\n",Tlocal->tm_sec,Tlocal->tm_min,Tlocal->tm_hour,Tlocal->tm_mday,Tlocal->tm_mon,Tlocal->tm_year);
#endif
		gps_adjust_info.cur_seconds = sec;
		//printk("adjust time:%d--%ld(%d)\n",gps_adjust_info.cur_seconds,SysTimep.tv_sec,SysTimep.tv_usec);
    }
	else{
		gps_adjust_info.cur_seconds++;
		
		//printk("adjust time:%d--%ld(%d)\n",gps_adjust_info.cur_seconds,SysTimep.tv_sec,SysTimep.tv_usec);
	}
	//printk("en:%d(NeedSet:%d,gps_adjust_info.enable:%d),gps sig:%d,adjust sec:%d\n",NeedSet&&gps_adjust_info.enable,NeedSet,gps_adjust_info.enable,
	//																		gps_adjust_info.signal,gps_adjust_info.cur_seconds);
	 gps_adjust_info.enable = false;
      //  system("hwclock -s");
     // do_gettimeofday(&SysTimep);
	
    return;
}

static long
gps_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	size_t byte_out = sizeof(unsigned long);
	
	switch( cmd)
	{
	case CROSS_CHECK_IRQ_CNT:
	{
		void __user *buf_out = (void __user *)(arg);
		if (__copy_to_user(buf_out, &_gps_irq_sum, byte_out)) {
			return -EFAULT;
		}
	}
		break;
	case SET_NEXT_SEC_STAMP:
	{
		struct gps_settime_info __user *  info = (struct gps_settime_info __user *)(arg);
		gps_adjust_info.usr_set_seconds = info->usr_set_seconds;
		gps_adjust_info.enable = true;
		//gps_adjust_info.signal = info->signal;
		printk("signal:%d,timeoffset:%d  preset:%u-useset:%u\n",gps_adjust_info.signal,gps_adjust_info.offset,gps_adjust_info.SetTime.tv_sec,gps_adjust_info.usr_set_seconds );
		//gps_adjust_info.usr_settime_tick  = (unsigned long)times(NULL);
		//printk("usr set seconds:%ul\n",gps_adjust_info.usr_set_seconds);
	}
		break;
	case SET_SIG_VALUE:
	{
		struct gps_sig_info __user *  info = (struct gps_sig_info  __user *)(arg);
		gps_adjust_info.signal = info->signal;	
	}
	break;

	case  GET_RECENT_OFFSET:
	{
		int __user *offset = (int __user *)(arg);
		*offset = gps_adjust_info.offset;
	}
		break;
	case GET_PRE_ADJUST_TIME:
	{
		unsigned long __user *t = (unsigned long __user *)(arg);
		*t = (unsigned long)(gps_adjust_info.SetTime.tv_sec) ;
	}
		break;
	case SETUP_ADJUST_TIME:
	{
		registerGpsCallback(gps_adjust_proc);
		printk("===register GpsCallback ok\n");
	}
		break;
	case GPS_CK__SIG:
	{
		int __user *signal = (int __user *)(arg);
		*signal = gps_adjust_info.signal;
	}
		break;
	default:
		break;
	}
	
	return 0;
}

static int gpio_int_open(struct inode *inode, struct file *filp)
{ 
	return 0;
}

static int gpio_int_release(struct inode *inode, struct file *filp)
{
	return gi_gps_fasync(-1, filp, 0);
}

static struct file_operations gpio_int_fops = {
	.owner = THIS_MODULE,
	.read = gpio_int_read,
	.open = gpio_int_open,
	.unlocked_ioctl = gps_ioctl, 
	.fasync  =  gi_gps_fasync,
	.release = gpio_int_release,
};



static 
int gpio_int_init(void)
{
	int ret, i;
	int devno;
    printk("enter gpio int driver\n");
	
	pulse_callback.adc_func = NULL;
	pulse_callback.gps_func = NULL;
	memset(&gps_adjust_info,0,sizeof(struct gps_adjust_stuct));
	//gps_adjust_info.sysHZ= sysconf(_SC_CLK_TCK);
    for(i=0;i<GI_COUNT;i++) {
		gi_map[i].irq = gpio_to_irq(IMX_GPIO_NR(gi_map[i].grp_index,gi_map[i].entry_index));
		
		/*gpio request and config*/
        if(gpio_is_valid(IMX_GPIO_NR(gi_map[i].grp_index,gi_map[i].entry_index))) {
            gpio_request_one(IMX_GPIO_NR(gi_map[i].grp_index,gi_map[i].entry_index), GPIOF_DIR_IN, gi_map[i].name);
        }
		/*irq request*/
		ret = request_irq(gi_map[i].irq, gi_map[i].irq_handler, gi_map[i].flags, gi_map[i].name, &gi_triggered[i]);  
		if (ret)  {
			return ret;  
		}
    }
	
	/*cdev register*/	
    ret = alloc_chrdev_region(&devno, 0, 1, GI_NAMESPACE"devrg");
    gi_major = MAJOR(devno);
	if(ret != 0) {
	    printk(KERN_ERR "gpio_int: unable alloc_chrdev_region\n");
        return ret;
	}
	
    gi_cdevp = kmalloc(sizeof(struct cdev), GFP_KERNEL);
    if(!gi_cdevp) {
        printk(KERN_ERR "gpio_int: unable kmalloc cdev\n");
        return -ENOMEM;
	}

	cdev_init(gi_cdevp, &gpio_int_fops);
	gi_cdevp->owner = THIS_MODULE;
	if ((ret = cdev_add(gi_cdevp, MKDEV(gi_major, 0), 1)) != 0) {
		printk(KERN_ERR "gpio_int: unable register character device\n");
		return ret;
	}
	
	gi_cls = class_create(THIS_MODULE, GI_NAMESPACE"cls");
	if (IS_ERR(gi_cls)) {
		ret = PTR_ERR(gi_cls);
		goto error;
	}

	device_create(gi_cls, NULL, MKDEV(gi_major, 0), NULL, GI_NAMESPACE);

	
	return 0;

error:
	cdev_del(gi_cdevp);
	unregister_chrdev_region(MKDEV(gi_major, 0), 1);
	return ret;
}

static 
void gpio_int_exit(void)
{
    int i;
    for(i=0; i<GI_COUNT;i++) {
	    gpio_free(IMX_GPIO_NR(gi_map[i].grp_index,gi_map[i].entry_index));
		free_irq(gi_map[i].irq, &gi_triggered[i]); 
    }
    
	device_destroy(gi_cls, MKDEV(gi_major, 0));
	cdev_del(gi_cdevp);
	kfree(gi_cdevp);
	class_destroy(gi_cls);
	unregister_chrdev_region(MKDEV(gi_major, 0), 1);
	gps_adjust_info.enable = false;
	pulse_callback.adc_func = NULL;
	pulse_callback.gps_func = NULL;	
	printk("exit gpio int driver\n");
}


void registerAdcCallback(giadc_callback fun)
{
	pulse_callback.adc_func = fun;	
}
EXPORT_SYMBOL(registerAdcCallback);


void registerGpsCallback(gi_callback fun)
{
	pulse_callback.gps_func = fun;
}
EXPORT_SYMBOL(registerGpsCallback);

module_init(gpio_int_init);
module_exit(gpio_int_exit);
