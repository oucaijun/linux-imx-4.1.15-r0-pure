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

#define IMX_GPIO_NR(bank, nr)		(((bank) - 1) * 32 + (nr))

MODULE_AUTHOR("oucaijun@huania.com");
MODULE_DESCRIPTION("adc interrupt");
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

#define GI_NAMESPACE "gi_adc_ready"

static struct cdev* gi_cdevp;
static struct class* gi_cls;
static dev_t gi_major;

/*irq variable*/
static DECLARE_WAIT_QUEUE_HEAD(gi_read_wait);  
static int wait_flag = 0;  


unsigned long _adc_irq_sum = 0; //for check
unsigned long  _adc_read_sum = 0; //for check
 
static irqreturn_t gi_irq(int irq, void *dev_id)  
{  
    volatile int *press_cnt = (volatile int *)dev_id;
    *press_cnt = *press_cnt + 1; 

    _adc_irq_sum++;

    //printk("gpio irq happend! *press_cnt=%d\n", *press_cnt); 
	
    wait_flag = 1;      
    wake_up(&gi_read_wait); 

    return IRQ_HANDLED;  
}  

#define GI_COUNT 1
static int gi_triggered[GI_COUNT];
static struct gi_desc_struct gi_map[GI_COUNT] = {
    {6,8,-1,gi_irq,IRQF_TRIGGER_FALLING, GI_NAMESPACE },
};
#include <linux/fs.h>
static struct fasync_struct *fasync;
static int gi_adc_ready_fasync(int fd, struct file *fp, int on)
{
	printk("driver: gi_adc_ready_fasync\n");
	return fasync_helper(fd, fp, on, &fasync);
}

static ssize_t gpio_int_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret;
	_adc_read_sum++;
	
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
static int gpio_int_open(struct inode *inode, struct file *filp)
{ 
	return 0;
}


static int gpio_int_release(struct inode *inode, struct file *filp)
{
	return gi_adc_ready_fasync(-1, filp, 0);
}

static long
adc_int_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	#define CROSS_CHECK_IRQ_CNT 0xA0
	size_t byte_out = 2 * sizeof(unsigned long);
	unsigned long long for_check[2];
	for_check[0] = _adc_read_sum;
	for_check[1] = _adc_irq_sum;
	
	if( cmd  ==  CROSS_CHECK_IRQ_CNT) 
	{
	
		void __user *buf_out = (void __user *)(arg);
		if (__copy_to_user(buf_out, for_check, byte_out)) {
			return -EFAULT;
		}
	}
	
	return 0;
}

static struct file_operations gpio_int_fops = {
	.owner = THIS_MODULE,
	.read = gpio_int_read,
	.open = gpio_int_open,	
	.fasync = gi_adc_ready_fasync,
	.release = gpio_int_release,
	.unlocked_ioctl = adc_int_ioctl,
};

static 
int gpio_int_init(void)
{
	int ret, i;
	int devno;
    printk("enter gpio int driver\n");
	
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

	printk("exit gpio int driver\n");
}

module_init(gpio_int_init);
module_exit(gpio_int_exit);
