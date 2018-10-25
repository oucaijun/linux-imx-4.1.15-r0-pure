/*
 * gpio input dev driver
 *
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
//#include <asm/system.h>
#include <asm/uaccess.h>

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/gpio.h>

#define N_GPIO_IN 2
#define IMX_GPIO_NR(bank, nr)		(((bank) - 1) * 32 + (nr))

struct gpio_dev {
	struct cdev cdev;
	unsigned int gpio_idx;
};

#define MAX_DEVNAME_LEN 30
struct gpio_idx_struct
{
	int grp_idx;
	int entry_idx;
	char dev_name[MAX_DEVNAME_LEN];
	int init_value; //GPIOF_IN   
};

static ssize_t gpio_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos);
static int gpio_dev_open(struct inode *inode, struct file *filp);
static int gpio_dev_release(struct inode *inode, struct file *filp);

static struct class *gpio_class;
static int gpio_major = 0;
static struct gpio_dev *gpio_devp;
static const struct file_operations gpio_dev_fops = {
	.owner = THIS_MODULE, 
	.read = gpio_dev_read, 
	.open = gpio_dev_open, 
	.release = gpio_dev_release, 
};
static struct gpio_idx_struct gpio_array[N_GPIO_IN] = 
{
	{4, 19, "gpio_in_production_check", GPIOF_IN  },
	{6, 16, "gpio_in_drv8812_fault", GPIOF_IN  },
	//{5, 14, "gpio_in_rtc3231", GPIOF_IN  },   //将注册为中断，不用了。
	//{1, 12, "gpio_in_gps", GPIOF_IN  },		//将注册为中断，不用了。
	//{6, 8, "gpio_in_adc_rdy", GPIOF_IN  },	//已经注册为中断，不用了。
};

static int gpio_dev_open(struct inode *inode, struct file *filp)
{
	struct gpio_dev *dev;

	dev = container_of(inode->i_cdev, struct gpio_dev, cdev);
	filp->private_data = dev;

	return 0;
}

static int gpio_dev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t gpio_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned char val;
	int ret = 0;
	struct gpio_dev *dev;
	
	if(count != 1) {
		return -EFAULT;
	}
		
	dev = (struct gpio_dev *) filp->private_data;
	val = gpio_get_value(dev->gpio_idx);
	
	printk("gpio_in: val=%x\n", gpio_dev_read);
	
	if (copy_to_user(buf, &val,  1)) {
		ret = -EFAULT;
	} else {
		ret = count;
	}

	return ret;
}

static void setup_gpio_cdev(struct gpio_dev *dev, int index)
{
	int err, devno = MKDEV(gpio_major, index);

	cdev_init(&dev->cdev, &gpio_dev_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
	{
		printk(KERN_NOTICE "Error %d adding gpio in dev %d", err, index);
	}
}

static 
int gpio_dev_init(void)
{
	int result;
	dev_t devno = MKDEV(gpio_major, 0);
	int error;
	int i;

	printk("*****gpio in dev init*****\n");

	//申请设备号
	if (gpio_major) {
		result = register_chrdev_region(devno, N_GPIO_IN, "gpio in dev");
	} else {
		result = alloc_chrdev_region(&devno, 0, N_GPIO_IN, "gpio in dev");
		gpio_major = MAJOR(devno);
	}
	if (result < 0) {
		printk("***register dev fail.\n");
		return result;
	}

	//申请内存空间
	gpio_devp = kmalloc(N_GPIO_IN * sizeof(struct gpio_dev), GFP_KERNEL);
	if (!gpio_devp) {
		result = -ENOMEM;
		goto fail_malloc;
	}
	memset(gpio_devp, 0, N_GPIO_IN * sizeof(struct gpio_dev));

	//初始化gpio_idx并申请GPIO口
	for (i = 0; i < N_GPIO_IN; i++) {
		gpio_devp[i].gpio_idx = IMX_GPIO_NR(gpio_array[i].grp_idx, gpio_array[i].entry_idx);
		if (gpio_is_valid(gpio_devp[i].gpio_idx)) {
			error = gpio_request_one(gpio_devp[i].gpio_idx, gpio_array[i].init_value, "input gpio in dev");
			if (error < 0) {
				printk("Failed to request GPIO %d, error %d\n", gpio_devp[i].gpio_idx, error);
				return error;
			}
		}
	}
	
	//创建类和设备节点
	gpio_class = class_create(THIS_MODULE, "gpio_in");
	if (IS_ERR(gpio_class)) {
		printk("fail in create class.\n");
		return -1;
	}
	for (i = 0; i < N_GPIO_IN; i++) {
		device_create(gpio_class, NULL, MKDEV(gpio_major, i), NULL, gpio_array[i].dev_name, i);
	}

	//初始化、注册设备
	for (i = 0; i < N_GPIO_IN; i++) {
		setup_gpio_cdev(gpio_devp + i, i);
	}
	printk("***gpio in dev init ok.\n");
	
	return 0;

fail_malloc:
	printk("***alloc mem fail.\n");
	unregister_chrdev_region(devno, N_GPIO_IN);
	return result;
}

static void gpio_dev_exit(void)
{
	int i;	
	for (i = 0; i < N_GPIO_IN; i++) {
		gpio_free(IMX_GPIO_NR(gpio_array[i].grp_idx, gpio_array[i].entry_idx));
		device_destroy(gpio_class, MKDEV(gpio_major, i));
		cdev_del(&(gpio_devp[i].cdev));
	}

	kfree(gpio_devp);
	unregister_chrdev_region(MKDEV(gpio_major, 0), N_GPIO_IN);
	class_destroy(gpio_class);
}

MODULE_AUTHOR("ocean");
MODULE_DESCRIPTION("gpio in");
MODULE_LICENSE("GPL v2");
module_param(gpio_major, int, S_IRUGO);

module_init(gpio_dev_init);
module_exit(gpio_dev_exit);

