/*
 * gpio output dev driver
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

#define N_GPIO_OUT 24
#define IMX_GPIO_NR(bank, nr)		(((bank) - 1) * 32 + (nr))

struct gpio_dev {
	struct cdev cdev;
	unsigned int gpio_idx;
};

#define MAX_DEVNAME_LEN 31
struct gpio_idx_struct
{
	int grp_idx;
	int entry_idx;
	char dev_name[MAX_DEVNAME_LEN];
	int init_value; //GPIOF_OUT_INIT_HIGH   GPIOF_OUT_INIT_LOW
};

static ssize_t gpio_dev_read (struct file *filp, char __user *buf, size_t count, loff_t *ppos);
static ssize_t gpio_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos);
static int gpio_dev_open(struct inode *inode, struct file *filp);
static int gpio_dev_release(struct inode *inode, struct file *filp);


static struct class *gpio_class;
static int gpio_major = 0;
static struct gpio_dev *gpio_devp;
static const struct file_operations gpio_dev_fops = {
	.owner = THIS_MODULE, 
	.write = gpio_dev_write, 
	.read = gpio_dev_read,
	.open = gpio_dev_open, 
	.release = gpio_dev_release, 
};
static struct gpio_idx_struct gpio_array[N_GPIO_OUT] = 
{
//	{6, 11, "gpio_MB_DISABLE", GPIOF_OUT_INIT_HIGH }, //0 : RF OFF(Airplane mode)
//	{6, 14, "gpio_MB_RST"   , GPIOF_OUT_INIT_HIGH },  //0 and t>=1s : RF OFF, Module will reset.
	{6, 15, "gpio_3V3_OUT2" , GPIOF_OUT_INIT_HIGH },  // high to enable
	{1, 7 , "gpio_SLEEP"    , GPIOF_OUT_INIT_HIGH  }, // low to sleep drv8812
//	{5, 10, "gpio_X_RST"    , GPIOF_OUT_INIT_HIGH  }, // low to reset ads1256 x
//	{5, 17, "gpio_Y_RST"    , GPIOF_OUT_INIT_HIGH  }, // low to reset ads1256 y
//	{5, 5 , "gpio_Z_RST"    , GPIOF_OUT_INIT_HIGH  }, // low to reset ads1256 z
	{5, 7 , "gpio_RESET"    , GPIOF_OUT_INIT_HIGH  }, // low to reset drv8812
	{4, 29, "gpio_12V_EN"   , GPIOF_OUT_INIT_HIGH  }, // step motor contorl chip power, HIGH to enable
	{4, 24, "gpio_MX"       , GPIOF_OUT_INIT_LOW  }, //high to enable MX
	{4, 22, "gpio_MY"       , GPIOF_OUT_INIT_LOW  }, //high to enable MY
	{4, 17, "gpio_MZ"       , GPIOF_OUT_INIT_LOW  }, //high to enable MZ
	{4, 16, "gpio_3231_RST" , GPIOF_OUT_INIT_HIGH  }, // low to reset
	{4, 27, "gpio_DM_VDD_EN", GPIOF_OUT_INIT_HIGH }, // gpio_DM_VDD_EN 1 --> Super C127 Charge
	{5, 15, "gpio_DMX_EN"   , GPIOF_OUT_INIT_LOW  }, //high to enable DMX
	{4, 23, "gpio_DMY_EN2"  , GPIOF_OUT_INIT_LOW  }, //high to enable DMY
	{4, 31, "gpio_DMZ_EN"   , GPIOF_OUT_INIT_LOW  }, //high to enable DMZ
//	{6, 7 , "gpio_X_SYNC"   , GPIOF_OUT_INIT_HIGH  },  // ads1256 sync pin, low to sync
//	{6, 9 , "gpio_Y_SYNC"   , GPIOF_OUT_INIT_HIGH  },  // ads1256 sync pin, low to sync
//	{1, 20, "gpio_Z_SYNC"   , GPIOF_OUT_INIT_HIGH  },  // ads1256 sync pin, low to sync
	{5, 16, "gpio_B1"       , GPIOF_OUT_INIT_LOW  },   //drv8812 ctrl
	{5, 8 , "gpio_B0"       , GPIOF_OUT_INIT_LOW  },   //drv8812 ctrl
	{5, 11, "gpio_A1"       , GPIOF_OUT_INIT_LOW  },   //drv8812 ctrl
	{4, 30, "gpio_A0"       , GPIOF_OUT_INIT_LOW  },   //drv8812 ctrl
	{5, 6 , "gpio_BPHASE"   , GPIOF_OUT_INIT_LOW  },   //drv8812 ctrl
	{4, 26, "gpio_BENBL"    , GPIOF_OUT_INIT_LOW  },   //drv8812 ctrl
	{4, 28, "gpio_AENBL"    , GPIOF_OUT_INIT_LOW  },   //drv8812 ctrl
	{4, 21, "gpio_APHASE"   , GPIOF_OUT_INIT_LOW  },   //drv8812 ctrl
	{4, 18, "gpio_DECAY"    , GPIOF_OUT_INIT_LOW  },   //drv8812 ctrl
	{1, 4 , "gpio_DAC_LDACn", GPIOF_OUT_INIT_LOW  },   //active low. When LDAC is Low, the DAC latch is simultaneously updated with the content of the input register
	{1, 15, "gpio_CLOCK_EN" , GPIOF_OUT_INIT_HIGH  },  //high to enable input clk for all ads1256
	{1, 10, "gpio_GPS_RESET", GPIOF_OUT_INIT_HIGH  },  //low to reset gps
//	{5, 20, "gpio_CCE", GPIOF_OUT_INIT_LOW  },  //cce
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
static ssize_t gpio_dev_read (struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    unsigned int gpio_idx;
    unsigned int val;
    ssize_t ret = 0;
	struct gpio_dev *dev = filp->private_data;
	gpio_idx = dev->gpio_idx;
    val = gpio_get_value(gpio_idx);

    unsigned char v = val;
	if (copy_to_user(buf, &v, 1)) {
		ret = -1;
	} else {
		ret = count;
	}

	return ret;
}
static ssize_t gpio_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned int gpio_idx;
	unsigned char val;
	int ret = 0;


	struct gpio_dev *dev = filp->private_data;
	gpio_idx = dev->gpio_idx;
	
	if (copy_from_user(&val, buf, 1)) {
		ret = -EFAULT;
	} else {
		ret = count;
	}
	
	if (val) {
		gpio_set_value(gpio_idx, 1);
	} else {
		gpio_set_value(gpio_idx, 0);
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
		printk(KERN_NOTICE "Error %d adding gpio out dev %d", err, index);
	}
}

static 
int gpio_dev_init(void)
{
	int result;
	dev_t devno = MKDEV(gpio_major, 0);
	int error;
	int i;

	printk("*****gpio out dev init*****\n");

	//申请设备号
	if (gpio_major) {
		result = register_chrdev_region(devno, N_GPIO_OUT, "gpio out dev");
	} else {
		result = alloc_chrdev_region(&devno, 0, N_GPIO_OUT, "gpio out dev");
		gpio_major = MAJOR(devno);
	}
	if (result < 0) {
		printk("***register dev fail.\n");
		return result;
	}

	//申请内存空间
	gpio_devp = kmalloc(N_GPIO_OUT * sizeof(struct gpio_dev), GFP_KERNEL);
	if (!gpio_devp) {
		result = -ENOMEM;
		goto fail_malloc;
	}
	memset(gpio_devp, 0, N_GPIO_OUT * sizeof(struct gpio_dev));

	//初始化gpio_idx并申请GPIO口
	for (i = 0; i < N_GPIO_OUT; i++) {
		gpio_devp[i].gpio_idx = IMX_GPIO_NR(gpio_array[i].grp_idx, gpio_array[i].entry_idx);
		if (gpio_is_valid(gpio_devp[i].gpio_idx)) {
			error = gpio_request_one(gpio_devp[i].gpio_idx, gpio_array[i].init_value, "input gpio out dev");
			if (error < 0) {
				printk("Failed to request GPIO %d, error %d\n", gpio_devp[i].gpio_idx, error);
				return error;
			}
		}
	}
	
	//创建类和设备节点
	gpio_class = class_create(THIS_MODULE, "gpio_out");
	if (IS_ERR(gpio_class)) {
		printk("fail in create class.\n");
		return -1;
	}
	for (i = 0; i < N_GPIO_OUT; i++) {
		device_create(gpio_class, NULL, MKDEV(gpio_major, i), NULL, gpio_array[i].dev_name, i);
	}

	//初始化、注册设备
	for (i = 0; i < N_GPIO_OUT; i++) {
		setup_gpio_cdev(gpio_devp + i, i);
	}
	printk("***gpio out dev init ok.\n");
	
	return 0;

fail_malloc:
	printk("***alloc mem fail.\n");
	unregister_chrdev_region(devno, N_GPIO_OUT);
	return result;
}

static 
void gpio_dev_exit(void)
{
	int i;	
	for (i = 0; i < N_GPIO_OUT; i++) {
		gpio_free(IMX_GPIO_NR(gpio_array[i].grp_idx, gpio_array[i].entry_idx));
		device_destroy(gpio_class, MKDEV(gpio_major, i));
		cdev_del(&(gpio_devp[i].cdev));
	}

	kfree(gpio_devp);
	unregister_chrdev_region(MKDEV(gpio_major, 0), N_GPIO_OUT);
	class_destroy(gpio_class);
}

MODULE_AUTHOR("ocean");
MODULE_DESCRIPTION("gpio out");
MODULE_LICENSE("GPL v2");
module_param(gpio_major, int, S_IRUGO);

module_init(gpio_dev_init);
module_exit(gpio_dev_exit);

