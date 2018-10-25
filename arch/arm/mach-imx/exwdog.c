
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/semaphore.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/workqueue.h>
#include <linux/timer.h> // for timer_list API
#include <linux/param.h> // for HZ 
#include <linux/jiffies.h> // for jiffies
#include <asm/uaccess.h>
#include <asm/io.h>

//#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
//#include <linux/slab.h>

#define LED_IMX_GPIO_NR(bank, nr) (((bank) - 1) * 32 + (nr))
#define ENABLE_LED_GPIO LED_IMX_GPIO_NR(1, 21)

static struct timer_list ext_wdt_timer;

static void ext_wdt_handler(unsigned long time) {
    
    static unsigned int cnt = 0;
    cnt++;
    if(0 == (cnt % 2)) {
        gpio_set_value(ENABLE_LED_GPIO, 0);
    }
    else
    {
        gpio_set_value(ENABLE_LED_GPIO, 1);
    }
    mod_timer(&ext_wdt_timer, jiffies + msecs_to_jiffies(400));//    return;
}   

static void ext_wdt_timer_setup(unsigned int ntime) {
    /*** Initialize the timer structure***/
    gpio_request_one(LED_IMX_GPIO_NR(1, 21), GPIOF_OUT_INIT_LOW, "ext_wdt_ctrl_gpio");
    init_timer(&ext_wdt_timer);//
    ext_wdt_timer.function = ext_wdt_handler;//
    ext_wdt_timer.expires = jiffies + msecs_to_jiffies(ntime);
    add_timer(&ext_wdt_timer);
    /***Initialisation ends***/
    return;
}

static int __init ext_wdt_init(void) {
    unsigned int ntime = 10; //10ms
    ext_wdt_timer_setup(ntime);
    printk("init ext_wdt ok\n");
    return 0;         
}

static void __exit ext_wdt_exit(void) {
    del_timer_sync(&ext_wdt_timer);
}

module_init(ext_wdt_init);
module_exit(ext_wdt_exit);

MODULE_AUTHOR("oucaijun.happy@163.com");
MODULE_LICENSE("GPL");


