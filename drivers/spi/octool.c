#include <linux/module.h>
#include <linux/timekeeping.h>
#include "octool.h"

uint64_t get_timestamp(void)
{
    struct timeval now;
    uint64_t ts;
    do_gettimeofday(&now);
    ts = now.tv_sec;
    ts *= 1000000;
    ts += now.tv_usec;
    return ts;
}


struct tr_flag_t g_tr_flag[10];


#define OPENCK 0

int oc_callc_inc(int index)
{
#ifdef OPENCK
    g_tr_flag[index].cnt++;
    g_tr_flag[index].trig = 1;
    g_tr_flag[index].trig_time = get_timestamp();
#endif    
    return 0;
}

int oc_callc_check(int index)
{
    int ret =0;
 #ifdef OPENCK
 if(g_tr_flag[index].trig == 1) {
        g_tr_flag[index].trig = 0;
        ret = 1;
    } else {
        ret = 0;
    }
#endif    
    return ret;
    
}

unsigned int oc_callc_cnt(int index)
{
#ifdef OPENCK
    return g_tr_flag[index].cnt;
#else    
    return 0
 #endif      
}

EXPORT_SYMBOL(get_timestamp);
EXPORT_SYMBOL(oc_callc_inc);
EXPORT_SYMBOL(oc_callc_check);
EXPORT_SYMBOL(oc_callc_cnt);
EXPORT_SYMBOL_GPL(g_tr_flag);
MODULE_LICENSE("GPL");


