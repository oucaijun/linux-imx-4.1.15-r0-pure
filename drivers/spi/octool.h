#ifndef _MTOOL_H_
#define _MTOOL_H_

#define MDEBUG  (printk("XXX MDEBUG %s (%d) <%s> ",__FILE__,__LINE__,__FUNCTION__),printk)
#define MWARN  (printk("XXX MWARN %s (%d) <%s>" ,__FILE__,__LINE__,__FUNCTION__),printk)
extern uint64_t get_timestamp(void);
extern int oc_callc_inc(int index);
extern int oc_callc_check(int index);
extern unsigned int oc_callc_cnt(int index);

#include <linux/types.h>
struct tr_flag_t {
    unsigned int cnt;
    unsigned int trig;
    char* info;
    uint64_t trig_time;
};

extern struct tr_flag_t g_tr_flag[10];
#endif

