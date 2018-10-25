#ifndef __ASM_GENERIC_CURRENT_H
#define __ASM_GENERIC_CURRENT_H

#include <linux/thread_info.h>

//#define get_current() (current_thread_info()->task)
#define get_current() ((current_thread_info() != NULL)? current_thread_info()->task : NULL)
#define current get_current()

#endif /* __ASM_GENERIC_CURRENT_H */
