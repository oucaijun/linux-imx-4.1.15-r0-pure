#ifndef _SPI_QUE_H_
#define _SPI_QUE_H_

#include <linux/list.h>

#define ADC_QUEUE_SIZE 0xffff

#define ADD_POINT_EN  1


typedef struct
{
    int32_t data[3];
    int32_t gps_sig;
    uint64_t timestamp;
    uint64_t timestamp_sys;
#if ADD_POINT_EN
    int32_t valid;
#endif
} AdcData;

typedef struct
{
    int32_t data[3];
    uint64_t timestamp;
} AdcDataOld;

#define QueueElement AdcData

typedef struct
{
    int capacity;
    int size;
    QueueElement e;
    struct list_head list;  
} Queue;

Queue * init_queue(int max);
int front_queue(Queue *Q, QueueElement *element);
int push_queue(Queue *Q, QueueElement element);
void pop_queue(Queue *Q);
void clean_queue(Queue *Q);
int get_queue_size(Queue *Q);
int get_queue_capacity(Queue *Q);
int queue_full(Queue *Q);
int queue_null(Queue *Q);

#endif
