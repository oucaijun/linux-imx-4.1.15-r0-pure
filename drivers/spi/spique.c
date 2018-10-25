
#include "spique.h"
#include <linux/slab.h>
/* initialize the list head*/
Queue *init_queue(int max)
{
    Queue *Q;
    Q = (Queue *)kmalloc(sizeof(Queue),GFP_KERNEL);
    Q->size = 0;
    Q->capacity = max;        
    INIT_LIST_HEAD(&Q->list);
    return Q;
}

int32_t front_queue(Queue *Q, QueueElement *e)
{
    Queue* first_element;
    struct list_head * first;
    if(Q->size==0)
    {
        return -1;
    }
    if(e == NULL)
    {
        return -2;
    }

    /* find the first element first */
    first = Q->list.next;
    /* reconstruct the first structure */
    first_element = list_entry(first, Queue, list);

    *e = first_element->e;
    return 0;
}

/* add to the tail */
int push_queue(Queue *Q, QueueElement element)
{
    Queue* newQ;
    if(Q->size == Q->capacity)
    {
        return -1;
    }
    else
    {
        Q->size++;
        newQ = (Queue*) kmalloc(sizeof(Queue),GFP_KERNEL);
        newQ->e = element;

        list_add_tail(&(newQ->list), &(Q->list));
    }
    return 0;
}

/* pop out the front, and free the element space */
void pop_queue(Queue *Q)
{
    Queue* tmp;
    if(Q->size==0)
    {
        return;
    }
    else
    {
        Q->size--;
        tmp = list_entry(Q->list.next, Queue, list);
        list_del(Q->list.next);
        kfree(tmp);
    }
}

int queue_full(Queue *Q)
{
    if(Q->size >= Q->capacity) {
        return 1;
    } 
    return 0;
}
int queue_null(Queue *Q)
{
    if(Q->size==0) {
        return 1;
    } 
    return 0;
}

void clean_queue(Queue *Q)
{
    while(Q->size > 0)
    {
        pop_queue(Q);
    }
}

int get_queue_size(Queue *Q)
{
    return Q->size;
}

int get_queue_capacity(Queue *Q)
{
    return Q->capacity;
}

