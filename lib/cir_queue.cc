/*
 * Copyright (C) 2018 Korea Advanced Institute of Science and Technology (KAIST)
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License.
 */

/**
 * @file        cir_queue.cpp
 * @brief       Circular queue.
 *
 * @author      Nhat Pham <nhatphd@kaist.ac.kr>
 */

#include <stddef.h>
#include "cir_queue.h"


/*----------------------------------------------------------------------------*/
cir_queue::cir_queue()
{
    this->queue_p = default_queue;
    this->queue_size = CQUEUE_DEFAULT_SIZE;

    this->head = 0;
    this->tail = -1;
    this->preview_pos = this->tail;

    overflowed = false;
}

/*----------------------------------------------------------------------------*/
cir_queue::cir_queue(CQUEUE_UNIT_TYPE *queue_p, CQUEUE_SIZE_TYPE queue_size)
{
    this->queue_p = queue_p;
    this->queue_size = queue_size;

    this->head = 0;
    this->tail = -1;
    this->preview_pos = this->tail;

    overflowed = false;
}

/*----------------------------------------------------------------------------*/
void cir_queue::add_data(CQUEUE_UNIT_TYPE a_unit)
{
    CQUEUE_SIZE_TYPE old_head = head;

    queue_p[head] = a_unit;

    if (tail == -1) { /* queue is empty */
        tail = head;
    }

    head = (head + 1) % queue_size;

    /* check overflowed */
    if ((old_head < tail) && (head >= tail)) {
        overflowed = true;
    }
}

/*----------------------------------------------------------------------------*/
void cir_queue::add_data(CQUEUE_UNIT_TYPE* buf, CQUEUE_SIZE_TYPE size)
{
    CQUEUE_SIZE_TYPE count;

    /* check size */
    if (size < 1) {
        return;
    }

    /* copy data from buffer to queue */
    for (count = 0; count < size; count++) {
        this->add_data(buf[count]);
    }
}

/*----------------------------------------------------------------------------*/
CQUEUE_UNIT_TYPE cir_queue::preview_data(bool cont)
{
    CQUEUE_UNIT_TYPE retval;

    if (tail == -1) {
        return 0;
    }

    if (cont == false) {
        preview_pos = tail;
    }

    retval = queue_p[preview_pos];

    preview_pos = (preview_pos + 1) % queue_size;

    return retval;
}

/*----------------------------------------------------------------------------*/
CQUEUE_UNIT_TYPE cir_queue::get_data(void)
{
    CQUEUE_UNIT_TYPE retdata;

    if (this->tail == -1) {
        return 0;
    }

    retdata = queue_p[this->tail];
    this->tail = (this->tail + 1) % queue_size;

    if (this->tail == this->head) {
        this->tail = -1;
    }

    return retdata;
}

/*----------------------------------------------------------------------------*/
CQUEUE_SIZE_TYPE cir_queue::get_data(CQUEUE_UNIT_TYPE* buf, CQUEUE_SIZE_TYPE size)
{
    CQUEUE_SIZE_TYPE queue_size;
    CQUEUE_SIZE_TYPE count;
    CQUEUE_SIZE_TYPE retsize;

    /* check size */
    if (size < 1) {
        return 0;
    }

    queue_size = this->get_size();

    if (queue_size > size) {
        for (count = 0; count < size; count++) {
            buf[count] = this->get_data();
        }

        retsize = size;
    } else {
        for (count = 0; count < queue_size; count++) {
            buf[count] = this->get_data();
        }

        retsize = queue_size;
    }

    return retsize;
}

CQUEUE_SIZE_TYPE cir_queue::get_size(void)
{
    return (tail==-1) ? 0 : (head > tail ? head - tail : head + queue_size - tail);
}

