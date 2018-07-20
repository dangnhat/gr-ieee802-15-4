/*
 * Copyright (C) 2018 Korea Advanced Institute of Science and Technology (KAIST)
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License.
 */

/**
 * @file        cir_queue.h
 * @brief       Circular queue.
 *
 * @author      Nhat Pham <nhatphd@kaist.ac.kr>
 */

#ifndef CIR_QUEUE_H_
#define CIR_QUEUE_H_

#include <cstdint>

#define CQUEUE_UNIT_TYPE int64_t
#define CQUEUE_SIZE_TYPE int64_t

#define CQUEUE_DEFAULT_SIZE 10

class cir_queue
{
public:

  /**
   * @brief   constructor, use default allocated memory space for the queue.
   *          Just for convenience with small queue.
   *          Init private (head = 0, tail = -1)
   *
   * @return  none
   */
  cir_queue ();

  /**
   * @brief   constructor, user must allocates memory space for the queue.
   *          Init private (head = 0, tail = -1)
   *
   * @param[in]   queue_p, pointer to buffer for queue.
   * @param[in]   queue_size, size of the buffer.
   *
   * @return  none
   */
  cir_queue (CQUEUE_UNIT_TYPE* queue_p, CQUEUE_SIZE_TYPE queue_size);

  /**
   * @brief   add one unit to the head of the circular queue.
   *
   * @param [in]  a_unit
   *
   * @return  none
   */
  void
  add_data (CQUEUE_UNIT_TYPE a_unit);

  /**
   * @brief   add a buffer of units to the head of the circular queue.
   *
   * @param [in]  buf, a buffer of units.
   * @param [in]  size, size of the buffer.
   *
   * @return  none
   */
  void
  add_data (CQUEUE_UNIT_TYPE* buf, CQUEUE_SIZE_TYPE size);

  /**
   * @brief   preview one data unit from the tail of the queue.
   *
   * @param [in] cont, true if continue from previous point, otherwise, begin from the tail.
   *
   * @return  a data unit (0 if queue is empty).
   */
  CQUEUE_UNIT_TYPE
  preview_data (bool cont);

  /**
   * @brief   get one data unit from the tail of the queue.
   *
   * @return  a data unit (0 if queue is empty).
   */
  CQUEUE_UNIT_TYPE
  get_data (void);

  /**
   * @brief   get a buffer of data units from the tail of the queue.
   *
   * @param [out]  buf, a buffer of data units.
   * @param [in]  size, size of the buffer.
   *
   * @return  actual number of data units has been read (<= size).
   */
  CQUEUE_SIZE_TYPE
  get_data (CQUEUE_UNIT_TYPE* buf, CQUEUE_SIZE_TYPE size);

  /**
   * @brief   get the queue's size.
   *
   * @return  size
   */
  CQUEUE_SIZE_TYPE
  get_size (void);

  /**
   * @brief   get the queue's head.
   *
   * @return  head
   */
  CQUEUE_SIZE_TYPE
  get_head (void)
  {
    return head;
  }

  /**
   * @brief   get the queue's tail.
   *
   * @return  tail
   */
  CQUEUE_SIZE_TYPE
  get_tail (void)
  {
    return tail;
  }

  /**
   * @brief   check if the queue has overflowed or not.
   *
   * @return  true if overflowed. Otherwise, false.
   */
  bool
  is_overflowed (void)
  {
    return overflowed;
  }

protected:
  CQUEUE_UNIT_TYPE* queue_p;
  CQUEUE_UNIT_TYPE queue_size;

private:
  CQUEUE_SIZE_TYPE head; /* next pos for new data to be pushed to the queue*/
  CQUEUE_SIZE_TYPE tail; /* next data to be pop from the queue */
  CQUEUE_SIZE_TYPE preview_pos;

  /* error indicators */
  bool overflowed;

  /* default memory space */
  CQUEUE_UNIT_TYPE default_queue[CQUEUE_DEFAULT_SIZE];
};

/** @} */
#endif // misc.h
