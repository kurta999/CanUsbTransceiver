#include "circbuff.h"
#include "stm32g4xx_hal.h"

#include <stddef.h>
#include <string.h>

void circbuff_init(circbuff_t* cb, uint8_t* buff_ptr, circbuffsize_t size)
{
  uint32_t prim;
  prim = __get_PRIMASK();
  __disable_irq();
  cb->buff_ptr = buff_ptr;
  cb->end_ptr = buff_ptr + size;
  cb->tail_ptr = buff_ptr;
  cb->head_ptr = buff_ptr;
  if(!prim)
    __enable_irq();
}

uint8_t circbuff_add_byte(circbuff_t* cb, uint8_t b)
{
  uint8_t* head = cb->head_ptr;
  uint8_t* bgn = cb->buff_ptr;
  uint8_t* end = cb->end_ptr;
  uint8_t* new_head;
  uint8_t* tail;
  uint8_t is_full;
  uint32_t prim;
  prim = __get_PRIMASK();
  __disable_irq();
  tail = cb->tail_ptr;
  if(!prim)
    __enable_irq();
  is_full = (head == tail - 1) || (head == end - 1 && tail == bgn);
  if(is_full)
    return 0;
  *head = b;
  head++;
  new_head = head<end ? head : bgn;
	prim = __get_PRIMASK();
	__disable_irq();
  cb->head_ptr = new_head;
  if (!prim)
  	__enable_irq();
  return 1;
}

uint8_t circbuff_get_byte(circbuff_t* cb, uint8_t* b)
{
  uint8_t* tail = cb->tail_ptr;
  uint8_t* bgn = cb->buff_ptr;
  uint8_t* end = cb->end_ptr;
  uint8_t* new_tail;
  uint8_t* head;
  uint8_t is_empty;
  uint32_t prim;
  prim = __get_PRIMASK();
  __disable_irq();
  head = cb->head_ptr;
  if(!prim)
    __enable_irq();
  is_empty = head == tail;
  if(is_empty)
    return 0;
  *b = *tail;
  tail++;
  new_tail = tail < end ? tail : bgn;
  prim = __get_PRIMASK();
  __disable_irq();
  cb->tail_ptr = new_tail;
  if(!prim)
    __enable_irq();
  return 1;
}

