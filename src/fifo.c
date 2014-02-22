/*
 *  B-Queue -- An efficient and practical queueing for fast core-to-core
 *             communication
 *
 *  Copyright (C) 2011 Junchang Wang <junchang.wang@gmail.com>
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "fifo.h"
#include <sched.h>

#if defined(FIFO_DEBUG)
#include <assert.h>
#endif

/* get current time  */
inline uint64_t read_tsc()
{
        uint64_t        time;
        uint32_t        msw   , lsw;
        __asm__         __volatile__("rdtsc\n\t"
                        "movl %%edx, %0\n\t"
                        "movl %%eax, %1\n\t"
                        :         "=r"         (msw), "=r"(lsw)
                        :   
                        :         "%edx"      , "%eax");
        time = ((uint64_t) msw << 32) | lsw;
        return time;
}


inline void wait_ticks(uint64_t ticks)
{
        uint64_t        current_time;
        uint64_t        time = read_tsc();
        time += ticks;
        do {
                current_time = read_tsc();
        } while (current_time < time);
}

/* uint64_t */
static ELEMENT_TYPE ELEMENT_ZERO = 0x0UL;

/*************************************************/
/********** Queue Functions **********************/
/*************************************************/

void queue_init(struct queue_t *q)
{
	memset(q, 0, sizeof(struct queue_t));
#if defined(CONS_BATCH)
	q->batch_history = CONS_BATCH_SIZE;
#endif
}

/* compare tow points wheather they equal to each other*/
#if defined(PROD_BATCH) || defined(CONS_BATCH)
inline int leqthan(volatile ELEMENT_TYPE point, volatile ELEMENT_TYPE batch_point)
{
	return (point == batch_point);
}
#endif


#if defined(PROD_BATCH)
int enqueue(struct queue_t * q, ELEMENT_TYPE value)
{
	uint32_t tmp_head;
	/* if there is no space for producer*/
	if( q->head == q->batch_head ) {
		// move head forward
		tmp_head = q->head + PROD_BATCH_SIZE;
		// if overhead set to 0
		if ( tmp_head >= QUEUE_SIZE )
			tmp_head = 0;
		// if the destination is full wait.
		if ( q->data[tmp_head] ) {
			wait_ticks(CONGESTION_PENALTY);
			return BUFFER_FULL;
		}
		// else change bathch_head to the next step
		q->batch_head = tmp_head;
	}
	
	// enqueue
	q->data[q->head] = value;
	q->head ++;
	// adjust head
	if ( q->head >= QUEUE_SIZE ) {
		q->head = 0;
	}

	return SUCCESS;
}
#else
int enqueue(struct queue_t * q, ELEMENT_TYPE value)
{
	// if current head is full then return
	if ( q->data[q->head] )
		return BUFFER_FULL;
	// else enqueue
	q->data[q->head] = value;
	q->head ++;
	// adjust head
	if ( q->head >= QUEUE_SIZE ) {
		q->head = 0;
	}

	return SUCCESS;
}
#endif

#if defined(CONS_BATCH)

static inline int backtracking(struct queue_t * q)
{
	uint32_t tmp_tail;
	// get next batch_tail
	tmp_tail = q->tail + CONS_BATCH_SIZE;
	// if next batch_tail is lager than queue_size then adjust.
	if ( tmp_tail >= QUEUE_SIZE ) {
		tmp_tail = 0;
#if defined(ADAPTIVE)
		// if history is smaller then adjust history
		if (q->batch_history < CONS_BATCH_SIZE) {
			q->batch_history = 
				// if history+increment > batch_size than history is batch_size else is history+increment
				// to make sure history no more than batch_size
				(CONS_BATCH_SIZE < (q->batch_history + BATCH_INCREAMENT))? 
				CONS_BATCH_SIZE : (q->batch_history + BATCH_INCREAMENT);
		}
#endif
	}

#if defined(BACKTRACKING)

	// uodate current batch_size to history
	unsigned long batch_size = q->batch_history;
	// if tmp_tail is empty then loop
	while (!(q->data[tmp_tail])) {
		// wait a moment
		wait_ticks(CONGESTION_PENALTY);
		// half the batch_size
		batch_size = batch_size >> 1;
		// if batch_size >= 0 then modify the tmp_tail
		if( batch_size >= 0 ) {
			tmp_tail = q->tail + batch_size;
			if (tmp_tail >= QUEUE_SIZE)
				tmp_tail = 0;		
		// then go to the next loop until it is not empty
		// so that the comsumer can read from the queue
		}
		else
			return -1;
	}
#if defined(ADAPTIVE)
	q->batch_history = batch_size;
#endif

// else no BACKTRACKING
#else
	// wait until the element is not empty
	if ( !q->data[tmp_tail] ) {
		wait_ticks(CONGESTION_PENALTY); 
		return -1;
	}
#endif  /* end BACKTRACKING */
	
	// it indecats that no space is available to read if tmp_tail == tail
	// because each time tmp_tail will forwarad at most batch_size
	if ( tmp_tail == q->tail ) {
		tmp_tail = (tmp_tail + 1) >= QUEUE_SIZE ?
			0 : tmp_tail + 1;
	}
	// update batch_tail
	q->batch_tail = tmp_tail;

	return 0;
}

int dequeue(struct queue_t * q, ELEMENT_TYPE * value)
{
	// if tail go to the end of this batch
	if( q->tail == q->batch_tail ) {
		// get a backracing to update batch_tail
		if ( backtracking(q) != 0 )
			return BUFFER_EMPTY;
	}
	
	// else dqueue
	*value = q->data[q->tail];
	q->data[q->tail] = ELEMENT_ZERO;
	q->tail ++;
	if ( q->tail >= QUEUE_SIZE )
		q->tail = 0;

	return SUCCESS;
}

// no CONS_BATCH
#else

int dequeue(struct queue_t * q, ELEMENT_TYPE * value)
{
	// if tail is empty then return empty
	if ( !q->data[q->tail] )
		return BUFFER_EMPTY;
	// else dequeue
	*value = q->data[q->tail];
	q->data[q->tail] = ELEMENT_ZERO;
	q->tail ++;
	// adjust tail
	if ( q->tail >= QUEUE_SIZE )
		q->tail = 0;

	return SUCCESS;
}

#endif  /* end of CONS_BATCH */
