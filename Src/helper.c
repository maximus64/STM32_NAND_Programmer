/*
 * helper.c
 *
 *  Created on: Jul 20, 2017
 *      Author: maximus
 */
#include "helper.h"

static uint8_t receiveBuf[RECV_BUFFER_SIZE];
static size_t receiveHead;
static size_t receiveTail;


void receive_buff_init()
{
	receiveHead = 0;
	receiveTail = 0;
}

void receive_buff_queue(uint8_t* buf, size_t len)
{
	size_t i;
	for(i=0;i<len;i++)
	{
		receiveBuf[receiveHead] = buf[i];
		receiveHead=(receiveHead+1) % RECV_BUFFER_SIZE;
	}
}

int receive_buff_dequeue(uint8_t* buf, size_t len)
{
	size_t i;
	if (len > receive_buff_size())
		return -1;

	for(i=0;i<len;i++)
	{
		buf[i] = receiveBuf[receiveTail];
		receiveTail=(receiveTail+1) % RECV_BUFFER_SIZE;
	}

	return 0;
}

size_t receive_buff_size()
{
	return (receiveHead-receiveTail) % RECV_BUFFER_SIZE;
}

