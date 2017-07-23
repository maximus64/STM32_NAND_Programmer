/*
 * header.h
 *
 *  Created on: Jul 20, 2017
 *      Author: maximus
 */

#ifndef HELPER_H_
#define HELPER_H_

#include <stdio.h>

#define RECV_BUFFER_SIZE (4096+224)
void receive_buff_init();
void receive_buff_queue(uint8_t* buf, size_t len);
int receive_buff_dequeue(uint8_t* buf, size_t len);
size_t receive_buff_size();

#endif /* HELPER_H_ */
