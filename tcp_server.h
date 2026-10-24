/*
 * tcp_server.h
 *
 *  Created on: 11 juil. 2022
 *      Author: Madben
 */

#ifndef TCP_SERVER_H_
#define TCP_SERVER_H_




#include "lwip/opt.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"





#ifndef TCP_SERVER_DEBUG
#define TCP_SERVER_DEBUG         LWIP_DBG_OFF
#endif

/** Maximum retries before the connection is aborted/closed.
 * - number of times pcb->poll is called -> default is 4*500ms = 2s;
 * - reset when pcb->sent is called
 */
#ifndef TCP_SERVER_MAX_RETRIES
#define TCP_SERVER_MAX_RETRIES                   4
#endif

/** The poll delay is X*500ms */
#ifndef TCP_SERVER_POLL_INTERVAL
#define TCP_SERVER_POLL_INTERVAL                 20
#endif

/** Priority for tcp pcbs created by TCP_SERVER (very high by default).
 *  Lower priorities get killed first when running out of memory.
 */
#ifndef TCP_SERVER_PRIO_LISTEN
#define TCP_SERVER_PRIO_LISTEN                      TCP_PRIO_MIN
#endif

#ifndef TCP_SERVER_PRIO_ACCEPTED
#define TCP_SERVER_PRIO_ACCEPTED                    TCP_PRIO_MAX
#endif



bool REPLY_FROM_TCP_WANTED;
bool BUFFERED_MODE_JUST_CHANGED;
bool BUFFERED_MODE;


extern void tcp_server_init(void);


#endif /* TCP_SERVER_H_ */
