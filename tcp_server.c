/*
 * tcp_server.c
 *
 *  Created on: 11 juil. 2022
 *      Author: Madben
 */

#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include <stdbool.h>
#include "utils/ustdlib.h"
#include <stdlib.h>
#include <stdint.h>

#include "tcp_server.h"
//#include "AD9852.h"
#include "AD9959.h"
#include "ARM TM4C1294.h"


//#include "driverlib/sysctl.h" // to remove when sysclock msg not needed anymore



#if LWIP_TCP



enum tcp_server_states
{
  SS_NONE = 0,
  SS_ACCEPTED,
  SS_RECEIVED,
  SS_CLOSING,
  SS_WAITING_FOR_CLIENT_TO_CLOSE
};





char msg[TCP_MSG_SIZE];
char msg_DDS_nb[TCP_MSG_SIZE];
char msg_DDS_channel_nb[TCP_MSG_SIZE];
char msg_command[TCP_MSG_SIZE];
char msg_value[TCP_MSG_SIZE];
char msg_time[TCP_MSG_SIZE];
char msg_reply[TCP_MSG_SIZE];

//char msg1[TCP_MSG_SIZE];
//char msg2[TCP_MSG_SIZE];
//char msg3[TCP_MSG_SIZE];
//char msg4[TCP_MSG_SIZE];


char *pDollar;
char *pDollar2;
char *pColon;
char *pStar;
char *pHashtag;
char *pindexHt;


u16_t index;
u16_t indexDollar;
u16_t indexDollar2;
u16_t indexStar;
u16_t indexColon;
u16_t indexHashtag;
u16_t indexAt;
u16_t indexHt;

char dollarCom[1];
char starCom[1];
char colonCom[1];
char hashtagCom[1];
char atCom[1];




/* structure for maintaining connection infos to be passed as argument
   to LwIP callbacks*/
struct tcp_server_struct
{
  u8_t state;             /* current connection state */
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
  u8_t retries;
};




static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_server_err(void *arg, err_t err);
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_server_send(struct tcp_pcb *tpcb, struct tcp_server_struct *serv_struc);
static err_t tcp_server_connection_close(struct tcp_pcb *tpcb, struct tcp_server_struct *serv_struc);



static bool parse_received_msg(struct pbuf *p);
static void cleanCharTable(char* msg_to_clean, int size_of_msg);


/**
  * @brief  This function is used to send data for TCP connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  serv_struc: pointer on server_state structure
  * @retval None
  */
static void tcp_server_send(struct tcp_pcb *tpcb, struct tcp_server_struct *serv_struc)
{
    struct pbuf *ptr;
    err_t wr_err = ERR_OK;
    //err_t out_err = ERR_OK;

    while ((wr_err == ERR_OK) &&
            (serv_struc->p != NULL) &&
            (serv_struc->p->len <= tcp_sndbuf(tpcb)))
    {
        /* get pointer on pbuf from serv_struc structure */
        ptr = serv_struc->p;

        /* enqueue data for transmission */
        wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);

        if (wr_err == ERR_OK)
        {
            // out_err = tcp_output(tpcb); //BP: try to add it to send data as fast as possible?

            u16_t plen;
            plen = ptr->len;

            /* continue with next pbuf in chain (if any) */
            serv_struc->p = ptr->next;
            if(serv_struc->p != NULL)
            {
                /* increment reference count for serv_struc->p */
                pbuf_ref(serv_struc->p);
            }
            /* free pbuf: will free pbufs up to serv_struc->p (because serv_struc->p has a reference count > 0) */
            pbuf_free(ptr);

            /* Update tcp window size to be advertised : should be called when received
              data (with the amount plen) has been processed by the application layer */
            tcp_recved(tpcb, plen);
        }
        else if(wr_err == ERR_MEM)
        {
            /* we are low on memory, try later / harder, defer to poll */
            serv_struc->p = ptr;
        }
        else
        {
            /* other problem ?? */
        }
  }
}


/**
  * @brief  This function implements the tcp_sent LWIP callback (called when ACK
  *         is received from remote host for sent data)
  * @param  arg: pointer on a argument for the tcp_pcb connection
  * @retval tpcb: pointer on the tcp_pcb connection
  * @param  len:
  * @retval err_t: error code
  */
static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    struct tcp_server_struct *serv_struc;

    LWIP_UNUSED_ARG(len);

    serv_struc = (struct tcp_server_struct *)arg;


    if(serv_struc->p != NULL)
    {

        /* still got pbufs to send */
        tcp_server_send(tpcb, serv_struc);
    }
    else
    {
        /* if no more data to send and client closed connection*/
        if(serv_struc->state == SS_CLOSING)
        {
            tcp_server_connection_close(tpcb, serv_struc);
        }
    }
  return ERR_OK;
}


/**
  * @brief  This function is the implementation for tcp_recv LwIP callback
  * @param  arg: pointer on a argument for the tcp_pcb connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  p: pointer on the received pbuf
  * @param  err: error information regarding the received pbuf
  * @retval err_t: error code
  */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *pIncoming, err_t err)
{
    struct tcp_server_struct *serv_struc;
    err_t ret_err;


    //usprintf(msg1, "Test 1 is super working");
    //usprintf(msg2, "Test 2 is super working");
    //usprintf(msg3, "Test 3 is super working");
    //usprintf(msg4, "Test 4 is super working");

    LWIP_ASSERT("arg != NULL", arg != NULL);

    serv_struc = (struct tcp_server_struct *)arg;

    /* if we receive an empty TCP frame from client => close connection */
    if (pIncoming == NULL)
    {
        /* remote host closed connection */
        serv_struc->state = SS_CLOSING;
        //serv_struc->state = SS_WAITING_FOR_CLIENT_TO_CLOSE;
        if(serv_struc->p == NULL)
        {
            /* we're done sending, close connection */
            tcp_server_connection_close(tpcb, serv_struc); // closing the connection is the role of the client, the server should listen always, unless not told to
        }
        else
        {
            //struct pbuf *ptr2;
            //ptr2 = serv_struc->p;
            //memcpy(ptr2->payload, msg3, sizeof(msg3)); // not very useful if the client waits correctly for the answer

            /* free pbuf and do nothing */
            //serv_struc->p = NULL;
            //tcp_server_connection_close(tpcb, serv_struc);

            ////////////////
            // for echo only
            ////////////////
            /* we're not done yet */
            /* acknowledge received packet */
            tcp_sent(tpcb, tcp_server_sent);

            /* send remaining data*/
            tcp_server_send(tpcb, serv_struc);
            ////////////////
        }
        ret_err = ERR_OK;
    }
    /* else : a non empty frame was received from client but for some reason err != ERR_OK */
    else if(err != ERR_OK)
    {
        /* free received pbuf*/
        if (pIncoming != NULL)
        {
            serv_struc->p = NULL;
            pbuf_free(pIncoming);
        }
        ret_err = err;
    }
    else if(serv_struc->state == SS_ACCEPTED)
    {
        /* first data chunk in p->payload */
        serv_struc->state = SS_RECEIVED;

        /* store reference to incoming pbuf (chain) */
        //serv_struc->p = pIncoming; // proper code from before


        // first message will be processed via this "if" condition
        parse_received_msg(pIncoming);
        pbuf_free(pIncoming);

        struct pbuf *pParsed;
        pParsed = pbuf_alloc(PBUF_TRANSPORT, sizeof(msg_reply), PBUF_POOL);
        memcpy(pParsed->payload, msg_reply, sizeof(msg_reply));
        serv_struc->p = pParsed;

        ////////////////
        // for echo only
        ////////////////
        /* initialize LwIP tcp_sent callback function */
        tcp_sent(tpcb, tcp_server_sent);

        tcp_server_send(tpcb, serv_struc);
        ////////////////

        ret_err = ERR_OK;
    }
    else if (serv_struc->state == SS_RECEIVED)
    {
        /* more data received from client and previous data has been already sent*/

        parse_received_msg(pIncoming);
        pbuf_free(pIncoming);

        struct pbuf *pParsed2;
        pParsed2 = pbuf_alloc(PBUF_TRANSPORT, sizeof(msg_reply), PBUF_POOL);
        memcpy(pParsed2->payload, msg_reply, sizeof(msg_reply));

        if(serv_struc->p == NULL)
        {
            // most messages, 2nd and others, will be processed via this "if" condition
            //serv_struc->p = pIncoming;   // proper code from before
            serv_struc->p = pParsed2;

            ////////////////
            // for echo only
            ////////////////
            /* send back received data */
            tcp_server_send(tpcb, serv_struc);
            ////////////////
        }
        else
        {
            /* chain pbufs to the end of what we received previously  */
            struct pbuf *ptr;
            ptr = serv_struc->p;
            pbuf_chain(ptr, pParsed2);
            //pbuf_chain(ptr, pIncoming); // proper code from before
        }
        ret_err = ERR_OK;
    }
    /* data received when connection already closed */
    else
    {
        /* Acknowledge data reception */

        //pbuf_realloc(p, sizeof(msg4));
        //tcp_write(tpcb, msg4, sizeof(msg4), 1);

        tcp_recved(tpcb, pIncoming->tot_len);

        /* free pbuf and do nothing */
        serv_struc->p = NULL;
        pbuf_free(pIncoming);
        ret_err = ERR_OK;
    }
    return ret_err;
}


static void cleanCharTable(char* msg_to_clean, int size_of_msg)
{
    int count = 0;
    while(count < size_of_msg)
    {
        msg_to_clean[count] = '\0';
        count=count+1;
    }
}



static bool parse_received_msg(struct pbuf *p)
{

    unsigned int DDS_nb_value;

    index = pbuf_strstr(p, dollarCom);
    if (index!=0xFFFF)
    {
        indexDollar = index;
    }
    index = pbuf_strstr(p, starCom);
    if (index!=0xFFFF)
    {
        indexStar = index;
    }
    index = pbuf_strstr(p, colonCom);
    if (index!=0xFFFF)
    {
        indexColon = index;
    }
    index = pbuf_strstr(p, hashtagCom);
    if (index!=0xFFFF)
    {
        indexHashtag = index;
    }


    //if (p->tot_len > sizeof(msg))
    //{
    //    LWIP_DEBUGF(TCP_SERVER_DEBUG, ("parse_received_msg: pBuf payload is bigger than alloted msg size\n"));
    //    return false;
    //}


    cleanCharTable(msg, TCP_MSG_SIZE);
    cleanCharTable(msg_DDS_nb, TCP_MSG_SIZE);
    cleanCharTable(msg_command, TCP_MSG_SIZE);
    cleanCharTable(msg_value, TCP_MSG_SIZE);
    cleanCharTable(msg_reply, TCP_MSG_SIZE);


    memcpy(msg, p->payload, p->tot_len);

    pDollar = ustrstr(msg, dollarCom);
    pStar = ustrstr(msg, starCom);
    pColon = ustrstr(msg, colonCom);
    pHashtag = ustrstr(msg, hashtagCom);



    ustrncpy(msg_DDS_nb, pDollar+1, indexStar-indexDollar-1);
    ustrncpy(msg_command, pStar+1, indexColon-indexStar-1);
    ustrncpy(msg_value, pColon+1, indexHashtag-indexColon-1);






    if (BUFFERED_MODE)
    {
        /*if (ustrcmp(msg_command, "BuffOFF") == 0)
        {
            BUFFERED_MODE = false;
            //BUFFERED_MODE_JUST_CHANGED = true;
            if (REPLY_FROM_TCP_WANTED) { usprintf(msg_reply, "Manual mode activated"); }
        }
        else
        {
            index = pbuf_strstr(p, atCom);
            if (index!=0xFFFF)
            {
                indexAt = index;
            }
            cleanCharTable(msg_time, TCP_MSG_SIZE);
            ustrncpy(msg_time, pHashtag+1, indexAt-indexHashtag-1);
            //ustrncpy(msg_time, pDollar2+1, indexAt-indexDollar2-1);
            acquireBufferedSequence(msg_command, msg_value, msg_DDS_nb, msg_DDS_channel_nb, msg_time, msg_reply, REPLY_FROM_TCP_WANTED, BUFFERED_MODE_JUST_CHANGED);
            //acquireBufferedSequence(msg_command, msg_value, msg_DDS_nb, msg_time, msg_reply, REPLY_FROM_TCP_WANTED, BUFFERED_MODE_JUST_CHANGED);
            BUFFERED_MODE_JUST_CHANGED = false;
        }
        */
    }
    else
    {
        if (ustrcmp(msg_command, "Trig") == 0)
        {
           //launchBufferedSequence();
            if (REPLY_FROM_TCP_WANTED)
            {
                usprintf(msg_reply, "Triggered sequence done");
                //usprintf(msg_reply, "System clock: %x or %x", SysCtlClockGet(), g_ui32SysClock);
                //usprintf(msg_reply, "System clock: %x", SysCtlClockGet());
                //usprintf(msg_reply, "System clock: %x ", g_ui32SysClock);
            }
        }
        else if (ustrcmp(msg_command, "Ready?") == 0)
        {
            usprintf(msg_reply, "Yes");
        }
        else if (ustrcmp(msg_command, "BuffON") == 0)
        {
            BUFFERED_MODE = true;
            BUFFERED_MODE_JUST_CHANGED = true;
            if (REPLY_FROM_TCP_WANTED) { usprintf(msg_reply, "Buffered mode activated"); }
        }
        else if (ustrcmp(msg_command, "Reset") == 0)
        {
            start_DDSSequence();
            if (REPLY_FROM_TCP_WANTED) { usprintf(msg_reply, "DDSs reset"); }
        }
        else if  (ustrcmp(msg_command, "Reply") == 0)
        {
            REPLY_FROM_TCP_WANTED = (bool) (0b1 & atoi(msg_value));
            if (REPLY_FROM_TCP_WANTED)
            {
                usprintf(msg_reply, "Replies enabled");
            }
            else
            {
                usprintf(msg_reply, "Replies disabled");
            }
        }
        else // La commande est envoy�e au DDS (incluant PHASE, FTW1, FSTEP, TSTEP)
        {
            DDS_nb_value = atoi(msg_DDS_nb);

            if (!choiceNbDDS((unsigned char)DDS_nb_value, false))
            {
                usprintf(msg_reply, "DDS number not well set");
                return false;
            }
            else if (TYPEDDS[DDS_nb_value-1] == 1 )
            {
                //getAD9852CommandFromTCP(msg_command, msg_value, msg_reply, DDS_nb_value, REPLY_FROM_TCP_WANTED);
                //getAD9959CommandFromTCP(msg_command, msg_value, msg_reply, DDS_nb_value, DDS_channel_value, REPLY_FROM_TCP_WANTED);
                getAD9959CommandFromTCP(msg_command, msg_value, msg_reply, DDS_nb_value, REPLY_FROM_TCP_WANTED);
            }
            else if (TYPEDDS[DDS_nb_value-1] == 2)
            {
                // ToDo: implement AD9858
                usprintf(msg_reply, "DDS type not implemented yet");
                return false;
            }
            else
            {
                usprintf(msg_reply, "DDS type None or not defined");
                return false;
            }
        }
    }


    pbuf_free(p);


    return true;


}


/**
  * @brief  This function implements the tcp_poll LwIP callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: pointer on the tcp_pcb for the current tcp connection
  * @retval err_t: error code
  */
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
    err_t ret_err;
    struct tcp_server_struct *serv_struc;

    serv_struc = (struct tcp_server_struct *)arg;
    LWIP_DEBUGF(TCP_SERVER_DEBUG | LWIP_DBG_TRACE, ("tcp_server_poll: pcb=%p ss=%p pcb_state=%s\n",
            (void*)pcb, (void*)serv_struc, tcp_debug_state_str(tpcb->state)));

    if (serv_struc != NULL)
    {
        if (serv_struc->state != SS_WAITING_FOR_CLIENT_TO_CLOSE)
        {
            serv_struc->retries++;
            if (serv_struc->retries == TCP_SERVER_MAX_RETRIES) {
              LWIP_DEBUGF(TCP_SERVER_DEBUG, ("tcp_server_poll: too many retries, close\n"));
              tcp_server_connection_close(tpcb, serv_struc);
              return ERR_OK;
            }
            if (serv_struc->p != NULL)
            {
                /* there is a remaining pbuf (chain) , try to send data */
                LWIP_DEBUGF(TCP_SERVER_DEBUG | LWIP_DBG_TRACE, ("tcp_server_poll: try to send more data\n"));
                tcp_server_send(tpcb, serv_struc);
            }
            else
            {
                /* no remaining pbuf (chain)  */
                if (serv_struc->state == SS_CLOSING)
                {
                    /*  close tcp connection */
                    tcp_server_connection_close(tpcb, serv_struc);
                }
            }
        }
        ret_err = ERR_OK;
    }
    else
    {
        err_t closed;
        LWIP_DEBUGF(TCP_SERVER_DEBUG, ("tcp_server_poll: arg is NULL, close\n"));
        closed = tcp_server_connection_close(tpcb, serv_struc);
        LWIP_UNUSED_ARG(closed);

        /* nothing to be done (tcp echo version) */
        //tcp_abort(tpcb);
        //ret_err = ERR_ABRT;

        ret_err = ERR_OK;
    }
    return ret_err;
}


/**
  * @brief  This functions closes the TCP connection
  * @param  tpcb: pointer on the TCP connection
  * @param  serv_struc: pointer on server_state structure
  * @retval err_t: error code
  */
static err_t tcp_server_connection_close(struct tcp_pcb *tpcb, struct tcp_server_struct *serv_struc)
{
    err_t err;
    LWIP_DEBUGF(TCP_SERVER_DEBUG, ("Closing connection %p\n", (void*)tpcb));

    /* remove all callbacks */
    tcp_arg(tpcb, NULL);
    tcp_sent(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_err(tpcb, NULL);
    tcp_poll(tpcb, NULL, 0);

    /* delete serv_struc structure */
    if (serv_struc != NULL)
    {
        mem_free(serv_struc);
    }

    /* close TCP connection */
    err = tcp_close(tpcb);

    if (err != ERR_OK) {
        LWIP_DEBUGF(TCP_SERVER_DEBUG, ("Error %d closing %p\n", err, (void*)tpcb));
        /* error closing, try again later in poll */
        tcp_poll(tpcb, tcp_server_poll, TCP_SERVER_POLL_INTERVAL);
    }
    return err;
}


/**
  * @brief  This function implements the tcp_err callback function (called
  *         when a fatal tcp_connection error occurs.
  * @param  arg: pointer on argument parameter
  * @param  err: not used
  * @retval None
  */
static void tcp_server_err(void *arg, err_t err)
{
    struct tcp_server_struct *serv_struc;
    serv_struc = (struct tcp_server_struct *)arg;

    LWIP_UNUSED_ARG(err);

    LWIP_DEBUGF(TCP_SERVER_DEBUG, ("tcp_server_err: %s", lwip_strerr(err)));

    if (serv_struc != NULL)
    {
        /*  free serv_struc structure */
        mem_free(serv_struc);
    }
}


/**
  * @brief  This function is the implementation of tcp_accept LwIP callback, when
  *         a new incoming connection has been accepted.
  * @param  arg: not used
  * @param  newpcb: pointer on tcp_pcb struct for the newly created tcp connection
  * @param  err: not used
  * @retval err_t: error status
  */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    LWIP_DEBUGF(TCP_SERVER_DEBUG, ("tcp_server_accept %p / %p\n", (void*)newpcb, arg));

    struct tcp_pcb_listen *lpcb = (struct tcp_pcb_listen*)arg;

    err_t ret_err;
    struct tcp_server_struct *serv_struc;

    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(err);


    /* Decrease the listen backlog counter */
    tcp_accepted(lpcb);

    /* set priority for the newly accepted TCP connection newpcb */
    tcp_setprio(newpcb, TCP_SERVER_PRIO_ACCEPTED);


  /* allocate structure serv_struc to maintain TCP connection informations */
    serv_struc = (struct tcp_server_struct *)mem_malloc(sizeof(struct tcp_server_struct));
    if (serv_struc != NULL)
    {
        serv_struc->state = SS_ACCEPTED;
        serv_struc->pcb = newpcb;
        serv_struc->p = NULL;
        serv_struc->retries = 0;

        /* pass newly allocated serv_struc structure as argument to newpcb */
        tcp_arg(newpcb, serv_struc);

        /* initialize LWIP tcp_recv callback function for newpcb  */
        tcp_recv(newpcb, tcp_server_recv);

        /* initialize LWIP tcp_err callback function for newpcb  */
        tcp_err(newpcb, tcp_server_err);

        /* initialize LWIP tcp_poll callback function for newpcb */
        tcp_poll(newpcb, tcp_server_poll, TCP_SERVER_POLL_INTERVAL);

        //tcp_sent(newpcb, tcp_server_sent); // not needed for this server only, does only replies and thus this is initialized in the tcp_server_recv function.

        ret_err = ERR_OK;
    }
    else
    {
        LWIP_DEBUGF(TCP_SERVER_DEBUG, ("tcp_server_accept: Out of memory, RST\n"));
        /* return memory error */
        ret_err = ERR_MEM;
    }
    return ret_err;
}


/**
  * @brief  Initializes the tcp server with the specified local address and port.
  * @param  None
  * @retval None
  */
void tcp_server_init(void)
{
    usprintf(dollarCom, "$");
    usprintf(starCom, "*");
    usprintf(colonCom, ":");
    usprintf(hashtagCom, "#");
    usprintf(atCom, "@");

    BUFFERED_MODE_JUST_CHANGED = false;
    BUFFERED_MODE = false;
    REPLY_FROM_TCP_WANTED = true;



    LWIP_DEBUGF(TCP_SERVER_DEBUG, ("tcp_server_init\n"));
    struct tcp_pcb *tcp_server_pcb;

    /* create new tcp pcb */
    tcp_server_pcb = tcp_new();
    LWIP_ASSERT("tcp_server_init: tcp_new failed", tcp_server_pcb != NULL);
    if (tcp_server_pcb != NULL)
    {
        err_t err;

        tcp_setprio(tcp_server_pcb, TCP_SERVER_PRIO_LISTEN);
        /* set SOF_REUSEADDR here to explicitly bind httpd to multiple interfaces */

        /* bind tcp_echoserver_pcb to port TCP_SERVER_SERVER_PORT */
        err = tcp_bind(tcp_server_pcb, IP_ADDR_ANY, TCP_SERVER_SERVER_PORT);
        LWIP_ASSERT("tcp_server_init: tcp_bind failed", err == ERR_OK);

        if (err == ERR_OK)
        {
            /* start tcp listening for echo_pcb */
            tcp_server_pcb = tcp_listen(tcp_server_pcb);
            LWIP_ASSERT("tcp_server_init: tcp_listen failed", tcp_server_pcb != NULL);


            /* initialize callback arg and initialize LwIP tcp_accept callback function */
            tcp_arg(tcp_server_pcb, tcp_server_pcb);
            tcp_accept(tcp_server_pcb, tcp_server_accept);
        }
    }
}



#endif /* LWIP_TCP */

