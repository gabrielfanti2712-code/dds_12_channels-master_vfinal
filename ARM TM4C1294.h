/*
 * ARM TM4C1294.h
 *
 *  Created on: 09/10/25
 *      Author: Wiotte Fabrice
 */

#ifndef ARM_TM4C1294_H_
#define ARM_TM4C1294_H_

#include <stdint.h>
//*************************************************************************************************************
//
// Values that can be changes before uploading program to a particular microcontroller.
//
//*************************************************************************************************************

#define SYSCLKDDS                      ((uint32_t)    511574000UL)    // DDS system clock frequency, in Hz
#define SSICLKDDS                      ((uint32_t)       60000000)    // Serial communication speed for DDS AD9959 is 60MHz
#define SYSCLKMICRO                    ((uint32_t)      120000000)    // Microcontroller system clock frequency, in Hz 120MHz
#define RATIO_CLK_TO_RES               ((uint32_t)             12)    // Make it equal to SYSCLKMICRO / RATIO_CLK_TO_RES


#define TYPEDDS1                       ((int)                   1)    // 0: no DDS / 1: AD9959
#define TYPEDDS2                       ((int)                   1)    // 0: no DDS / 1: AD9959
#define TYPEDDS3                       ((int)                   1)    // 0: no DDS / 1: AD9959
//#define TYPEDDS4                       ((int)                   1)    // 0: no DDS / 1: AD9959

// MAC address
// Sr2 DDS RACK 1: 00:1A:B6:23:09:0D
// Sr2 DDS RACK 3: 00:1A:B6:3B:E5:19
// MACADDRESS1 = 0x00B61A00 and MACADDRESS2 = 0x00740200 => this example gives a MAC address of 00:1A:B6:00:02:74
#define MACADDRESS1                    ((uint32_t)     0x00B61A00)    // need to be changed for every new microcontroller. // value for elec DDS 12 channel
#define MACADDRESS2                    ((uint32_t)     0x0019E53C)    // need to be changed for every new microcontroller. // value for elec DDS 12 channel
//adresse MAC DDS 11 channel : 00:1A:B6:3C:E5:19

// DDS rack  is port 22301, DDS rack  is port 22303
#define TCP_SERVER_SERVER_PORT                              22303     // The server TCP communication port. Need to be changed for every new microcontroller.
#define TCP_MSG_SIZE                   ((int)                 300)    // Could be set smaller than 300

#define BUF_INST1_SIZE                 ((int)                8000)    // Size of buffers allocated to the SRAM for storing the time lines and NbCmdPerTime lines
#define BUF_INST2_SIZE                 ((int)               10000)    // Size of buffers allocated to the SRAM for storing the cmd/DDSnb/value (must be equal or bigger than the buffer BUF_INST1_SIZE)


//#define RUN_HTTP_SERVER                            // Comment this if you don't want to run the http server. Uses RUN_IO_CONTROLS too.
//#define RUN_IO_CONTROLS                            // Used by the http server. Uncomment this if you want to use the AnimTimerIntHandler (io.h). You also need to modify startup_ccs.c
//#define USE_PORTN                                  // Uncomment this if you want to use the PortNIntHandler. You also need to modify startup_ccs.c

//Interrupt priority definitions.  The top 3 bits of these values are
// significant with lower values indicating higher priority interrupts.
#define SYSTICK_INT_PRIORITY                     0x80      // I think this is for getting the IP address from DHCP. Calls LwIPTimer
#define TIMER2_INT_PRIORITY                      0x60      // Timer interrupt for precise timing of commands during triggered sequence. Probably should be highest priority.
#define TIMER1_INT_PRIORITY                      0xA0      // Timer interrupt for general timing or less critical tasks.
#define ETHERNET_TCP_INT_PRIORITY                0xE0      // Ethernet controller for TCP communication. Calls lwIPEthernetIntHandler
#define GPIOL_INT_PRIORITY                       0xF0

#ifdef USE_PORTN
#define GPION_INT_PRIORITY                                   0xC0
#endif
#ifdef RUN_HTTP_SERVER
#define ETHERNET_INT_PRIORITY                                0xE0     // Setting for the HTTP server. Will conflict with ETHERNET_TCP_INT_PRIORITY
#endif
//#define GPIOB_INT_PRIORITY                                 0xF0

//*************************************************************************************************************

//*****************************************************************************
//
// UART0
//
//*****************************************************************************
//
#define UART0_DR_R              (*((volatile unsigned long *)0x4000C000))
#define UART0_FR_R              (*((volatile unsigned long *)0x4000C018))
#define UART0_IBRD_R            (*((volatile unsigned long *)0x4000C024))
#define UART0_FBRD_R            (*((volatile unsigned long *)0x4000C028))
#define UART0_LCRH_R            (*((volatile unsigned long *)0x4000C02C))
#define UART0_CTL_R             (*((volatile unsigned long *)0x4000C030))

#define UART_FR_RXFF            0x00000040  // UART Receive FIFO Full
#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_LCRH_WLEN_8        0x00000060  // 8 bit word length
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_CTL_UARTEN         0x00000001  // UART Enable

//*****************************************************************************
//
// SSI registers (SSI0)
//
//*****************************************************************************
#define SSI0_CR0_R              (*((volatile uint32_t *)0x40008000))
#define SSI0_CR1_R              (*((volatile uint32_t *)0x40008004))
#define SSI0_DR_R               (*((volatile uint32_t *)0x40008008))
#define SSI0_SR_R               (*((volatile uint32_t *)0x4000800C))
#define SSI0_CPSR_R             (*((volatile uint32_t *)0x40008010))
#define SSI0_IM_R               (*((volatile uint32_t *)0x40008014))
#define SSI0_RIS_R              (*((volatile uint32_t *)0x40008018))
#define SSI0_MIS_R              (*((volatile uint32_t *)0x4000801C))
#define SSI0_ICR_R              (*((volatile uint32_t *)0x40008020))
#define SSI0_DMACTL_R           (*((volatile uint32_t *)0x40008024))
#define SSI0_PP_R               (*((volatile uint32_t *)0x40008FC0))
#define SSI0_CC_R               (*((volatile uint32_t *)0x40008FC8))

//*****************************************************************************
//
// SSI registers (SSI1)
//
//*****************************************************************************
#define SSI1_CR0_R              (*((volatile uint32_t *)0x40009000))
#define SSI1_CR1_R              (*((volatile uint32_t *)0x40009004))
#define SSI1_DR_R               (*((volatile uint32_t *)0x40009008))
#define SSI1_SR_R               (*((volatile uint32_t *)0x4000900C))
#define SSI1_CPSR_R             (*((volatile uint32_t *)0x40009010))
#define SSI1_IM_R               (*((volatile uint32_t *)0x40009014))
#define SSI1_RIS_R              (*((volatile uint32_t *)0x40009018))
#define SSI1_MIS_R              (*((volatile uint32_t *)0x4000901C))
#define SSI1_ICR_R              (*((volatile uint32_t *)0x40009020))
#define SSI1_DMACTL_R           (*((volatile uint32_t *)0x40009024))
#define SSI1_PP_R               (*((volatile uint32_t *)0x40009FC0))
#define SSI1_CC_R               (*((volatile uint32_t *)0x40009FC8))

//*****************************************************************************
//
// SSI registers (SSI2)
//
//*****************************************************************************
#define SSI2_CR0_R              (*((volatile uint32_t *)0x4000A000))
#define SSI2_CR1_R              (*((volatile uint32_t *)0x4000A004))
#define SSI2_DR_R               (*((volatile uint32_t *)0x4000A008))
#define SSI2_SR_R               (*((volatile uint32_t *)0x4000A00C))
#define SSI2_CPSR_R             (*((volatile uint32_t *)0x4000A010))
#define SSI2_IM_R               (*((volatile uint32_t *)0x4000A014))
#define SSI2_RIS_R              (*((volatile uint32_t *)0x4000A018))
#define SSI2_MIS_R              (*((volatile uint32_t *)0x4000A01C))
#define SSI2_ICR_R              (*((volatile uint32_t *)0x4000A020))
#define SSI2_DMACTL_R           (*((volatile uint32_t *)0x4000A024))
#define SSI2_PP_R               (*((volatile uint32_t *)0x4000AFC0))
#define SSI2_CC_R               (*((volatile uint32_t *)0x4000AFC8))

//*****************************************************************************
//
// GPIO registers (PORTA)
//
//*****************************************************************************
#define GPIO_PORTA_DATA_R       (*((volatile uint32_t *)0x400583FC))
#define GPIO_PORTA_DIR_R        (*((volatile uint32_t *)0x40058400))
#define GPIO_PORTA_AFSEL_R      (*((volatile uint32_t *)0x40058420))
#define GPIO_PORTA_PUR_R        (*((volatile uint32_t *)0x40058510))
#define GPIO_PORTA_DEN_R        (*((volatile uint32_t *)0x4005851C))
#define GPIO_PORTA_AMSEL_R      (*((volatile uint32_t *)0x40058528))
#define GPIO_PORTA_PCTL_R       (*((volatile uint32_t *)0x4005852C))
#define SYSCTL_PRGPIO_R0        0x00000001  // GPIO Port A Peripheral Ready

// ************************************************ *****************************
//
// Registres GPIO (PORTB)
//
// ************************************************ *****************************
//#define GPIO_PORTB_DATA_BITS_R ((uint32_t * volatile) 0x40059000)
#define GPIO_PORTB_DATA_R (* ((uint32_t * volatile) 0x400593FC))
//#define GPIO_PORTB_DIR_R (* ((uint32_t * volatile) 0x40059400))// old cast
#define GPIO_PORTB_DIR_R (* ((volatile uint32_t * ) 0x40059400)) //cast change by BP
#define GPIO_PORTB_AFSEL_R (* ((volatile uint32_t *) 0x40059420))
#define GPIO_PORTB_PUR_R (* ((volatile uint32_t *) 0x40059510))
#define GPIO_PORTB_DEN_R (* ((volatile uint32_t *) 0x4005951C))
//#define GPIO_PORTB_LOCK_R (* ((uint32_t * volatile) 0x40059520))
//#define GPIO_PORTB_CR_R (* ((volatile uint32_t *) 0x40059524))
#define GPIO_PORTB_AMSEL_R (* ((volatile uint32_t *) 0x40059528))
//#define GPIO_PORTB_PCTL_R (* ((uint32_t * volatile) 0x4005952C))
#define GPIO_PORTB_PCTL_R (*((volatile uint32_t *)0x4000552C))
#define SYSCTL_PRGPIO_R1        0x00000002  // GPIO Port B Peripheral Ready


//*****************************************************************************
//
// GPIO registers (PORTD)
//
//*****************************************************************************
#define GPIO_PORTD_DATA_BITS_R  ((volatile uint32_t *)0x4005B000)
#define GPIO_PORTD_DATA_R       (*((volatile uint32_t *)0x4005B3FC))
#define GPIO_PORTD_DIR_R        (*((volatile uint32_t *)0x4005B400))
#define GPIO_PORTD_IS_R         (*((volatile uint32_t *)0x4005B404))
#define GPIO_PORTD_IBE_R        (*((volatile uint32_t *)0x4005B408))
#define GPIO_PORTD_IEV_R        (*((volatile uint32_t *)0x4005B40C))
#define GPIO_PORTD_IM_R         (*((volatile uint32_t *)0x4005B410))
#define GPIO_PORTD_RIS_R        (*((volatile uint32_t *)0x4005B414))
#define GPIO_PORTD_MIS_R        (*((volatile uint32_t *)0x4005B418))
#define GPIO_PORTD_ICR_R        (*((volatile uint32_t *)0x4005B41C))
#define GPIO_PORTD_AFSEL_R      (*((volatile uint32_t *)0x4005B420))
#define GPIO_PORTD_DR2R_R       (*((volatile uint32_t *)0x4005B500))
#define GPIO_PORTD_DR4R_R       (*((volatile uint32_t *)0x4005B504))
#define GPIO_PORTD_DR8R_R       (*((volatile uint32_t *)0x4005B508))
#define GPIO_PORTD_ODR_R        (*((volatile uint32_t *)0x4005B50C))
#define GPIO_PORTD_PUR_R        (*((volatile uint32_t *)0x4005B510))
#define GPIO_PORTD_PDR_R        (*((volatile uint32_t *)0x4005B514))
#define GPIO_PORTD_SLR_R        (*((volatile uint32_t *)0x4005B518))
#define GPIO_PORTD_DEN_R        (*((volatile uint32_t *)0x4005B51C))
#define GPIO_PORTD_LOCK_R       (*((volatile uint32_t *)0x4005B520))
#define GPIO_PORTD_CR_R         (*((volatile uint32_t *)0x4005B524))
#define GPIO_PORTD_AMSEL_R      (*((volatile uint32_t *)0x4005B528))
#define GPIO_PORTD_PCTL_R       (*((volatile uint32_t *)0x4005B52C))
#define GPIO_PORTD_ADCCTL_R     (*((volatile uint32_t *)0x4005B530))
#define GPIO_PORTD_DMACTL_R     (*((volatile uint32_t *)0x4005B534))
#define GPIO_PORTD_SI_R         (*((volatile uint32_t *)0x4005B538))
#define GPIO_PORTD_DR12R_R      (*((volatile uint32_t *)0x4005B53C))
#define GPIO_PORTD_WAKEPEN_R    (*((volatile uint32_t *)0x4005B540))
#define GPIO_PORTD_WAKELVL_R    (*((volatile uint32_t *)0x4005B544))
#define GPIO_PORTD_WAKESTAT_R   (*((volatile uint32_t *)0x4005B548))
#define GPIO_PORTD_PP_R         (*((volatile uint32_t *)0x4005BFC0))
#define GPIO_PORTD_PC_R         (*((volatile uint32_t *)0x4005BFC4))

//*****************************************************************************
//
// GPIO registers (PORTH)
//
//*****************************************************************************
#define GPIO_PORTH_DATA_R       (*((volatile uint32_t *)0x4005F3FC))
#define GPIO_PORTH_DIR_R        (*((volatile uint32_t *)0x4005F400))
#define GPIO_PORTH_AFSEL_R      (*((volatile uint32_t *)0x4005F420))
#define GPIO_PORTH_PUR_R        (*((volatile uint32_t *)0x4005F510))
#define GPIO_PORTH_DEN_R        (*((volatile uint32_t *)0x4005F51C))
#define GPIO_PORTH_AMSEL_R      (*((volatile uint32_t *)0x4005F528))
#define GPIO_PORTH_PCTL_R       (*((volatile uint32_t *)0x4005F52C))
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_RCGCGPIO_R7      0x00000080  // GPIO Port H Run Mode Clock
#define SYSCTL_PRGPIO_R7        0x00000080  // GPIO Port H Peripheral Ready

//*****************************************************************************
//
// GPIO registers (PORTN)
//
//*****************************************************************************
#define GPIO_PORTN_DATA_BITS_R    ((volatile uint32_t *)0x40064000)
#define GPIO_PORTN_DATA_R       (*((volatile uint32_t *)0x400643FC))
#define GPIO_PORTN_DIR_R        (*((volatile uint32_t *)0x40064400))
#define GPIO_PORTN_AFSEL_R      (*((volatile uint32_t *)0x40064420))
#define GPIO_PORTN_PUR_R        (*((volatile uint32_t *)0x40064510))
#define GPIO_PORTN_SLR_R        (*((volatile uint32_t *)0x40064518))
#define GPIO_PORTN_DEN_R        (*((volatile uint32_t *)0x4006451C))
#define GPIO_PORTN_LOCK_R       (*((volatile uint32_t *)0x40064520))
#define GPIO_PORTN_CR_R         (*((volatile uint32_t *)0x40064524))
#define GPIO_PORTN_AMSEL_R      (*((volatile uint32_t *)0x40064528))
#define GPIO_PORTN_PCTL_R       (*((volatile uint32_t *)0x4006452C))


//*****************************************************************************
//
// GPIO registers (PORTM)
//
//*****************************************************************************
#define GPIO_PORTM_DATA_BITS_R    ((volatile uint32_t *)0x40063000)
#define GPIO_PORTM_DATA_R       (*((volatile uint32_t *)0x400633FC))
#define GPIO_PORTM_DIR_R        (*((volatile uint32_t *)0x40063400))
#define GPIO_PORTM_IS_R         (*((volatile uint32_t *)0x40063404))
#define GPIO_PORTM_IBE_R        (*((volatile uint32_t *)0x40063408))
#define GPIO_PORTM_IEV_R        (*((volatile uint32_t *)0x4006340C))
#define GPIO_PORTM_IM_R         (*((volatile uint32_t *)0x40063410))
#define GPIO_PORTM_RIS_R        (*((volatile uint32_t *)0x40063414))
#define GPIO_PORTM_MIS_R        (*((volatile uint32_t *)0x40063418))
#define GPIO_PORTM_ICR_R        (*((volatile uint32_t *)0x4006341C))
#define GPIO_PORTM_AFSEL_R      (*((volatile uint32_t *)0x40063420))
#define GPIO_PORTM_DR2R_R       (*((volatile uint32_t *)0x40063500))
#define GPIO_PORTM_DR4R_R       (*((volatile uint32_t *)0x40063504))
#define GPIO_PORTM_DR8R_R       (*((volatile uint32_t *)0x40063508))
#define GPIO_PORTM_ODR_R        (*((volatile uint32_t *)0x4006350C))
#define GPIO_PORTM_PUR_R        (*((volatile uint32_t *)0x40063510))
#define GPIO_PORTM_PDR_R        (*((volatile uint32_t *)0x40063514))
#define GPIO_PORTM_SLR_R        (*((volatile uint32_t *)0x40063518))
#define GPIO_PORTM_DEN_R        (*((volatile uint32_t *)0x4006351C))
#define GPIO_PORTM_LOCK_R       (*((volatile uint32_t *)0x40063520))
#define GPIO_PORTM_CR_R         (*((volatile uint32_t *)0x40063524))
#define GPIO_PORTM_AMSEL_R      (*((volatile uint32_t *)0x40063528))
#define GPIO_PORTM_PCTL_R       (*((volatile uint32_t *)0x4006352C))
#define GPIO_PORTM_ADCCTL_R     (*((volatile uint32_t *)0x40063530))
#define GPIO_PORTM_DMACTL_R     (*((volatile uint32_t *)0x40063534))
#define GPIO_PORTM_SI_R         (*((volatile uint32_t *)0x40063538))
#define GPIO_PORTM_DR12R_R      (*((volatile uint32_t *)0x4006353C))
#define GPIO_PORTM_WAKEPEN_R    (*((volatile uint32_t *)0x40063540))
#define GPIO_PORTM_WAKELVL_R    (*((volatile uint32_t *)0x40063544))
#define GPIO_PORTM_WAKESTAT_R   (*((volatile uint32_t *)0x40063548))
#define GPIO_PORTM_PP_R         (*((volatile uint32_t *)0x40063FC0))
#define GPIO_PORTM_PC_R         (*((volatile uint32_t *)0x40063FC4))

//*****************************************************************************
//
// GPIO registers (PORTL)
//
//*****************************************************************************
#define GPIO_PORTL_DATA_BITS_R    ((volatile uint32_t *)0x40062000)
#define GPIO_PORTL_DATA_R       (*((volatile uint32_t *)0x400623FC))
#define GPIO_PORTL_DIR_R        (*((volatile uint32_t *)0x40062400))
#define GPIO_PORTL_IS_R         (*((volatile uint32_t *)0x40062404))
#define GPIO_PORTL_IBE_R        (*((volatile uint32_t *)0x40062408))
#define GPIO_PORTL_IEV_R        (*((volatile uint32_t *)0x4006240C))
#define GPIO_PORTL_IM_R         (*((volatile uint32_t *)0x40062410))
#define GPIO_PORTL_RIS_R        (*((volatile uint32_t *)0x40062414))
#define GPIO_PORTL_MIS_R        (*((volatile uint32_t *)0x40062418))
#define GPIO_PORTL_ICR_R        (*((volatile uint32_t *)0x4006241C))
#define GPIO_PORTL_AFSEL_R      (*((volatile uint32_t *)0x40062420))
#define GPIO_PORTL_DR2R_R       (*((volatile uint32_t *)0x40062500))
#define GPIO_PORTL_DR4R_R       (*((volatile uint32_t *)0x40062504))
#define GPIO_PORTL_DR8R_R       (*((volatile uint32_t *)0x40062508))
#define GPIO_PORTL_ODR_R        (*((volatile uint32_t *)0x4006250C))
#define GPIO_PORTL_PUR_R        (*((volatile uint32_t *)0x40062510))
#define GPIO_PORTL_PDR_R        (*((volatile uint32_t *)0x40062514))
#define GPIO_PORTL_SLR_R        (*((volatile uint32_t *)0x40062518))
#define GPIO_PORTL_DEN_R        (*((volatile uint32_t *)0x4006251C))
#define GPIO_PORTL_LOCK_R       (*((volatile uint32_t *)0x40062520))
#define GPIO_PORTL_CR_R         (*((volatile uint32_t *)0x40062524))
#define GPIO_PORTL_AMSEL_R      (*((volatile uint32_t *)0x40062528))
#define GPIO_PORTL_PCTL_R       (*((volatile uint32_t *)0x4006252C))
#define GPIO_PORTL_ADCCTL_R     (*((volatile uint32_t *)0x40062530))
#define GPIO_PORTL_DMACTL_R     (*((volatile uint32_t *)0x40062534))
#define GPIO_PORTL_SI_R         (*((volatile uint32_t *)0x40062538))
#define GPIO_PORTL_DR12R_R      (*((volatile uint32_t *)0x4006253C))
#define GPIO_PORTL_WAKEPEN_R    (*((volatile uint32_t *)0x40062540))
#define GPIO_PORTL_WAKELVL_R    (*((volatile uint32_t *)0x40062544))
#define GPIO_PORTL_WAKESTAT_R   (*((volatile uint32_t *)0x40062548))
#define GPIO_PORTL_PP_R         (*((volatile uint32_t *)0x40062FC0))
#define GPIO_PORTL_PC_R         (*((volatile uint32_t *)0x40062FC4))

//*****************************************************************************
//
// GPIO registers (PORTK)
//
//*****************************************************************************
// Adresse physique du PORT K sur le bus rapide (AHB) pour TM4C1294
//#define GPIO_PORTK_AHB_BASE     ((volatile uint32_t *)0x40061000)
#define GPIO_PORTK_DATA_BITS_R  ((volatile uint32_t *)0x40061000)
#define GPIO_PORTK_DATA_R       (*((volatile uint32_t *)0x400613FC))
#define GPIO_PORTK_DIR_R        (*((volatile uint32_t *)0x40061400))
#define GPIO_PORTK_IS_R         (*((volatile uint32_t *)0x40061404))
#define GPIO_PORTK_IBE_R        (*((volatile uint32_t *)0x40061408))
#define GPIO_PORTK_IEV_R        (*((volatile uint32_t *)0x4006140C))
#define GPIO_PORTK_IM_R         (*((volatile uint32_t *)0x40061410))
#define GPIO_PORTK_RIS_R        (*((volatile uint32_t *)0x40061414))
#define GPIO_PORTK_MIS_R        (*((volatile uint32_t *)0x40061418))
#define GPIO_PORTK_ICR_R        (*((volatile uint32_t *)0x4006141C))
#define GPIO_PORTK_AFSEL_R      (*((volatile uint32_t *)0x40061420))
#define GPIO_PORTK_DR2R_R       (*((volatile uint32_t *)0x40061500))
#define GPIO_PORTK_DR4R_R       (*((volatile uint32_t *)0x40061504))
#define GPIO_PORTK_DR8R_R       (*((volatile uint32_t *)0x40061508))
#define GPIO_PORTK_ODR_R        (*((volatile uint32_t *)0x4006150C))
#define GPIO_PORTK_PUR_R        (*((volatile uint32_t *)0x40061510))
#define GPIO_PORTK_PDR_R        (*((volatile uint32_t *)0x40061514))
#define GPIO_PORTK_SLR_R        (*((volatile uint32_t *)0x40061518))
#define GPIO_PORTK_DEN_R        (*((volatile uint32_t *)0x4006151C))
#define GPIO_PORTK_LOCK_R       (*((volatile uint32_t *)0x40061520))
#define GPIO_PORTK_CR_R         (*((volatile uint32_t *)0x40061524))
#define GPIO_PORTK_AMSEL_R      (*((volatile uint32_t *)0x40061528))
#define GPIO_PORTK_PCTL_R       (*((volatile uint32_t *)0x4006152C))
#define GPIO_PORTK_ADCCTL_R     (*((volatile uint32_t *)0x40061530))
#define GPIO_PORTK_DMACTL_R     (*((volatile uint32_t *)0x40061534))
#define GPIO_PORTK_SI_R         (*((volatile uint32_t *)0x40061538))
#define GPIO_PORTK_DR12R_R      (*((volatile uint32_t *)0x4006153C))
#define GPIO_PORTK_WAKEPEN_R    (*((volatile uint32_t *)0x40061540))
#define GPIO_PORTK_WAKELVL_R    (*((volatile uint32_t *)0x40061544))
#define GPIO_PORTK_WAKESTAT_R   (*((volatile uint32_t *)0x40061548))
#define GPIO_PORTK_PP_R         (*((volatile uint32_t *)0x40061FC0))
#define GPIO_PORTK_PC_R         (*((volatile uint32_t *)0x40061FC4))

//*****************************************************************************
//
// GPIO registers (PORTP)
//
//*****************************************************************************
#define GPIO_PORTP_DATA_BITS_R  ((volatile uint32_t *)0x40065000)
#define GPIO_PORTP_DATA_R       (*((volatile uint32_t *)0x400653FC))
#define GPIO_PORTP_DIR_R        (*((volatile uint32_t *)0x40065400))
#define GPIO_PORTP_IS_R         (*((volatile uint32_t *)0x40065404))
#define GPIO_PORTP_IBE_R        (*((volatile uint32_t *)0x40065408))
#define GPIO_PORTP_IEV_R        (*((volatile uint32_t *)0x4006540C))
#define GPIO_PORTP_IM_R         (*((volatile uint32_t *)0x40065410))
#define GPIO_PORTP_RIS_R        (*((volatile uint32_t *)0x40065414))
#define GPIO_PORTP_MIS_R        (*((volatile uint32_t *)0x40065418))
#define GPIO_PORTP_ICR_R        (*((volatile uint32_t *)0x4006541C))
#define GPIO_PORTP_AFSEL_R      (*((volatile uint32_t *)0x40065420))
#define GPIO_PORTP_DR2R_R       (*((volatile uint32_t *)0x40065500))
#define GPIO_PORTP_DR4R_R       (*((volatile uint32_t *)0x40065504))
#define GPIO_PORTP_DR8R_R       (*((volatile uint32_t *)0x40065508))
#define GPIO_PORTP_ODR_R        (*((volatile uint32_t *)0x4006550C))
#define GPIO_PORTP_PUR_R        (*((volatile uint32_t *)0x40065510))
#define GPIO_PORTP_PDR_R        (*((volatile uint32_t *)0x40065514))
#define GPIO_PORTP_SLR_R        (*((volatile uint32_t *)0x40065518))
#define GPIO_PORTP_DEN_R        (*((volatile uint32_t *)0x4006551C))
#define GPIO_PORTP_LOCK_R       (*((volatile uint32_t *)0x40065520))
#define GPIO_PORTP_CR_R         (*((volatile uint32_t *)0x40065524))
#define GPIO_PORTP_AMSEL_R      (*((volatile uint32_t *)0x40065528))
#define GPIO_PORTP_PCTL_R       (*((volatile uint32_t *)0x4006552C))
#define GPIO_PORTP_ADCCTL_R     (*((volatile uint32_t *)0x40065530))
#define GPIO_PORTP_DMACTL_R     (*((volatile uint32_t *)0x40065534))
#define GPIO_PORTP_SI_R         (*((volatile uint32_t *)0x40065538))
#define GPIO_PORTP_DR12R_R      (*((volatile uint32_t *)0x4006553C))
#define GPIO_PORTP_WAKEPEN_R    (*((volatile uint32_t *)0x40065540))
#define GPIO_PORTP_WAKELVL_R    (*((volatile uint32_t *)0x40065544))
#define GPIO_PORTP_WAKESTAT_R   (*((volatile uint32_t *)0x40065548))
#define GPIO_PORTP_PP_R         (*((volatile uint32_t *)0x40065FC0))
#define GPIO_PORTP_PC_R         (*((volatile uint32_t *)0x40065FC4))

                                            // Gating Control
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
//#define SYSCTL_RCGCGPIO_R12     0x00001000  // GPIO Port N Run Mode Clock
#define SYSCTL_PRGPIO_R12       0x00001000  // GPIO Port N Peripheral Ready
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))

#define PA3 (*((volatile unsigned long *)0x40058020))  // CS DDS1
#define PB4 (*((volatile unsigned long *)0x40059040))  // CS DDS2
#define PD2 (*((volatile unsigned long *)0x4005B010))  // CS DDS3
#define PB0 (*((volatile unsigned long *)0x40059004))   // on/off lock

#define PN0 (*((volatile unsigned long *)0x40064004))  //I/O_UPDATE_DDS3
#define PN1 (*((volatile unsigned long *)0x40064008))  //MASTER_RESET_DDS3
#define PN2 (*((volatile unsigned long *)0x40064010))  //PWR_DOWN_DDS3
#define PN3 (*((volatile unsigned long *)0x40064020))  //I/O_UPDATE_DDS2
#define PN4 (*((volatile unsigned long *)0x40064040))  //MASTER_RESET_DDS2
#define PN5 (*((volatile unsigned long *)0x40064080))  //PWR_DOWN_DDS2

#define PM4 (*((volatile unsigned long *)0x40063040))
#define PM5 (*((volatile unsigned long *)0x40063080))
#define PM6 (*((volatile unsigned long *)0x40063100))
#define PM7 (*((volatile unsigned long *)0x40063200))

#define PF0 (*((volatile unsigned long *)0x00050005))

#define PL0 (*((volatile unsigned long *)0x40062004))     //TTL8
#define PL1 (*((volatile unsigned long *)0x40062008))     //TTL7
#define PL2 (*((volatile unsigned long *)0x40062010))     //TTL6
#define PL3 (*((volatile unsigned long *)0x40062020))     //TTL5
#define PL4 (*((volatile unsigned long *)0x40062040))     //TTL4
#define PL5 (*((volatile unsigned long *)0x40062080))     //TTL3
#define PL6 (*((volatile unsigned long *)0x40062100))     //TTL1
#define PL7 (*((volatile unsigned long *)0x40062200))     //TTL2

#define PK5 (*((volatile unsigned long *)0x40061080))    //MASTER_RESET_DDS1
#define PK6 (*((volatile unsigned long *)0x40061100))    //PWR_DOWN_DDS1
#define PK7 (*((volatile unsigned long *)0x40061200))    //I/O_UPDATE_DDS1

#define GPIO_STRENGTH_12MA      0x00000077  // 12mA drive strength


//*****************************************************************************
//
// Flash ROM
//
//*****************************************************************************
// Flash ROM addresses must be 1k byte aligned, e.g., 0x8000, 0x8400, 0x8800...
// For TM4C1294, flash ROM addresses must be 16k byte aligned, e.g., 0x8000, 0x12000, 0x18000...
#define FLASHVALUESDDS1                 0x12000   // location in flash to write; make sure no program code is in this block
#define SIMPLEFLASHVALUESDDS1           0x88000
#define FLASHVALUESDDS2                 0x24000
#define SIMPLEFLASHVALUESDDS2           0x28000
#define FLASHVALUESDDS3                 0x32000
#define SIMPLEFLASHVALUESDDS3           0x36000
#define FLASHVALUESDDS4                 0x44000
#define SIMPLEFLASHVALUESDDS4           0x48000
#define SIMPLEFLASHVALUESDDS5           0x52000
#define SIMPLEFLASHVALUESDDS6           0x56000
#define SIMPLEFLASHVALUESDDS7           0x60000
#define SIMPLEFLASHVALUESDDS8           0x64000
#define SIMPLEFLASHVALUESDDS9           0x68000
#define SIMPLEFLASHVALUESDDS10          0x72000
#define SIMPLEFLASHVALUESDDS11          0x76000
#define SIMPLEFLASHVALUESDDS12          0x80000
#define SIMPLEFLASHVALUESDDS13          0x84000



//*****************************************************************************
//
//  Nouvelle Architecture
//
//******************************************************************************
#define SEQ_FRAME_LOAD  0x03U
#define SEQ_FRAME_ARM   0x04U
#define SEQ_FRAME_STOP  0x05U
#define SEQ_TRIGGER_TTL6 6U
#define SEQ_TRIGGER_TTL7 7U
#define SEQ_TRIGGER_TTL8 8U

// Current values of the DDSs AD9959, tracked by the microcontroller
//extern unsigned long long frequency1ValueAD9959DDS[4];
extern unsigned long frequencyValueAD9959DDS[4];
extern unsigned short amplValueAD9959DDS[4];
extern unsigned short powValueAD9959DDS[4];
unsigned short select_channelDDS[4];
unsigned short channel_DDSAD9852DDS[4];
unsigned short channel_DDSAD9959DDS[4];
bool flagEndOfTimer;

extern unsigned int CURRENT_DDS;           // DDS currently addressed by the microcontroller
extern unsigned int CURRENT_ChannelDDS;    // DDS currently channel by the microcontroller
extern unsigned long aux_CTL_REG[4];       // Value of control register, modified by single bit functions but not yet sent to register

extern const int TYPEDDS[3];

void DisplayIPAddress(uint32_t ui32Addr);
//void Displaydata(uint32_t ui32value);
//void DisplayPortAddress(uint32_t ui32Addr);

void init_UART0(void);
void write_immediate(void);

// DDS functions
bool MasterResetAllDDS(void);
bool extern start_DDSSequence(void);
bool extern IOupdateClk(bool high);
bool extern choiceNbDDS(unsigned int NbDDS, bool firstInit);
bool extern choiceChannelDDS1(unsigned int ChannelDDS_UART, bool firstInit);
bool extern choiceChannelDDS2(unsigned int ChannelDDS_UART, bool firstInit);
bool extern choiceChannelDDS3(unsigned int ChannelDDS_UART, bool firstInit);
bool extern choiceChannelDDS(unsigned int ChannelDDS_UART, bool firstInit);

// DDS Buffered functions
extern bool acquireBufferedSequence(char* msg_cmd, char* msg_val, char* msg_DDS_n, char* msg_t, char* msg_rpl, bool replyWanted, bool isFirstCmd);
extern bool launchBufferedSequence(void);

void trackRegisterValuesFromDDSOnReboot(void);
void trackRegisterValuesFromSingleAD9959DDSOnReboot(unsigned int DDSnb);

extern bool flashMemoryAD9959DDS(unsigned int ChannelDDS_UART);
extern bool simpleflashMemoryAD9959DDS1(unsigned int ChannelDDS_UART);
extern bool simpleflashMemoryAD9959DDS2(unsigned int ChannelDDS_UART);
extern bool simpleflashMemoryAD9959DDS3(unsigned int ChannelDDS_UART);

extern bool loadValuesFromFlashMemoryAD9959DDS(unsigned int ChannelDDS_UART);
bool simpleLoadValuesFromFlashMemoryAD9959DDS1(unsigned int ChannelDDS_UART);
bool simpleLoadValuesFromFlashMemoryAD9959DDS2(unsigned int ChannelDDS_UART);
bool simpleLoadValuesFromFlashMemoryAD9959DDS3(unsigned int ChannelDDS_UART);

extern bool simpleflashMemoryAD9959DDS_ethernet_frequency(unsigned int ChannelDDS_UART);

// SSI serial and SPI communications
void init_SPI0_mode_single_bit(void);
void init_SPI0_mode_quad_bit(void);

void init_SPI1_mode_quad_bit(void);
void init_SPI1_mode_single_bit(void);

void init_SPI2_mode_single_bit(void);
void init_SPI2_mode_quad_bit(void);


void ssi0PutData(int instruction,long data,int num_byte);
void ssi0PutData_Burst(int instruction,long data);
//void ssi0PutData_QSPI_Fast(uint8_t instruction, uint32_t data);

void ssi1PutData(int instruction,long data,int num_byte);
void ssi1PutData_Burst(int instruction,long data);

void ssi2PutData(int instruction,long data,int num_byte);
void ssi2PutData_Burst(int instruction,long data);

long long ssi0GetData(int instruction, int num_byte);
void SSI0IntHandler(void);

// Timer during the triggered sequence
void initTimer0(void);


void PortA_Init(void);
void PortB_Init(void);
void PortD_Init(void);
void PortN_Init(void);
void PortH_Init(void);
void PortM_Init(void);
void PortL_Init(void);
void PortP_Init(void);
void PeripheralEnableInit(void);


#endif /* ARM_TM4C1294_H_ */
