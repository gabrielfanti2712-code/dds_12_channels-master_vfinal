/*
 * AD9959.c
 *
 *  Created on: 14/02/2024
 *      Author: Fabrice
 */
#include <stdlib.h>
#include "utils/ustdlib.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "AD9959.h"
#include "ARM TM4C1294.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"    
#include "driverlib/timer.h"
#include "inc/hw_memmap.h" // Base addresses

#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"

char sMsg[20];
unsigned long freq_32bits;
unsigned int frequency_data_in_Hz;
int Displaydata(uint32_t ui32value);

// --- VARIABLES D'�TAT DU BALAYAGE GLOBAL (utilis�es par les ISR) ---
volatile unsigned long freq_start_hz;
volatile unsigned long start_freq_hz;
volatile unsigned long freq_end_hz;
volatile unsigned long freq_step_hz;
//unsigned long freq_step_hz;
volatile unsigned int g_channel;
volatile unsigned int g_ftw_end;
volatile unsigned int g_current_ftw;
volatile unsigned long g_ftw_start_config;
volatile bool g_is_rising;
volatile bool g_sweep_run = false;
double time_step_s;
unsigned long time_step_us;

// D�claration des variables globales
unsigned int NbDDS = 0;
unsigned long FreqToFTW(unsigned long frequency_hz);
unsigned int ChannelDDS = 0;

//*************************Ethernet config channels****************************************************//
bool SetchannelDDS(unsigned int channelValue, unsigned int ChannelDDS_UART)
{

    if (channelValue <4)
    {
     choiceChannelDDS(channelValue, true);

                   if(CURRENT_DDS == 1)
                   {
                       if (CURRENT_ChannelDDS == 0)
                            {
                                ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS1,CSR_NUM_BYTE);  //select CH0
                                //SysCtlDelay(100);
                                PK7 = 0x80; // AD9959 I/O update DDS1
                                SysCtlDelay(10);
                                PK7 = 0x00;
                                //UARTprintf("DDS1 Channel 0 \n");
                                return true;
                            }
                            else if (CURRENT_ChannelDDS == 1)
                            {
                                ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS1,CSR_NUM_BYTE);  //select CH1
                                //SysCtlDelay(100);
                                PK7 = 0x80; // AD9959 I/O update DDS1
                                SysCtlDelay(10);
                                PK7 = 0x00;
                                //UARTprintf("DDS1 Channel 1 \n");
                                return true;
                            }
                            else if (CURRENT_ChannelDDS == 2)
                            {
                                ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS1,CSR_NUM_BYTE);  //select CH2
                                //SysCtlDelay(100);
                                PK7 = 0x80; // AD9959 I/O update DDS1
                                SysCtlDelay(10);
                                PK7 = 0x00;
                                //UARTprintf("DDS1 Channel 2 \n");
                                return true;
                            }
                            else if (CURRENT_ChannelDDS == 3)
                            {
                                ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS1,CSR_NUM_BYTE);  //select CH3
                                //SysCtlDelay(100);
                                PK7 = 0x80; // AD9959 I/O update DDS1
                                SysCtlDelay(10);
                                PK7 = 0x00;
                                //UARTprintf("DDS1 Channel 3 \n");
                                return true;
                            }
                            else
                            {
                                return false;
                            }

                   }
                   else if(CURRENT_DDS == 2)
                   {
                       if (CURRENT_ChannelDDS == 0)
                          {
                               ssi1PutData(CSR_ADRESS,CSR0_4bit_DDS2,CSR_NUM_BYTE);  //select CH0
                               //SysCtlDelay(100);
                               PN3 = 0x08; // AD9959 I/O update DDS2
                               SysCtlDelay(10);
                               PN3 = 0x00;
                               //UARTprintf("DDS2 Channel 0 \n");
                               return true;
                          }
                        else if (CURRENT_ChannelDDS == 1)
                          {
                               ssi1PutData(CSR_ADRESS,CSR1_4bit_DDS2,CSR_NUM_BYTE);  //select CH1
                               //SysCtlDelay(100);
                               PN3 = 0x08; // AD9959 I/O update DDS2
                               SysCtlDelay(10);
                               PN3 = 0x00;
                               //UARTprintf("DDS2 Channel 1 \n");
                               return true;
                          }
                       else if (CURRENT_ChannelDDS == 2)
                          {
                               ssi1PutData(CSR_ADRESS,CSR2_4bit_DDS2,CSR_NUM_BYTE);  //select CH2
                               //SysCtlDelay(100);
                               PN3 = 0x08; // AD9959 I/O update DDS2
                               SysCtlDelay(10);
                               PN3 = 0x00;
                               //UARTprintf("DDS2 Channel 2 \n");
                               return true;
                          }
                       else if (CURRENT_ChannelDDS == 3)
                       {
                               ssi1PutData(CSR_ADRESS,CSR3_4bit_DDS2,CSR_NUM_BYTE);  //select CH3
                               //SysCtlDelay(100);
                               PN3 = 0x08; // AD9959 I/O update DDS2
                               SysCtlDelay(10);
                               PN3 = 0x00;
                               //UARTprintf("DDS2 Channel 3 \n");
                               return true;
                        }
                       else
                        {
                               return false;
                        }

                    }
                    else if(CURRENT_DDS == 3)
                        {
                           if (CURRENT_ChannelDDS == 0)
                           {
                               ssi2PutData(CSR_ADRESS,CSR0_4bit_DDS3,CSR_NUM_BYTE);  //select CH0
                               //SysCtlDelay(100);
                               PN0 = 0x01; // AD9959 I/O update DDS3
                               SysCtlDelay(10);
                               PN0 = 0x00;
                               //UARTprintf("DDS3 Channel 0 \n");
                               return true;
                            }
                           else if (CURRENT_ChannelDDS == 1)
                            {
                               ssi2PutData(CSR_ADRESS,CSR1_4bit_DDS3,CSR_NUM_BYTE);  //select CH1
                               //SysCtlDelay(100);
                               PN0 = 0x01; // AD9959 I/O update DDS3
                               SysCtlDelay(10);
                               PN0 = 0x00;
                               //UARTprintf("DDS3 Channel 1 \n");
                               return true;
                            }
                           else if (CURRENT_ChannelDDS == 2)
                            {
                               ssi2PutData(CSR_ADRESS,CSR2_4bit_DDS3,CSR_NUM_BYTE);  //select CH2
                               //SysCtlDelay(100);
                               PN0 = 0x01; // AD9959 I/O update DDS3
                               SysCtlDelay(10);
                               PN0 = 0x00;
                               //UARTprintf("DDS3 Channel 2 \n");
                               return true;
                            }
                           else if (CURRENT_ChannelDDS == 3)
                            {
                               ssi2PutData(CSR_ADRESS,CSR3_4bit_DDS3,CSR_NUM_BYTE);  //select CH3
                               //SysCtlDelay(100);
                               PN0 = 0x01; // AD9959 I/O update DDS3
                               SysCtlDelay(10);
                               PN0 = 0x00;
                               //UARTprintf("DDS3 Channel 3 \n");
                               return true;
                            }
                       else
                        {
                               return false;
                        }

               }

               else
               {
               return false;
               }

   }else
              {
               return false;
              }
}
//*************************Ethernet config****************************************************//
bool SetFrequencyTuningWord_ethernet(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{
if (frequencyValue <= 0xFFFFFFFF)
{
  if( CURRENT_DDS == 1)
    {
        if (CURRENT_ChannelDDS == 0)
        {
            frequency_data_in_Hz =
                (
                    (double)frequencyValue
                    * (double)SYSCLKDDS
                    / 4294967296.0
                );            start_freq_hz =  frequency_data_in_Hz;
            start_freq_hz = (start_freq_hz * 4294967296.0 / SYSCLKDDS);
            ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS1,CSR_NUM_BYTE);  //select CH0
            ssi0PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 1)
        {
            frequency_data_in_Hz =
                (
                    (double)frequencyValue
                    * (double)SYSCLKDDS
                    / 4294967296.0
                );            start_freq_hz =  frequency_data_in_Hz;
            start_freq_hz = (start_freq_hz * 4294967296.0 / SYSCLKDDS);
            ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS1,CSR_NUM_BYTE);  //select CH1
            ssi0PutData(FTW_ADRESS,start_freq_hz,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 2)
        {
            frequency_data_in_Hz =
                (
                    (double)frequencyValue
                    * (double)SYSCLKDDS
                    / 4294967296.0
                );            start_freq_hz =  frequency_data_in_Hz;
            start_freq_hz = (start_freq_hz * 4294967296.0 / SYSCLKDDS);
            ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS1,CSR_NUM_BYTE);  //select CH2
            ssi0PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 3)
        {
            frequency_data_in_Hz =
                (
                    (double)frequencyValue
                    * (double)SYSCLKDDS
                    / 4294967296.0
                );            start_freq_hz =  frequency_data_in_Hz;
            start_freq_hz = (start_freq_hz * 4294967296.0 / SYSCLKDDS);
            ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS1,CSR_NUM_BYTE);  //select CH3
            ssi0PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }

        else
        {
            return false;
        }

    }

    else if( CURRENT_DDS == 2)
      {
          if (CURRENT_ChannelDDS == 0)
          {
              frequency_data_in_Hz =
                  (
                      (double)frequencyValue
                      * (double)SYSCLKDDS
                      / 4294967296.0
                  );              start_freq_hz =  frequency_data_in_Hz;
              start_freq_hz = (start_freq_hz * 4294967296.0 / SYSCLKDDS);
              ssi1PutData(CSR_ADRESS,CSR0_4bit_DDS2,CSR_NUM_BYTE);  //select CH0
              ssi1PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
              frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
              PN3 = 0x08; // AD9959 I/O update DDS2
              SysCtlDelay(10);
              PN3 = 0x00;
              return true;
          }
          else if (CURRENT_ChannelDDS == 1)
          {
              frequency_data_in_Hz =
                  (
                      (double)frequencyValue
                      * (double)SYSCLKDDS
                      / 4294967296.0
                  );              start_freq_hz =  frequency_data_in_Hz;
              start_freq_hz = (start_freq_hz * 4294967296.0 / SYSCLKDDS);
              ssi1PutData(CSR_ADRESS,CSR1_4bit_DDS2,CSR_NUM_BYTE);  //select CH1
              ssi1PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
              frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
              PN3 = 0x08; // AD9959 I/O update DDS2
              SysCtlDelay(10);
              PN3 = 0x00;
              return true;
          }
          else if (CURRENT_ChannelDDS == 2)
          {
              frequency_data_in_Hz =
                  (
                      (double)frequencyValue
                      * (double)SYSCLKDDS
                      / 4294967296.0
                  );              start_freq_hz =  frequency_data_in_Hz;
              start_freq_hz = (start_freq_hz * 4294967296.0 / SYSCLKDDS);
              ssi1PutData(CSR_ADRESS,CSR2_4bit_DDS2,CSR_NUM_BYTE);  //select CH2
              ssi1PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
              frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
              PN3 = 0x08; // AD9959 I/O update DDS2
              SysCtlDelay(10);
              PN3 = 0x00;
              return true;
          }
          else if (CURRENT_ChannelDDS == 3)
          {

              frequency_data_in_Hz =
                  (
                      (double)frequencyValue
                      * (double)SYSCLKDDS
                      / 4294967296.0
                  );              start_freq_hz =  frequency_data_in_Hz;
              start_freq_hz = (start_freq_hz * 4294967296.0 / SYSCLKDDS);
              ssi1PutData(CSR_ADRESS,CSR3_4bit_DDS2,CSR_NUM_BYTE);  //select CH3
              ssi1PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
              frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
              PN3 = 0x08; // AD9959 I/O update DDS2
              SysCtlDelay(10);
              PN3 = 0x00;
              return true;
          }

          else
          {
              return false;
          }

      }
    else if( CURRENT_DDS == 3)
          {
              if (CURRENT_ChannelDDS == 0)
              {
                  /*ssi2PutData(CSR_ADRESS,CSR0_4bit_DDS3,CSR_NUM_BYTE);  //select CH0
                  ssi2PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
                  frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
                  frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                  PN0 = 0x01; // AD9959 I/O update DDS3
                  SysCtlDelay(20);
                  PN0 = 0x00;
                  */
                  return true;
              }
              else if (CURRENT_ChannelDDS == 1)
              {
                  frequency_data_in_Hz =
                      (
                          (double)frequencyValue
                          * (double)SYSCLKDDS
                          / 4294967296.0
                      );                  start_freq_hz =  frequency_data_in_Hz;
                  start_freq_hz = (start_freq_hz * 4294967296.0 / SYSCLKDDS);
                  ssi2PutData(CSR_ADRESS,CSR1_4bit_DDS3,CSR_NUM_BYTE);  //select CH1
                  ssi2PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
                  frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
                  PN0 = 0x01; // AD9959 I/O update DDS3
                  SysCtlDelay(10);
                  PN0 = 0x00;
                  return true;
              }
              else if (CURRENT_ChannelDDS == 2)
              {
                  frequency_data_in_Hz =
                      (
                          (double)frequencyValue
                          * (double)SYSCLKDDS
                          / 4294967296.0
                      );                  start_freq_hz =  frequency_data_in_Hz;
                  start_freq_hz = (start_freq_hz * 4294967296.0 / SYSCLKDDS);
                  ssi2PutData(CSR_ADRESS,CSR2_4bit_DDS3,CSR_NUM_BYTE);  //select CH2
                  ssi2PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
                  frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
                  PN0 = 0x01; // AD9959 I/O update DDS3
                  SysCtlDelay(10);
                  PN0 = 0x00;
                  return true;
              }
              else if (CURRENT_ChannelDDS == 3)
              {
                  frequency_data_in_Hz =
                      (
                          (double)frequencyValue
                          * (double)SYSCLKDDS
                          / 4294967296.0
                      );                  start_freq_hz =  frequency_data_in_Hz;
                  start_freq_hz = (start_freq_hz * 4294967296.0 / SYSCLKDDS);
                  ssi2PutData(CSR_ADRESS,CSR3_4bit_DDS3,CSR_NUM_BYTE);  //select CH3
                  ssi2PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
                  frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
                  PN0 = 0x01; // AD9959 I/O update DDS3
                  SysCtlDelay(10);
                  PN0 = 0x00;
                  return true;
              }

              else
              {
                  return false;
              }
     }
    else {
          return false;
         }

   }else {
          return false;
         }
}
//*************************Ethernet config****************************************************//
bool SetFrequencyTuningWord1_ethernet(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{

if (frequencyValue <= 0xFFFFFFFF)
{
  if( CURRENT_DDS == 1)
    {
        if (CURRENT_ChannelDDS == 0)
        {
            // --- Conversion en Mots de R�glage (Tuning Words) ---
            frequency_data_in_Hz =
                (
                    (double)frequencyValue
                    * (double)SYSCLKDDS
                    / 4294967296.0
                );            freq_end_hz =  frequency_data_in_Hz;
            freq_start_hz =
                (
                    (double)start_freq_hz
                    * (double)SYSCLKDDS
                    / 4294967296.0
                );            if (g_sweep_run) {
                    // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                    TimerDisable(TIMER1_BASE, TIMER_A);
                g_sweep_run = false;
                }

Apply_Sweep_Configuration_ethernet(
                    NbDDS,
                    ChannelDDS,
                    freq_start_hz,
                    freq_end_hz,
                    freq_step_hz,
                    time_step_s
                   );
            return true;
        }
        else if (CURRENT_ChannelDDS == 1)
        {
            // --- Conversion en Mots de R�glage (Tuning Words) ---
                       frequency_data_in_Hz =
                           (
                               (double)frequencyValue
                               * (double)SYSCLKDDS
                               / 4294967296.0
                           );                       freq_end_hz =  frequency_data_in_Hz;
                       freq_start_hz =
                           (
                               (double)start_freq_hz
                               * (double)SYSCLKDDS
                               / 4294967296.0
                           );                       if (g_sweep_run) {
                               // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                               TimerDisable(TIMER1_BASE, TIMER_A);
                           g_sweep_run = false;
                           }

           Apply_Sweep_Configuration_ethernet(
                               NbDDS,
                               ChannelDDS,
                               freq_start_hz,
                               freq_end_hz,
                               freq_step_hz,
                               time_step_s
                              );
            return true;
        }
        else if (CURRENT_ChannelDDS == 2)
        {
            // --- Conversion en Mots de R�glage (Tuning Words) ---
                       frequency_data_in_Hz =
                           (
                               (double)frequencyValue
                               * (double)SYSCLKDDS
                               / 4294967296.0
                           );                       freq_end_hz =  frequency_data_in_Hz;
                       freq_start_hz =
                           (
                               (double)start_freq_hz
                               * (double)SYSCLKDDS
                               / 4294967296.0
                           );                       if (g_sweep_run) {
                               // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                               TimerDisable(TIMER1_BASE, TIMER_A);
                           g_sweep_run = false;
                           }

           Apply_Sweep_Configuration_ethernet(
                               NbDDS,
                               ChannelDDS,
                               freq_start_hz,
                               freq_end_hz,
                               freq_step_hz,
                               time_step_s
                              );
            return true;
        }
        else if (CURRENT_ChannelDDS == 3)
        {
            // --- Conversion en Mots de R�glage (Tuning Words) ---
                       frequency_data_in_Hz =
                           (
                               (double)frequencyValue
                               * (double)SYSCLKDDS
                               / 4294967296.0
                           );                       freq_end_hz =  frequency_data_in_Hz;
                       freq_start_hz =
                           (
                               (double)start_freq_hz
                               * (double)SYSCLKDDS
                               / 4294967296.0
                           );                       if (g_sweep_run) {
                               // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                               TimerDisable(TIMER1_BASE, TIMER_A);
                           g_sweep_run = false;
                           }

           Apply_Sweep_Configuration_ethernet(
                               NbDDS,
                               ChannelDDS,
                               freq_start_hz,
                               freq_end_hz,
                               freq_step_hz,
                               time_step_s
                              );

            return true;
        }

        else
        {
            return false;
        }

    }

    else if( CURRENT_DDS == 2)
      {
          if (CURRENT_ChannelDDS == 0)
          {
              // --- Conversion en Mots de R�glage (Tuning Words) ---
                         frequency_data_in_Hz =
                             (
                                 (double)frequencyValue
                                 * (double)SYSCLKDDS
                                 / 4294967296.0
                             );                         freq_end_hz =  frequency_data_in_Hz;
                         freq_start_hz =
                             (
                                 (double)start_freq_hz
                                 * (double)SYSCLKDDS
                                 / 4294967296.0
                             );                         if (g_sweep_run) {
                                 // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                 TimerDisable(TIMER1_BASE, TIMER_A);
                             g_sweep_run = false;
                             }

             Apply_Sweep_Configuration_ethernet(
                                 NbDDS,
                                 ChannelDDS,
                                 freq_start_hz,
                                 freq_end_hz,
                                 freq_step_hz,
                                 time_step_s
                                );
              return true;
          }
          else if (CURRENT_ChannelDDS == 1)
          {
              // --- Conversion en Mots de R�glage (Tuning Words) ---
                         frequency_data_in_Hz =
                             (
                                 (double)frequencyValue
                                 * (double)SYSCLKDDS
                                 / 4294967296.0
                             );                         freq_end_hz =  frequency_data_in_Hz;
                         freq_start_hz =
                             (
                                 (double)start_freq_hz
                                 * (double)SYSCLKDDS
                                 / 4294967296.0
                             );                         if (g_sweep_run) {
                                 // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                 TimerDisable(TIMER1_BASE, TIMER_A);
                             g_sweep_run = false;
                             }

             Apply_Sweep_Configuration_ethernet(
                                 NbDDS,
                                 ChannelDDS,
                                 freq_start_hz,
                                 freq_end_hz,
                                 freq_step_hz,
                                 time_step_s
                                );
              return true;
          }
          else if (CURRENT_ChannelDDS == 2)
          {
              // --- Conversion en Mots de R�glage (Tuning Words) ---
                         frequency_data_in_Hz =
                             (
                                 (double)frequencyValue
                                 * (double)SYSCLKDDS
                                 / 4294967296.0
                             );                         freq_end_hz =  frequency_data_in_Hz;
                         freq_start_hz =
                             (
                                 (double)start_freq_hz
                                 * (double)SYSCLKDDS
                                 / 4294967296.0
                             );                         if (g_sweep_run) {
                                 // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                 TimerDisable(TIMER1_BASE, TIMER_A);
                             g_sweep_run = false;
                             }

             Apply_Sweep_Configuration_ethernet(
                                 NbDDS,
                                 ChannelDDS,
                                 freq_start_hz,
                                 freq_end_hz,
                                 freq_step_hz,
                                 time_step_s
                                );
              return true;
          }
          else if (CURRENT_ChannelDDS == 3)
          {
              // --- Conversion en Mots de R�glage (Tuning Words) ---
                         frequency_data_in_Hz =
                             (
                                 (double)frequencyValue
                                 * (double)SYSCLKDDS
                                 / 4294967296.0
                             );                         freq_end_hz =  frequency_data_in_Hz;
                         freq_start_hz =
                             (
                                 (double)start_freq_hz
                                 * (double)SYSCLKDDS
                                 / 4294967296.0
                             );                         if (g_sweep_run) {
                                 // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                 TimerDisable(TIMER1_BASE, TIMER_A);
                             g_sweep_run = false;
                             }

             Apply_Sweep_Configuration_ethernet(
                                 NbDDS,
                                 ChannelDDS,
                                 freq_start_hz,
                                 freq_end_hz,
                                 freq_step_hz,
                                 time_step_s
                                );

              return true;
          }

          else
          {
              return false;
          }

      }
    else if( CURRENT_DDS == 3)
          {
              if (CURRENT_ChannelDDS == 0)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                           frequency_data_in_Hz =
                                               (
                                                   (double)frequencyValue
                                                   * (double)SYSCLKDDS
                                                   / 4294967296.0
                                               );                                           freq_end_hz =  frequency_data_in_Hz;
                                           freq_start_hz =
                                               (
                                                   (double)start_freq_hz
                                                   * (double)SYSCLKDDS
                                                   / 4294967296.0
                                               );                                           if (g_sweep_run) {
                                                   // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                   TimerDisable(TIMER1_BASE, TIMER_A);
                                               g_sweep_run = false;
                                               }

                               Apply_Sweep_Configuration_ethernet(
                                                   NbDDS,
                                                   ChannelDDS,
                                                   freq_start_hz,
                                                   freq_end_hz,
                                                   freq_step_hz,
                                                   time_step_s
                                                  );
                  return true;
              }
              else if (CURRENT_ChannelDDS == 1)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                           frequency_data_in_Hz =
                                               (
                                                   (double)frequencyValue
                                                   * (double)SYSCLKDDS
                                                   / 4294967296.0
                                               );                                           freq_end_hz =  frequency_data_in_Hz;
                                           freq_start_hz =
                                               (
                                                   (double)start_freq_hz
                                                   * (double)SYSCLKDDS
                                                   / 4294967296.0
                                               );                                           if (g_sweep_run) {
                                                   // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                   TimerDisable(TIMER1_BASE, TIMER_A);
                                               g_sweep_run = false;
                                               }

                               Apply_Sweep_Configuration_ethernet(
                                                   NbDDS,
                                                   ChannelDDS,
                                                   freq_start_hz,
                                                   freq_end_hz,
                                                   freq_step_hz,
                                                   time_step_s
                                                  );
                  return true;
              }
              else if (CURRENT_ChannelDDS == 2)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                           frequency_data_in_Hz =
                                               (
                                                   (double)frequencyValue
                                                   * (double)SYSCLKDDS
                                                   / 4294967296.0
                                               );                                           freq_end_hz =  frequency_data_in_Hz;
                                           freq_start_hz =
                                               (
                                                   (double)start_freq_hz
                                                   * (double)SYSCLKDDS
                                                   / 4294967296.0
                                               );                                           if (g_sweep_run) {
                                                   // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                   TimerDisable(TIMER1_BASE, TIMER_A);
                                               g_sweep_run = false;
                                               }

                               Apply_Sweep_Configuration_ethernet(
                                                   NbDDS,
                                                   ChannelDDS,
                                                   freq_start_hz,
                                                   freq_end_hz,
                                                   freq_step_hz,
                                                   time_step_s
                                                  );
                  return true;
              }
              else if (CURRENT_ChannelDDS == 3)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                           frequency_data_in_Hz =
                                               (
                                                   (double)frequencyValue
                                                   * (double)SYSCLKDDS
                                                   / 4294967296.0
                                               );                                           freq_end_hz =  frequency_data_in_Hz;
                                           freq_start_hz =
                                               (
                                                   (double)start_freq_hz
                                                   * (double)SYSCLKDDS
                                                   / 4294967296.0
                                               );                                           if (g_sweep_run) {
                                                   // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                   TimerDisable(TIMER1_BASE, TIMER_A);
                                               g_sweep_run = false;
                                               }

                               Apply_Sweep_Configuration_ethernet(
                                                   NbDDS,
                                                   ChannelDDS,
                                                   freq_start_hz,
                                                   freq_end_hz,
                                                   freq_step_hz,
                                                   time_step_s
                                                  );
                  return true;
              }

              else
              {
                  return false;
              }
     }
    else {
          return false;
         }

   }else {
          return false;
         }
}
//*************************Ethernet config****************************************************//
bool Set_frequency_step(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{


    if (frequencyValue <= 0xFFFFFFFF)
    {
      if( CURRENT_DDS == 1)
        {
            if (CURRENT_ChannelDDS == 0)
            {
                // --- Conversion en Mots de R�glage (Tuning Words) ---
                            //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                            //freq_end_hz =  frequency_data_in_Hz;
                            frequency_data_in_Hz =
                                (
                                    (double)frequencyValue
                                    * (double)SYSCLKDDS
                                    / 4294967296.0
                                );                            freq_step_hz =  frequency_data_in_Hz;
                            freq_start_hz =
                                (
                                    (double)start_freq_hz
                                    * (double)SYSCLKDDS
                                    / 4294967296.0
                                );                            if (g_sweep_run) {
                                    // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                    TimerDisable(TIMER1_BASE, TIMER_A);
                                g_sweep_run = false;
                                }

Apply_Sweep_Configuration_ethernet(
                   CURRENT_DDS,
                   ChannelDDS,
                   freq_start_hz,
                   freq_end_hz,
                   freq_step_hz,
                   time_step_s
                   );
                return true;
            }
            else if (CURRENT_ChannelDDS == 1)
            {
                // --- Conversion en Mots de R�glage (Tuning Words) ---
                                            //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                            //freq_end_hz =  frequency_data_in_Hz;
                                            frequency_data_in_Hz =
                                                (
                                                    (double)frequencyValue
                                                    * (double)SYSCLKDDS
                                                    / 4294967296.0
                                                );                                            freq_step_hz =  frequency_data_in_Hz;
                                            freq_start_hz =
                                                (
                                                    (double)start_freq_hz
                                                    * (double)SYSCLKDDS
                                                    / 4294967296.0
                                                );                                            if (g_sweep_run) {
                                                    // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                    TimerDisable(TIMER1_BASE, TIMER_A);
                                                g_sweep_run = false;
                                                }

                Apply_Sweep_Configuration_ethernet(
                                   CURRENT_DDS,
                                   ChannelDDS,
                                   freq_start_hz,
                                   freq_end_hz,
                                   freq_step_hz,
                                   time_step_s
                                   );
                return true;
            }
            else if (CURRENT_ChannelDDS == 2)
            {
                // --- Conversion en Mots de R�glage (Tuning Words) ---
                                            //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                            //freq_end_hz =  frequency_data_in_Hz;
                                            frequency_data_in_Hz =
                                                (
                                                    (double)frequencyValue
                                                    * (double)SYSCLKDDS
                                                    / 4294967296.0
                                                );                                            freq_step_hz =  frequency_data_in_Hz;
                                            freq_start_hz =
                                                (
                                                    (double)start_freq_hz
                                                    * (double)SYSCLKDDS
                                                    / 4294967296.0
                                                );                                            if (g_sweep_run) {
                                                    // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                    TimerDisable(TIMER1_BASE, TIMER_A);
                                                g_sweep_run = false;
                                                }

                Apply_Sweep_Configuration_ethernet(
                                   CURRENT_DDS,
                                   ChannelDDS,
                                   freq_start_hz,
                                   freq_end_hz,
                                   freq_step_hz,
                                   time_step_s
                                   );
                return true;
            }
            else if (CURRENT_ChannelDDS == 3)
            {
                // --- Conversion en Mots de R�glage (Tuning Words) ---
                                            //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                            //freq_end_hz =  frequency_data_in_Hz;
                                            frequency_data_in_Hz =
                                                (
                                                    (double)frequencyValue
                                                    * (double)SYSCLKDDS
                                                    / 4294967296.0
                                                );                                            freq_step_hz =  frequency_data_in_Hz;
                                            freq_start_hz =
                                                (
                                                    (double)start_freq_hz
                                                    * (double)SYSCLKDDS
                                                    / 4294967296.0
                                                );                                            if (g_sweep_run) {
                                                    // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                    TimerDisable(TIMER1_BASE, TIMER_A);
                                                g_sweep_run = false;
                                                }

                Apply_Sweep_Configuration_ethernet(
                                   CURRENT_DDS,
                                   ChannelDDS,
                                   freq_start_hz,
                                   freq_end_hz,
                                   freq_step_hz,
                                   time_step_s
                                   );

                return true;
            }

            else
            {
                return false;
            }

        }

        else if( CURRENT_DDS == 2)
          {
              if (CURRENT_ChannelDDS == 0)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                              //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                              //freq_end_hz =  frequency_data_in_Hz;
                                              frequency_data_in_Hz =
                                                  (
                                                      (double)frequencyValue
                                                      * (double)SYSCLKDDS
                                                      / 4294967296.0
                                                  );                                              freq_step_hz =  frequency_data_in_Hz;
                                              freq_start_hz =
                                                  (
                                                      (double)start_freq_hz
                                                      * (double)SYSCLKDDS
                                                      / 4294967296.0
                                                  );                                              if (g_sweep_run) {
                                                      // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                      TimerDisable(TIMER1_BASE, TIMER_A);
                                                  g_sweep_run = false;
                                                  }

                  Apply_Sweep_Configuration_ethernet(
                                     CURRENT_DDS,
                                     ChannelDDS,
                                     freq_start_hz,
                                     freq_end_hz,
                                     freq_step_hz,
                                     time_step_s
                                     );
                  return true;
              }
              else if (CURRENT_ChannelDDS == 1)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                              //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                              //freq_end_hz =  frequency_data_in_Hz;
                                              frequency_data_in_Hz =
                                                  (
                                                      (double)frequencyValue
                                                      * (double)SYSCLKDDS
                                                      / 4294967296.0
                                                  );                                              freq_step_hz =  frequency_data_in_Hz;
                                              freq_start_hz =
                                                  (
                                                      (double)start_freq_hz
                                                      * (double)SYSCLKDDS
                                                      / 4294967296.0
                                                  );                                              if (g_sweep_run) {
                                                      // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                      TimerDisable(TIMER1_BASE, TIMER_A);
                                                  g_sweep_run = false;
                                                  }

                  Apply_Sweep_Configuration_ethernet(
                                     CURRENT_DDS,
                                     ChannelDDS,
                                     freq_start_hz,
                                     freq_end_hz,
                                     freq_step_hz,
                                     time_step_s
                                     );
                  return true;
              }
              else if (CURRENT_ChannelDDS == 2)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                              //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                              //freq_end_hz =  frequency_data_in_Hz;
                                              frequency_data_in_Hz =
                                                  (
                                                      (double)frequencyValue
                                                      * (double)SYSCLKDDS
                                                      / 4294967296.0
                                                  );                                              freq_step_hz =  frequency_data_in_Hz;
                                              freq_start_hz =
                                                  (
                                                      (double)start_freq_hz
                                                      * (double)SYSCLKDDS
                                                      / 4294967296.0
                                                  );                                              if (g_sweep_run) {
                                                      // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                      TimerDisable(TIMER1_BASE, TIMER_A);
                                                  g_sweep_run = false;
                                                  }

                  Apply_Sweep_Configuration_ethernet(
                                     CURRENT_DDS,
                                     ChannelDDS,
                                     freq_start_hz,
                                     freq_end_hz,
                                     freq_step_hz,
                                     time_step_s
                                     );

                  return true;
              }
              else if (CURRENT_ChannelDDS == 3)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                              //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                              //freq_end_hz =  frequency_data_in_Hz;
                                              frequency_data_in_Hz =
                                                  (
                                                      (double)frequencyValue
                                                      * (double)SYSCLKDDS
                                                      / 4294967296.0
                                                  );                                              freq_step_hz =  frequency_data_in_Hz;
                                              freq_start_hz =
                                                  (
                                                      (double)start_freq_hz
                                                      * (double)SYSCLKDDS
                                                      / 4294967296.0
                                                  );                                              if (g_sweep_run) {
                                                      // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                      TimerDisable(TIMER1_BASE, TIMER_A);
                                                  g_sweep_run = false;
                                                  }

                  Apply_Sweep_Configuration_ethernet(
                                     CURRENT_DDS,
                                     ChannelDDS,
                                     freq_start_hz,
                                     freq_end_hz,
                                     freq_step_hz,
                                     time_step_s
                                     );
                  return true;
              }

              else
              {
                  return false;
              }

          }
        else if( CURRENT_DDS == 3)
              {
                  if (CURRENT_ChannelDDS == 0)
                  {
                      // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                  //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                  //freq_end_hz =  frequency_data_in_Hz;
                                                  frequency_data_in_Hz =
                                                      (
                                                          (double)frequencyValue
                                                          * (double)SYSCLKDDS
                                                          / 4294967296.0
                                                      );                                                  freq_step_hz =  frequency_data_in_Hz;
                                                  freq_start_hz =
                                                      (
                                                          (double)start_freq_hz
                                                          * (double)SYSCLKDDS
                                                          / 4294967296.0
                                                      );                                                  if (g_sweep_run) {
                                                          // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                          TimerDisable(TIMER1_BASE, TIMER_A);
                                                      g_sweep_run = false;
                                                      }

                      Apply_Sweep_Configuration_ethernet(
                                         CURRENT_DDS,
                                         ChannelDDS,
                                         freq_start_hz,
                                         freq_end_hz,
                                         freq_step_hz,
                                         time_step_s
                                         );
                       return true;
                  }
                  else if (CURRENT_ChannelDDS == 1)
                  {
                      // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                  //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                  //freq_end_hz =  frequency_data_in_Hz;
                                                  frequency_data_in_Hz =
                                                      (
                                                          (double)frequencyValue
                                                          * (double)SYSCLKDDS
                                                          / 4294967296.0
                                                      );                                                  freq_step_hz =  frequency_data_in_Hz;
                                                  freq_start_hz =
                                                      (
                                                          (double)start_freq_hz
                                                          * (double)SYSCLKDDS
                                                          / 4294967296.0
                                                      );                                                  if (g_sweep_run) {
                                                          // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                          TimerDisable(TIMER1_BASE, TIMER_A);
                                                      g_sweep_run = false;
                                                      }

                      Apply_Sweep_Configuration_ethernet(
                                         CURRENT_DDS,
                                         ChannelDDS,
                                         freq_start_hz,
                                         freq_end_hz,
                                         freq_step_hz,
                                         time_step_s
                                         );
                       return true;
                  }
                  else if (CURRENT_ChannelDDS == 2)
                  {
                      // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                  //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                  //freq_end_hz =  frequency_data_in_Hz;
                                                  frequency_data_in_Hz =
                                                      (
                                                          (double)frequencyValue
                                                          * (double)SYSCLKDDS
                                                          / 4294967296.0
                                                      );                                                  freq_step_hz =  frequency_data_in_Hz;
                                                  freq_start_hz =
                                                      (
                                                          (double)start_freq_hz
                                                          * (double)SYSCLKDDS
                                                          / 4294967296.0
                                                      );                                                  if (g_sweep_run) {
                                                          // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                          TimerDisable(TIMER1_BASE, TIMER_A);
                                                      g_sweep_run = false;
                                                      }

                      Apply_Sweep_Configuration_ethernet(
                                         CURRENT_DDS,
                                         ChannelDDS,
                                         freq_start_hz,
                                         freq_end_hz,
                                         freq_step_hz,
                                         time_step_s
                                         );
                       return true;
                  }
                  else if (CURRENT_ChannelDDS == 3)
                  {
                      // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                  //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                  //freq_end_hz =  frequency_data_in_Hz;
                                                  frequency_data_in_Hz =
                                                      (
                                                          (double)frequencyValue
                                                          * (double)SYSCLKDDS
                                                          / 4294967296.0
                                                      );                                                  freq_step_hz =  frequency_data_in_Hz;
                                                  freq_start_hz =
                                                      (
                                                          (double)start_freq_hz
                                                          * (double)SYSCLKDDS
                                                          / 4294967296.0
                                                      );                                                  if (g_sweep_run) {
                                                          // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                          TimerDisable(TIMER1_BASE, TIMER_A);
                                                      g_sweep_run = false;
                                                      }

                      Apply_Sweep_Configuration_ethernet(
                                         CURRENT_DDS,
                                         ChannelDDS,
                                         freq_start_hz,
                                         freq_end_hz,
                                         freq_step_hz,
                                         time_step_s
                                         );
                       return true;
                  }

                  else
                  {
                      return false;
                  }
         }
        else {
              return false;
             }

       }else {
              return false;
             }
}
//*************************Ethernet config****************************************************//
bool Set_time_step(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{
    if (frequencyValue <= 0xFFFFFFFF)
    {
      if( CURRENT_DDS == 1)
        {
            if (CURRENT_ChannelDDS == 0)
            {
                // --- Conversion en Mots de R�glage (Tuning Words) ---
                                            //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                            //freq_end_hz =  frequency_data_in_Hz;
                                            time_step_us = frequencyValue;
                                            time_step_s = (double)time_step_us / 1000000.0;
                                            //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                            //freq_step_hz =  frequency_data_in_Hz;
                                            freq_start_hz =
                                                (
                                                    (double)start_freq_hz
                                                    * (double)SYSCLKDDS
                                                    / 4294967296.0
                                                );                                            if (g_sweep_run) {
                                                    // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                    TimerDisable(TIMER1_BASE, TIMER_A);
                                                g_sweep_run = false;
                                                }

                                Apply_Sweep_Configuration_ethernet(
                                                    CURRENT_DDS,
                                                    ChannelDDS,
                                                    freq_start_hz,
                                                    freq_end_hz,
                                                    freq_step_hz,
                                                    time_step_s
                                                   );
                return true;
            }
            else if (CURRENT_ChannelDDS == 1)
            {
                // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                            //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                            //freq_end_hz =  frequency_data_in_Hz;
                                                            time_step_us = frequencyValue;
                                                            time_step_s = (double)time_step_us / 1000000.0;
                                                            //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                            //freq_step_hz =  frequency_data_in_Hz;
                                                            freq_start_hz =
                                                                (
                                                                    (double)start_freq_hz
                                                                    * (double)SYSCLKDDS
                                                                    / 4294967296.0
                                                                );                                                            if (g_sweep_run) {
                                                                    // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                                    TimerDisable(TIMER1_BASE, TIMER_A);
                                                                g_sweep_run = false;
                                                                }

                                                Apply_Sweep_Configuration_ethernet(
                                                                    CURRENT_DDS,
                                                                    ChannelDDS,
                                                                    freq_start_hz,
                                                                    freq_end_hz,
                                                                    freq_step_hz,
                                                                    time_step_s
                                                                   );
                return true;
            }
            else if (CURRENT_ChannelDDS == 2)
            {
                // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                            //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                            //freq_end_hz =  frequency_data_in_Hz;
                                                            time_step_us = frequencyValue;
                                                            time_step_s = (double)time_step_us / 1000000.0;
                                                            //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                            //freq_step_hz =  frequency_data_in_Hz;
                                                            freq_start_hz =
                                                                (
                                                                    (double)start_freq_hz
                                                                    * (double)SYSCLKDDS
                                                                    / 4294967296.0
                                                                );                                                            if (g_sweep_run) {
                                                                    // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                                    TimerDisable(TIMER1_BASE, TIMER_A);
                                                                g_sweep_run = false;
                                                                }

                                                Apply_Sweep_Configuration_ethernet(
                                                                    CURRENT_DDS,
                                                                    ChannelDDS,
                                                                    freq_start_hz,
                                                                    freq_end_hz,
                                                                    freq_step_hz,
                                                                    time_step_s
                                                                   );
                return true;
            }
            else if (CURRENT_ChannelDDS == 3)
            {
                // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                            //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                            //freq_end_hz =  frequency_data_in_Hz;
                                                            time_step_us = frequencyValue;
                                                            time_step_s = (double)time_step_us / 1000000.0;
                                                            //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                            //freq_step_hz =  frequency_data_in_Hz;
                                                            freq_start_hz =
                                                                (
                                                                    (double)start_freq_hz
                                                                    * (double)SYSCLKDDS
                                                                    / 4294967296.0
                                                                );                                                            if (g_sweep_run) {
                                                                    // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                                    TimerDisable(TIMER1_BASE, TIMER_A);
                                                                g_sweep_run = false;
                                                                }

                                                Apply_Sweep_Configuration_ethernet(
                                                                    CURRENT_DDS,
                                                                    ChannelDDS,
                                                                    freq_start_hz,
                                                                    freq_end_hz,
                                                                    freq_step_hz,
                                                                    time_step_s
                                                                   );

                return true;
            }

            else
            {
                return false;
            }

        }

        else if( CURRENT_DDS == 2)
          {
              if (CURRENT_ChannelDDS == 0)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                              //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                              //freq_end_hz =  frequency_data_in_Hz;
                                                              time_step_us = frequencyValue;
                                                              time_step_s = (double)time_step_us / 1000000.0;
                                                              //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                              //freq_step_hz =  frequency_data_in_Hz;
                                                              freq_start_hz =
                                                                  (
                                                                      (double)start_freq_hz
                                                                      * (double)SYSCLKDDS
                                                                      / 4294967296.0
                                                                  );                                                              if (g_sweep_run) {
                                                                      // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                                      TimerDisable(TIMER1_BASE, TIMER_A);
                                                                  g_sweep_run = false;
                                                                  }

                                                  Apply_Sweep_Configuration_ethernet(
                                                                      CURRENT_DDS,
                                                                      ChannelDDS,
                                                                      freq_start_hz,
                                                                      freq_end_hz,
                                                                      freq_step_hz,
                                                                      time_step_s
                                                                     );
                  return true;
              }
              else if (CURRENT_ChannelDDS == 1)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                              //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                              //freq_end_hz =  frequency_data_in_Hz;
                                                              time_step_us = frequencyValue;
                                                              time_step_s = (double)time_step_us / 1000000.0;
                                                              //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                              //freq_step_hz =  frequency_data_in_Hz;
                                                              freq_start_hz =
                                                                  (
                                                                      (double)start_freq_hz
                                                                      * (double)SYSCLKDDS
                                                                      / 4294967296.0
                                                                  );                                                              if (g_sweep_run) {
                                                                      // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                                      TimerDisable(TIMER1_BASE, TIMER_A);
                                                                  g_sweep_run = false;
                                                                  }

                                                  Apply_Sweep_Configuration_ethernet(
                                                                      CURRENT_DDS,
                                                                      ChannelDDS,
                                                                      freq_start_hz,
                                                                      freq_end_hz,
                                                                      freq_step_hz,
                                                                      time_step_s
                                                                     );
                  return true;
              }
              else if (CURRENT_ChannelDDS == 2)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                              //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                              //freq_end_hz =  frequency_data_in_Hz;
                                                              time_step_us = frequencyValue;
                                                              time_step_s = (double)time_step_us / 1000000.0;
                                                              //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                              //freq_step_hz =  frequency_data_in_Hz;
                                                              freq_start_hz =
                                                                  (
                                                                      (double)start_freq_hz
                                                                      * (double)SYSCLKDDS
                                                                      / 4294967296.0
                                                                  );                                                              if (g_sweep_run) {
                                                                      // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                                      TimerDisable(TIMER1_BASE, TIMER_A);
                                                                  g_sweep_run = false;
                                                                  }

                                                  Apply_Sweep_Configuration_ethernet(
                                                                      CURRENT_DDS,
                                                                      ChannelDDS,
                                                                      freq_start_hz,
                                                                      freq_end_hz,
                                                                      freq_step_hz,
                                                                      time_step_s
                                                                     );

                  return true;
              }
              else if (CURRENT_ChannelDDS == 3)
              {
                  // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                              //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                              //freq_end_hz =  frequency_data_in_Hz;
                                                              time_step_us = frequencyValue;
                                                              time_step_s = (double)time_step_us / 1000000.0;
                                                              //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                              //freq_step_hz =  frequency_data_in_Hz;
                                                              freq_start_hz =
                                                                  (
                                                                      (double)start_freq_hz
                                                                      * (double)SYSCLKDDS
                                                                      / 4294967296.0
                                                                  );                                                              if (g_sweep_run) {
                                                                      // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                                      TimerDisable(TIMER1_BASE, TIMER_A);
                                                                  g_sweep_run = false;
                                                                  }

                                                  Apply_Sweep_Configuration_ethernet(
                                                                      CURRENT_DDS,
                                                                      ChannelDDS,
                                                                      freq_start_hz,
                                                                      freq_end_hz,
                                                                      freq_step_hz,
                                                                      time_step_s
                                                                     );
                  return true;
              }

              else
              {
                  return false;
              }

          }
        else if( CURRENT_DDS == 3)
              {
                  if (CURRENT_ChannelDDS == 0)
                  {
                      // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                                  //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                                  //freq_end_hz =  frequency_data_in_Hz;
                                                                  time_step_us = frequencyValue;
                                                                  time_step_s = (double)time_step_us / 1000000.0;
                                                                  //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                                  //freq_step_hz =  frequency_data_in_Hz;
                                                                  freq_start_hz =
                                                                      (
                                                                          (double)start_freq_hz
                                                                          * (double)SYSCLKDDS
                                                                          / 4294967296.0
                                                                      );                                                                  if (g_sweep_run) {
                                                                          // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                                          TimerDisable(TIMER1_BASE, TIMER_A);
                                                                      g_sweep_run = false;
                                                                      }

                                                      Apply_Sweep_Configuration_ethernet(
                                                                          CURRENT_DDS,
                                                                          ChannelDDS,
                                                                          freq_start_hz,
                                                                          freq_end_hz,
                                                                          freq_step_hz,
                                                                          time_step_s
                                                                         );
                       return true;
                  }
                  else if (CURRENT_ChannelDDS == 1)
                  {
                      // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                                  //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                                  //freq_end_hz =  frequency_data_in_Hz;
                                                                  time_step_us = frequencyValue;
                                                                  time_step_s = (double)time_step_us / 1000000.0;
                                                                  //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                                  //freq_step_hz =  frequency_data_in_Hz;
                                                                  freq_start_hz =
                                                                      (
                                                                          (double)start_freq_hz
                                                                          * (double)SYSCLKDDS
                                                                          / 4294967296.0
                                                                      );                                                                  if (g_sweep_run) {
                                                                          // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                                          TimerDisable(TIMER1_BASE, TIMER_A);
                                                                      g_sweep_run = false;
                                                                      }

                                                      Apply_Sweep_Configuration_ethernet(
                                                                          CURRENT_DDS,
                                                                          ChannelDDS,
                                                                          freq_start_hz,
                                                                          freq_end_hz,
                                                                          freq_step_hz,
                                                                          time_step_s
                                                                         );
                       return true;
                  }
                  else if (CURRENT_ChannelDDS == 2)
                  {
                      // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                                  //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                                  //freq_end_hz =  frequency_data_in_Hz;
                                                                  time_step_us = frequencyValue;
                                                                  time_step_s = (double)time_step_us / 1000000.0;
                                                                  //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                                  //freq_step_hz =  frequency_data_in_Hz;
                                                                  freq_start_hz =
                                                                      (
                                                                          (double)start_freq_hz
                                                                          * (double)SYSCLKDDS
                                                                          / 4294967296.0
                                                                      );                                                                  if (g_sweep_run) {
                                                                          // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                                          TimerDisable(TIMER1_BASE, TIMER_A);
                                                                      g_sweep_run = false;
                                                                      }

                                                      Apply_Sweep_Configuration_ethernet(
                                                                          CURRENT_DDS,
                                                                          ChannelDDS,
                                                                          freq_start_hz,
                                                                          freq_end_hz,
                                                                          freq_step_hz,
                                                                          time_step_s
                                                                         );
                       return true;
                  }
                  else if (CURRENT_ChannelDDS == 3)
                  {
                      // --- Conversion en Mots de R�glage (Tuning Words) ---
                                                                  //frequency_data_in_Hz =  (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                                  //freq_end_hz =  frequency_data_in_Hz;
                                                                  time_step_us = frequencyValue;
                                                                  time_step_s = (double)time_step_us / 1000000.0;
                                                                  //step_time = (frequencyValue * SYSCLKDDS / 4294967296.0);
                                                                  //freq_step_hz =  frequency_data_in_Hz;
                                                                  freq_start_hz =
                                                                      (
                                                                          (double)start_freq_hz
                                                                          * (double)SYSCLKDDS
                                                                          / 4294967296.0
                                                                      );                                                                  if (g_sweep_run) {
                                                                          // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                                                                          TimerDisable(TIMER1_BASE, TIMER_A);
                                                                      g_sweep_run = false;
                                                                      }

                                                      Apply_Sweep_Configuration_ethernet(
                                                                          CURRENT_DDS,
                                                                          ChannelDDS,
                                                                          freq_start_hz,
                                                                          freq_end_hz,
                                                                          freq_step_hz,
                                                                          time_step_s
                                                                         );
                       return true;
                  }

                  else
                  {
                      return false;
                  }
         }
        else {
              return false;
             }

       }else {
              return false;
             }
}
//*************************UART config****************************************************//
bool SetFrequencyTuningWord0_DDS1(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{


    if (frequencyValue <= 0xFFFFFFFF)
    {

        if (CURRENT_ChannelDDS == 0)
        {
            //ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS1,CSR_NUM_BYTE);  //select CH0
            ssi0PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 1)
        {
            //ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS1,CSR_NUM_BYTE);  //select CH1
            ssi0PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 2)
        {
            //ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS1,CSR_NUM_BYTE);  //select CH2
            ssi0PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 3)
        {
            //ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS1,CSR_NUM_BYTE);  //select CH3
            ssi0PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }

        else
        {
            return false;
        }
    }
    else {
           return false;
         }
}
bool SetFrequencyTuningWord0_DDS1_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{


    if (frequencyValue <= 0xFFFFFFFF)
    {

        if (CURRENT_ChannelDDS == 0)
        {
            //ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS1,CSR_NUM_BYTE);  //select CH0
            //ssi0PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            ssi0PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7 � 1
            // Petit d�lai (Hold time) - 1 NOPs � 120MHz = ~8ns
            __asm(" NOP");
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7 � 0
            return true;
        }
        else if (CURRENT_ChannelDDS == 1)
        {
            //ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS1,CSR_NUM_BYTE);  //select CH1
            //ssi0PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            ssi0PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7 � 1
            // Petit d�lai (Hold time) - 1 NOPs � 120MHz = ~8ns
            __asm(" NOP");
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7 � 0
            return true;
        }
        else if (CURRENT_ChannelDDS == 2)
        {
            //ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS1,CSR_NUM_BYTE);  //select CH2
            //ssi0PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            ssi0PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7 � 1
            // Petit d�lai (Hold time) - 1 NOPs � 120MHz = ~8ns
            __asm(" NOP");
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7 � 0
            return true;
        }
        else if (CURRENT_ChannelDDS == 3)
        {
            //ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS1,CSR_NUM_BYTE);  //select CH3
            //ssi0PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            ssi0PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7 � 1
            // Petit d�lai (Hold time) - 1 NOPs � 120MHz = ~8ns
            __asm(" NOP");
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7 � 0
            return true;
        }

        else
        {
            return false;
        }
    }
    else {
           return false;
         }
}
//*************************UART config****************************************************//
bool SetFrequencyTuningWord0_DDS2(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{
    if (frequencyValue <= 0xFFFFFFFF)
    {

        if (CURRENT_ChannelDDS == 0)
        {
            ssi1PutData(CSR_ADRESS,CSR0_4bit_DDS2,CSR_NUM_BYTE);  //select CH0
            ssi1PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            //SysCtlDelay(100);
            PN3 = 0x08; // AD9959 I/O update DDS2
            SysCtlDelay(10);
            PN3 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 1)
        {
            ssi1PutData(CSR_ADRESS,CSR1_4bit_DDS2,CSR_NUM_BYTE);  //select CH1
            ssi1PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            //SysCtlDelay(100);
            PN3 = 0x08; // AD9959 I/O update DDS2
            SysCtlDelay(10);
            PN3 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 2)
        {
            ssi1PutData(CSR_ADRESS,CSR2_4bit_DDS2,CSR_NUM_BYTE);  //select CH2
            ssi1PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            //SysCtlDelay(100);
            PN3 = 0x08; // AD9959 I/O update DDS2
            SysCtlDelay(10);
            PN3 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 3)
        {
            ssi1PutData(CSR_ADRESS,CSR3_4bit_DDS2,CSR_NUM_BYTE);  //select CH3
            ssi1PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            //SysCtlDelay(100);
            PN3 = 0x08; // AD9959 I/O update DDS2
            SysCtlDelay(10);
            PN3 = 0x00;
            return true;
        }

        else
        {
            return false;
        }
    }
    else {
           return false;
         }
}
bool SetFrequencyTuningWord0_DDS2_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{


    if (frequencyValue <= 0xFFFFFFFF)
    {

        if (CURRENT_ChannelDDS == 0)
        {
            ssi1PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;  // Mettre PN3 � 1
            // Petit d�lai (Hold time) - 1 NOPs � 120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
            __asm(" NOP");
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00; // Mettre PN3 � 0
            return true;
        }
        else if (CURRENT_ChannelDDS == 1)
        {
            ssi1PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;  // Mettre PN3 � 1
            // Petit d�lai (Hold time) - 1 NOPs � 120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
            __asm(" NOP");
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00; // Mettre PN3 � 0
            return true;
        }
        else if (CURRENT_ChannelDDS == 2)
        {
            ssi1PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;  // Mettre PN3 � 1
            // Petit d�lai (Hold time) - 1 NOPs � 120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
            __asm(" NOP");
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00; // Mettre PN3 � 0
            return true;
        }
        else if (CURRENT_ChannelDDS == 3)
        {
            ssi1PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;  // Mettre PN3 � 1
            // Petit d�lai (Hold time) - 1 NOPs � 120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
            __asm(" NOP");
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00; // Mettre PN3 � 0
            return true;
        }

        else
        {
            return false;
        }
    }
    else {
           return false;
         }
}
//*************************UART config****************************************************//
bool SetFrequencyTuningWord0_DDS3(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{
    if (frequencyValue <= 0xFFFFFFFF)
    {

        if (CURRENT_ChannelDDS == 0)
        {
            ssi2PutData(CSR_ADRESS,CSR0_4bit_DDS3,CSR_NUM_BYTE);  //select CH0
            ssi2PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[0] = frequencyValue;
            PN0 = 0x01; // AD9959 I/O update DDS3
            SysCtlDelay(10);
            PN0 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 1)
        {
            ssi2PutData(CSR_ADRESS,CSR1_4bit_DDS3,CSR_NUM_BYTE);  //select CH1
            ssi2PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            //SysCtlDelay(100);
            PN0 = 0x01; // AD9959 I/O update DDS3
            SysCtlDelay(10);
            PN0 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 2)
        {
            ssi2PutData(CSR_ADRESS,CSR2_4bit_DDS3,CSR_NUM_BYTE);  //select CH2
            ssi2PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            //SysCtlDelay(100);
            PN0 = 0x01; // AD9959 I/O update DDS3
            SysCtlDelay(10);
            PN0 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 3)
        {
            ssi2PutData(CSR_ADRESS,CSR3_4bit_DDS3,CSR_NUM_BYTE);  //select CH3
            ssi2PutData(FTW_ADRESS,frequencyValue,FTW_NUM_BYTE);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            //SysCtlDelay(100);
            PN0 = 0x01; // AD9959 I/O update DDS3
            SysCtlDelay(10);
            PN0 = 0x00;
            return true;
        }

        else
        {
            return false;
        }
    }
    else {
           return false;
         }
}
bool SetFrequencyTuningWord0_DDS3_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{


    if (frequencyValue <= 0xFFFFFFFF)
    {

        if (CURRENT_ChannelDDS == 0)
        {
            ssi2PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[0] = frequencyValue;
            while(SSI2_SR_R & SSI_SR_BSY);
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01;
            __asm(" NOP");
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 1)
        {
            ssi2PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01; // Mettre PN0 � 1
            // Petit d�lai (Hold time) - 1 NOPs � 120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
            __asm(" NOP");
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00; // Mettre PN0 � 0
            return true;
        }
        else if (CURRENT_ChannelDDS == 2)
        {
            ssi2PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01; // Mettre PN0 � 1
            // Petit d�lai (Hold time) - 1 NOPs � 120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
            __asm(" NOP");
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00; // Mettre PN0 � 0
            return true;
        }
        else if (CURRENT_ChannelDDS == 3)
        {
            ssi2PutData_Burst(FTW_ADRESS,frequencyValue);
            frequencyValueAD9959DDS[ChannelDDS_UART-1] = frequencyValue;
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01; // Mettre PN0 � 1
            // Petit d�lai (Hold time) - 1 NOPs � 120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
            __asm(" NOP");
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00; // Mettre PN0 � 0
            return true;
        }

        else
        {
            return false;
        }
    }
    else {
           return false;
         }
}
bool SetamplValue_ethernet(unsigned short amplValue, unsigned int ChannelDDS_UART)
{
  if (amplValue <= 0x1399)
  {

    if( CURRENT_DDS == 1)
      {
       if (CURRENT_ChannelDDS == 0)
        {
            ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS1,CSR_NUM_BYTE);  //select CH0
            ssi0PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7 � 1
            // Petit d�lai (Hold time) - 3 NOPs � 120MHz = ~25ns
            __asm(" NOP");
            __asm(" NOP");
            __asm(" NOP");
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7 � 0
            //SysCtlDelay(100);
            //PK7 = 0x80; // AD9959 I/O update DDS1
            //SysCtlDelay(10);
            //PK7 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 1)
        {
            ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS1,CSR_NUM_BYTE);  //select CH1
            ssi0PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7 � 1
            // Petit d�lai (Hold time) - 3 NOPs � 120MHz = ~25ns
            __asm(" NOP");
            __asm(" NOP");
            __asm(" NOP");
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7 � 0
            //SysCtlDelay(100);
            //PK7 = 0x80; // AD9959 I/O update DDS1
            //SysCtlDelay(10);
            //PK7 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 2)
       {
            ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS1,CSR_NUM_BYTE);  //select CH2
            ssi0PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7 � 1
            // Petit d�lai (Hold time) - 3 NOPs � 120MHz = ~25ns
            __asm(" NOP");
            __asm(" NOP");
            __asm(" NOP");
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7 � 0
            //SysCtlDelay(100);
            //PK7 = 0x80; // AD9959 I/O update DDS1
            //SysCtlDelay(10);
            //PK7 = 0x00;
            return true;
       }
        else if (CURRENT_ChannelDDS == 3)
       {
           ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS1,CSR_NUM_BYTE);  //select CH3
           ssi0PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
           amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
           HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7 � 1
           // Petit d�lai (Hold time) - 3 NOPs � 120MHz = ~25ns
           __asm(" NOP");
           __asm(" NOP");
           __asm(" NOP");
           HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7 � 0
           //SysCtlDelay(100);
           //PK7 = 0x80; // AD9959 I/O update DDS1
           //SysCtlDelay(10);
           //PK7 = 0x00;
           return true;
       }
        else
        {
            return false;
        }
    }
    else if ( CURRENT_DDS == 2)
    {
      if (CURRENT_ChannelDDS == 0)
         {
           ssi1PutData(CSR_ADRESS,CSR0_4bit_DDS2,CSR_NUM_BYTE);  //select CH0
           ssi1PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
           amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
           //SysCtlDelay(100);
           PN3 = 0x08; // AD9959 I/O update DDS2
           SysCtlDelay(10);
           PN3 = 0x00;
           return true;
         }
      else if (CURRENT_ChannelDDS == 1)
         {
          ssi1PutData(CSR_ADRESS,CSR1_4bit_DDS2,CSR_NUM_BYTE);  //select CH1
          ssi1PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
          amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
          //SysCtlDelay(100);
          PN3 = 0x08; // AD9959 I/O update DDS2
          SysCtlDelay(10);
          PN3 = 0x00;
          return true;
         }
      else if (CURRENT_ChannelDDS == 2)
         {
          ssi1PutData(CSR_ADRESS,CSR2_4bit_DDS2,CSR_NUM_BYTE);  //select CH2
          ssi1PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
          amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
          //SysCtlDelay(100);
          PN3 = 0x08; // AD9959 I/O update DDS2
          SysCtlDelay(10);
          PN3 = 0x00;
          return true;
         }
     else if (CURRENT_ChannelDDS == 3)
         {
          ssi1PutData(CSR_ADRESS,CSR3_4bit_DDS2,CSR_NUM_BYTE);  //select CH3
          ssi1PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
          amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
          //SysCtlDelay(100);
          PN3 = 0x08; // AD9959 I/O update DDS2
          SysCtlDelay(10);
          PN3 = 0x00;
          return true;
         }
     else
        {
         return false;
         }
    }
    else if ( CURRENT_DDS == 3)
    {
      if (CURRENT_ChannelDDS == 0)
         {
           /*ssi2PutData(CSR_ADRESS,CSR0_4bit_DDS3,CSR_NUM_BYTE);  //select CH0
           ssi2PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
           amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
           SysCtlDelay(100);
           PN0 = 0x01; // AD9959 I/O update DDS3
           SysCtlDelay(10);
           PN0 = 0x00;
           */
           return true;
         }
      else if (CURRENT_ChannelDDS == 1)
         {
          ssi2PutData(CSR_ADRESS,CSR1_4bit_DDS3,CSR_NUM_BYTE);  //select CH1
          ssi2PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
          amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
          //SysCtlDelay(100);
          PN0 = 0x01; // AD9959 I/O update DDS3
          SysCtlDelay(10);
          PN0 = 0x00;
          return true;
         }
      else if (CURRENT_ChannelDDS == 2)
         {
          ssi2PutData(CSR_ADRESS,CSR2_4bit_DDS3,CSR_NUM_BYTE);  //select CH2
          ssi2PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
          amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
          //SysCtlDelay(100);
          PN0 = 0x01; // AD9959 I/O update DDS3
          SysCtlDelay(10);
          PN0 = 0x00;
          return true;
         }
     else if (CURRENT_ChannelDDS == 3)
         {
          ssi2PutData(CSR_ADRESS,CSR3_4bit_DDS3,CSR_NUM_BYTE);  //select CH3
          ssi2PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
          amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
          //SysCtlDelay(100);
          PN0 = 0x01; // AD9959 I/O update DDS3
          SysCtlDelay(10);
          PN0 = 0x00;
          return true;
         }
     else
        {
         return false;
         }

    }
    else
    {
        return false;
    }
} else
{
    return false;
}
}
bool SetpowValue_ethernet(unsigned short powValue, unsigned int ChannelDDS_UART)
{
  if (powValue <= 0x4000)
  {

    if( CURRENT_DDS == 1)
      {
       if (CURRENT_ChannelDDS == 0)
        {
            ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS1,CSR_NUM_BYTE);  //select CH0
            ssi0PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            //SysCtlDelay(100);
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 1)
        {
            ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS1,CSR_NUM_BYTE);  //select CH1
            ssi0PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            //SysCtlDelay(100);
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (CURRENT_ChannelDDS == 2)
       {
            ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS1,CSR_NUM_BYTE);  //select CH2
            ssi0PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            //SysCtlDelay(100);
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
       }
        else if (CURRENT_ChannelDDS == 3)
       {
           ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS1,CSR_NUM_BYTE);  //select CH3
           ssi0PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
           powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
           //SysCtlDelay(100);
           PK7 = 0x80; // AD9959 I/O update DDS1
           SysCtlDelay(10);
           PK7 = 0x00;
           return true;
       }
        else
        {
            return false;
        }
    }
    else if ( CURRENT_DDS == 2)
    {
      if (CURRENT_ChannelDDS == 0)
         {
           ssi1PutData(CSR_ADRESS,CSR0_4bit_DDS2,CSR_NUM_BYTE);  //select CH0
           ssi1PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
           powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
           //SysCtlDelay(100);
           PN3 = 0x08; // AD9959 I/O update DDS2
           SysCtlDelay(10);
           PN3 = 0x00;
           return true;
         }
      else if (CURRENT_ChannelDDS == 1)
         {
          ssi1PutData(CSR_ADRESS,CSR1_4bit_DDS2,CSR_NUM_BYTE);  //select CH1
          ssi1PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
          powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
          //SysCtlDelay(100);
          PN3 = 0x08; // AD9959 I/O update DDS2
          SysCtlDelay(10);
          PN3 = 0x00;
          return true;
         }
      else if (CURRENT_ChannelDDS == 2)
         {
          ssi1PutData(CSR_ADRESS,CSR2_4bit_DDS2,CSR_NUM_BYTE);  //select CH2
          ssi1PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
          powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
          //SysCtlDelay(100);
          PN3 = 0x08; // AD9959 I/O update DDS2
          SysCtlDelay(10);
          PN3 = 0x00;
          return true;
         }
     else if (CURRENT_ChannelDDS == 3)
         {
          ssi1PutData(CSR_ADRESS,CSR3_4bit_DDS2,CSR_NUM_BYTE);  //select CH3
          ssi1PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
          powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
          //SysCtlDelay(100);
          PN3 = 0x08; // AD9959 I/O update DDS2
          SysCtlDelay(10);
          PN3 = 0x00;
          return true;
         }
     else
        {
         return false;
         }
    }
    else if ( CURRENT_DDS == 3)
    {
      if (CURRENT_ChannelDDS == 0)
         {
           /*ssi2PutData(CSR_ADRESS,CSR0_4bit_DDS3,CSR_NUM_BYTE);  //select CH0
           ssi2PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
           powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
           SysCtlDelay(100);
           PN0 = 0x01; // AD9959 I/O update DDS3
           SysCtlDelay(10);
           PN0 = 0x00;
           */
           return true;
         }
      else if (CURRENT_ChannelDDS == 1)
         {
          ssi2PutData(CSR_ADRESS,CSR1_4bit_DDS3,CSR_NUM_BYTE);  //select CH1
          ssi2PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
          powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
          //SysCtlDelay(100);
          PN0 = 0x01; // AD9959 I/O update DDS3
          SysCtlDelay(10);
          PN0 = 0x00;
          return true;
         }
      else if (CURRENT_ChannelDDS == 2)
         {
          ssi2PutData(CSR_ADRESS,CSR2_4bit_DDS3,CSR_NUM_BYTE);  //select CH2
          ssi2PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
          powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
          //SysCtlDelay(100);
          PN0 = 0x01; // AD9959 I/O update DDS3
          SysCtlDelay(10);
          PN0 = 0x00;
          return true;
         }
     else if (CURRENT_ChannelDDS == 3)
         {
          ssi2PutData(CSR_ADRESS,CSR3_4bit_DDS3,CSR_NUM_BYTE);  //select CH3
          ssi2PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
          powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
          //SysCtlDelay(100);
          PN0 = 0x01; // AD9959 I/O update DDS3
          SysCtlDelay(10);
          PN0 = 0x00;
          return true;
         }
     else
        {
         return false;
         }

    }
    else
    {
        return false;
    }
} else
{
    return false;
}
}
//********************** uart amplitude word DDS1*********************************//
bool SetamplValueAD9959DDS1(unsigned short amplValue, unsigned int ChannelDDS_UART)
{
    if (amplValue <= 0x1399)
    {

        if (ChannelDDS_UART == 0)
        {
            ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS1,CSR_NUM_BYTE);  //select CH0
            ssi0PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (ChannelDDS_UART == 1)
        {
            ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS1,CSR_NUM_BYTE);  //select CH1
            ssi0PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (ChannelDDS_UART == 2)
       {
            ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS1,CSR_NUM_BYTE);  //select CH2
            ssi0PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
       }
        else if (ChannelDDS_UART == 3)
       {
           ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS1,CSR_NUM_BYTE);  //select CH3
           ssi0PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
           amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
           PK7 = 0x80; // AD9959 I/O update DDS1
           SysCtlDelay(10);
           PK7 = 0x00;
           return true;
       }
        else
        {
            return false;
        }
    } else
    {
        return false;
    }
}
//********************** uart amplitude word DDS2*********************************//
bool SetamplValueAD9959DDS2(unsigned short amplValue, unsigned int ChannelDDS_UART)
{
    if (amplValue <= 0x1399)
    {

        if (ChannelDDS_UART == 0)
        {
            ssi1PutData(CSR_ADRESS,CSR0_4bit_DDS2,CSR_NUM_BYTE);  //select CH0
            ssi1PutData(ACR_ADRESS,  amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            //SysCtlDelay(100);
            PN3 = 0x08; // AD9959 I/O update DDS2
            SysCtlDelay(10);
            PN3 = 0x00;
            return true;
        }
        else if (ChannelDDS_UART == 1)
        {
            ssi1PutData(CSR_ADRESS,CSR1_4bit_DDS2,CSR_NUM_BYTE);  //select CH1
            ssi1PutData(ACR_ADRESS,  amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            //SysCtlDelay(100);
            PN3 = 0x08; // AD9959 I/O update DDS2
            SysCtlDelay(10);
            PN3 = 0x00;
            return true;
        }
        else if (ChannelDDS_UART == 2)
       {
            ssi1PutData(CSR_ADRESS,CSR2_4bit_DDS2,CSR_NUM_BYTE);  //select CH2
            ssi1PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            //SysCtlDelay(100);
            PN3 = 0x08; // AD9959 I/O update DDS2
            SysCtlDelay(10);
            PN3 = 0x00;
            return true;
       }
        else if (ChannelDDS_UART == 3)
       {
           ssi1PutData(CSR_ADRESS,CSR3_4bit_DDS2,CSR_NUM_BYTE);  //select CH3
           ssi1PutData(ACR_ADRESS, amplValue + 0x1000, ACR_NUM_BYTE);
           amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
           //SysCtlDelay(100);
           PN3 = 0x08; // AD9959 I/O update DDS2
           SysCtlDelay(10);
           PN3 = 0x00;
           return true;
       }
        else
        {
            return false;
        }
    } else
    {
        return false;
    }
}
//********************** uart amplitude word DDS3*********************************//
bool SetamplValueAD9959DDS3(unsigned short amplValue, unsigned int ChannelDDS_UART)
{
    if (amplValue <= 0x1399)
    {

        if (ChannelDDS_UART == 0)
        {
            /*ssi2PutData(CSR_ADRESS,CSR0_4bit_DDS3,CSR_NUM_BYTE);  //select CH0
            ssi2PutData(ACR_ADRESS,  amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            SysCtlDelay(100);
            PN0 = 0x01; // AD9959 I/O update DDS3
            SysCtlDelay(10);
            PN0 = 0x00;
            */
            return true;
        }
        else if (ChannelDDS_UART == 1)
        {
            ssi2PutData(CSR_ADRESS,CSR1_4bit_DDS3,CSR_NUM_BYTE);  //select CH1
            ssi2PutData(ACR_ADRESS,  amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            //SysCtlDelay(100);
            PN0 = 0x01; // AD9959 I/O update DDS3
            SysCtlDelay(10);
            PN0 = 0x00;
            return true;
        }
        else if (ChannelDDS_UART == 2)
       {
            ssi2PutData(CSR_ADRESS,CSR2_4bit_DDS3,CSR_NUM_BYTE);  //select CH2
            ssi2PutData(ACR_ADRESS,  amplValue + 0x1000, ACR_NUM_BYTE);
            amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
            //SysCtlDelay(100);
            PN0 = 0x01; // AD9959 I/O update DDS3
            SysCtlDelay(10);
            PN0 = 0x00;
            return true;
       }
        else if (ChannelDDS_UART == 3)
       {
           ssi2PutData(CSR_ADRESS,CSR3_4bit_DDS3,CSR_NUM_BYTE);  //select CH3
           ssi2PutData(ACR_ADRESS,  amplValue + 0x1000, ACR_NUM_BYTE);
           amplValueAD9959DDS[ChannelDDS_UART-1] = amplValue;
           //SysCtlDelay(100);
           PN0 = 0x01; // AD9959 I/O update DDS3
           SysCtlDelay(10);
           PN0 = 0x00;
           return true;
       }
        else
        {
            return false;
        }
    }
   else {
         return false;
        }
}
//********************** uart phase offset word DDS1*********************************//
bool SetPOW_ValueAD9959DDS1(unsigned short powValue, unsigned int ChannelDDS_UART)
{
    if (powValue <= 0x4000)
    {

        if (ChannelDDS_UART == 0)
        {
            ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS1,CSR_NUM_BYTE);  //select CH0
            ssi0PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (ChannelDDS_UART == 1)
        {
            ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS1,CSR_NUM_BYTE);  //select CH1
            ssi0PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
        }
        else if (ChannelDDS_UART == 2)
       {
            ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS1,CSR_NUM_BYTE);  //select CH2
            ssi0PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            PK7 = 0x80; // AD9959 I/O update DDS1
            SysCtlDelay(10);
            PK7 = 0x00;
            return true;
       }
        else if (ChannelDDS_UART == 3)
       {
           ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS1,CSR_NUM_BYTE);  //select CH3
           ssi0PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
           powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
           PK7 = 0x80; // AD9959 I/O update DDS1
           SysCtlDelay(10);
           PK7 = 0x00;
           return true;
       }
        else
        {
            return false;
        }
    } else
    {
        return false;
    }
}
//********************** uart phase offset word DDS2*********************************//
bool SetPOW_ValueAD9959DDS2(unsigned short powValue, unsigned int ChannelDDS_UART)
{
    if (powValue <= 0x4000)
    {

        if (ChannelDDS_UART == 0)
        {
            ssi1PutData(CSR_ADRESS,CSR0_4bit_DDS2,CSR_NUM_BYTE);  //select CH0
            ssi1PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            //SysCtlDelay(100);
            PN3 = 0x08; // AD9959 I/O update DDS2
            SysCtlDelay(10);
            PN3 = 0x00;
            return true;
        }
        else if (ChannelDDS_UART == 1)
        {
            ssi1PutData(CSR_ADRESS,CSR1_4bit_DDS2,CSR_NUM_BYTE);  //select CH1
            ssi1PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            //SysCtlDelay(100);
            PN3 = 0x08; // AD9959 I/O update DDS2
            SysCtlDelay(10);
            PN3 = 0x00;
            return true;
        }
        else if (ChannelDDS_UART == 2)
       {
            ssi1PutData(CSR_ADRESS,CSR2_4bit_DDS2,CSR_NUM_BYTE);  //select CH2
            ssi1PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            //SysCtlDelay(100);
            PN3 = 0x08; // AD9959 I/O update DDS2
            SysCtlDelay(10);
            PN3 = 0x00;
            return true;
       }
        else if (ChannelDDS_UART == 3)
       {
           ssi1PutData(CSR_ADRESS,CSR3_4bit_DDS2,CSR_NUM_BYTE);  //select CH3
           ssi1PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
           powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
           //SysCtlDelay(100);
           PN3 = 0x08; // AD9959 I/O update DDS2
           SysCtlDelay(10);
           PN3 = 0x00;
           return true;
       }
        else
        {
            return false;
        }
    } else
    {
        return false;
    }
}
//********************** uart phase offset word DDS3*********************************//
bool SetPOW_ValueAD9959DDS3(unsigned short powValue, unsigned int ChannelDDS_UART)
{
    if (powValue <= 0x4000)
    {

        if (ChannelDDS_UART == 0)
        {
            /*ssi2PutData(CSR_ADRESS,CSR0_4bit_DDS3,CSR_NUM_BYTE);  //select CH0
            ssi2PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            SysCtlDelay(100);
            PN0 = 0x01; // AD9959 I/O update DDS3
            SysCtlDelay(10);
            PN0 = 0x00;
            */
            return true;
        }
        else if (ChannelDDS_UART == 1)
        {
            ssi2PutData(CSR_ADRESS,CSR1_4bit_DDS3,CSR_NUM_BYTE);  //select CH1
            ssi2PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            //SysCtlDelay(100);
            PN0 = 0x01; // AD9959 I/O update DDS3
            SysCtlDelay(10);
            PN0 = 0x00;
            return true;
        }
        else if (ChannelDDS_UART == 2)
       {
            ssi2PutData(CSR_ADRESS,CSR2_4bit_DDS3,CSR_NUM_BYTE);  //select CH2
            ssi2PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            //SysCtlDelay(100);
            PN0 = 0x01; // AD9959 I/O update DDS3
            SysCtlDelay(10);
            PN0 = 0x00;
            return true;
       }
        else if (ChannelDDS_UART == 3)
       {
            ssi2PutData(CSR_ADRESS,CSR3_4bit_DDS3,CSR_NUM_BYTE);  //select CH3
            ssi2PutData(CPOW_ADRESS, powValue, CPOW_NUM_BYTE);
            powValueAD9959DDS[ChannelDDS_UART-1] = powValue;
            //SysCtlDelay(100);
            PN0 = 0x01; // AD9959 I/O update DDS3
            SysCtlDelay(10);
            PN0 = 0x00;
            return true;
       }
        else
        {
            return false;
        }
    } else
    {
        return false;
    }
}
//////////////////////////////////////////////////////////////
// Get registers for serial communication mode:
//////////////////////////////////////////////////////////////

unsigned long long GetFrequencyTuningWord0(unsigned int DDSnb)
{
    return frequencyValueAD9959DDS[DDSnb-1];
}
unsigned short GetamplitudeValueDDSAD9959(unsigned int DDSnb)
{
    return amplValueAD9959DDS[DDSnb-1];
}
unsigned short GetpowValueDDSAD9959(unsigned int DDSnb)
{
    return powValueAD9959DDS[DDSnb-1];
}

//////////////////////////////////////////////////////////////
// Check command from TCP client and transfer it to DDS:
//////////////////////////////////////////////////////////////
bool getAD9959CommandFromTCP(char* msg_cmd, char* msg_val, char* msg_rpl, unsigned int DDS_nb_value, bool replyWanted)
{
    int compareCmd = 1;
    int enum_counter = 0;
    bool err;
    //int intValue;
    //int ChannelDDS_UART = 0;
    unsigned long unsignedLongValue;                // = 0xFFFFFFFF;
    //unsigned long long unsignedLongLongValue;       // = 0xFFFFFFFFFFFFFFFF;
    unsigned unsignedIntValue;

    while ((compareCmd != 0) & (enum_counter < MAX_NUMBER_ENUM_AD9959DDS_commands))
    {
        compareCmd = ustrcmp(msg_cmd, enum_AD9959DDS_commands_str[enum_counter]);
        enum_counter = enum_counter+1;
    }

    if (compareCmd != 0)
    {
        usprintf(msg_rpl, "Command not recognized");
        return false;
    }

    enum_counter = enum_counter-1;
    err = false;
    switch (enum_counter)
    {
         case Set_AD9959_Frq:
            unsignedLongValue = (0xFFFFFFFF & strtoull(msg_val, NULL, 16));
            start_freq_hz = (0xFFFFFFF & strtoull(msg_val, NULL, 16));
            err = SetFrequencyTuningWord_ethernet((unsigned long)unsignedLongValue, DDS_nb_value);
            if (replyWanted & err) { usprintf(msg_rpl, "Done, DDS%u, frequency 1 = %d ", DDS_nb_value, (unsigned long) unsignedLongValue); }
            break;
        case Set_AD9959_amp:
            unsignedLongValue = (0xFFFF & strtoul(msg_val, NULL, 16));
            err = SetamplValue_ethernet((unsigned short)unsignedLongValue, DDS_nb_value);
            simpleflashMemoryAD9959DDS_ethernet_frequency((unsigned int)DDS_nb_value);
            if (replyWanted & err) { usprintf(msg_rpl, "Done, DDS%u, amplitude = %d ", DDS_nb_value, (unsigned short) unsignedLongValue); }
            break;
        case Set_AD9959_channel:
            unsignedIntValue = (0xFFFF & strtoul(msg_val, NULL, 16));
            ChannelDDS = (0xFFFF & strtoul(msg_val, NULL, 16));
            err = SetchannelDDS((unsigned short)unsignedIntValue, DDS_nb_value);
            if (replyWanted & err) { usprintf(msg_rpl, "Done, DDS%u, Channel DDS = %d ", DDS_nb_value, (unsigned short) unsignedIntValue); }
            break;

        case Set_AD9959_phase: // PHASE (2 octets - UShort)
             unsignedIntValue = (0xFFFF & strtoul(msg_val, NULL, 16));
             // Assurez-vous que SetPhaseOffsetWord_ethernet prend un unsigned short
             err = SetpowValue_ethernet((unsigned short)unsignedIntValue, DDS_nb_value);
             if (replyWanted & err) { usprintf(msg_rpl, "Done, DDS%u, phase = %u", DDS_nb_value, (unsigned short) unsignedIntValue); }
             break;

        case Set_AD9959_Frq2: // FR�QUENCE de FIN (FTW1 - 4 OCTETS - ULong)
             unsignedLongValue = (0xFFFFFFFF & strtoull(msg_val, NULL, 16));
             freq_end_hz = (0xFFFFFFFF & strtoull(msg_val, NULL, 16));
             // Assurez-vous que la fonction utilise le registre FTW1
             err = SetFrequencyTuningWord1_ethernet((unsigned long)unsignedLongValue, DDS_nb_value);
             if (replyWanted & err) { usprintf(msg_rpl, "Done, DDS%u, frequency 2 = %d ", DDS_nb_value, (unsigned long)unsignedLongValue); }
             break;

        case Set_AD9959_Frq_step: // PAS DE TEMPS (4 OCTETS - ULong)
             unsignedLongValue = (0xFFFFFFFF & strtoull(msg_val, NULL, 16));
             freq_step_hz = (0xFFFFFFFF & strtoull(msg_val, NULL, 16));
             // Assurez-vous que la fonction utilise le registre de Pas de Temps
             err = Set_frequency_step((unsigned long)unsignedLongValue, DDS_nb_value);
             if (replyWanted & err) { usprintf(msg_rpl, "Done, DDS%u, FStep = %d ", DDS_nb_value, (unsigned long)unsignedLongValue); }
             break;

        case Set_AD9959_time_step: // PAS DE FR�QUENCE (4 OCTETS - ULong)
             unsignedLongValue = (0xFFFFFFFF & strtoull(msg_val, NULL, 16));
             // Lit la cha�ne 'msg_val' comme un nombre entier long non sign�
             time_step_us = (0xFFFFFFFF & strtoull(msg_val, NULL, 16));
             //Transmet le Mot de Pas de Fr�quence (FSW) de 32 bits
             err = Set_time_step((unsigned long)unsignedLongValue, DDS_nb_value);
             if (replyWanted & err) { usprintf(msg_rpl, "Done, DDS%u, time_step =  %d ", DDS_nb_value, (unsigned long)unsignedLongValue); }
             break;


        case Get_AD9959_AllData:
            usprintf(msg_rpl, "$%x$%x$%x$", (unsigned long) (frequencyValueAD9959DDS[DDS_nb_value-1]), (unsigned long) (0xFFFFFFFF & frequencyValueAD9959DDS[DDS_nb_value-1]), amplValueAD9959DDS[DDS_nb_value-1]);
            break;


        case Load_ROM_AD9959:
            err = loadValuesFromFlashMemoryAD9959DDS(DDS_nb_value);
            if (replyWanted)
            {
                if (err)
                {
                    usprintf(msg_rpl, "Flash memory loaded for DDS%u ", DDS_nb_value);
                } else {
                    usprintf(msg_rpl, "Flash memory NOT loaded for DDS%u ", DDS_nb_value);
                }
            }
            break;
        case Save_ROM_AD9959:
            err = flashMemoryAD9959DDS(DDS_nb_value);
            if (replyWanted)
            {
                if (err)
                {
                    usprintf(msg_rpl, "DDS%u values saved on flash memory ", DDS_nb_value);
                } else {
                    usprintf(msg_rpl, "DDS%u values NOT saved on flash memory ", DDS_nb_value);
                }
            }
            break;
        case Save_ROM_Smpl_AD9959:
            err = simpleflashMemoryAD9959DDS1(DDS_nb_value);
            if (replyWanted)
            {
                if (err)
                {
                    usprintf(msg_rpl, "DDS%u freq1 and ampl saved on flash memory ", DDS_nb_value);
                } else {
                    usprintf(msg_rpl, "DDS%u freq1 and ampl NOT saved on flash memory ", DDS_nb_value);
                }
            }
            break;
    }
    return err;
}
//////////////////////////////////////////////////////////////
// Execute single command (during triggered sequence)
//////////////////////////////////////////////////////////////
void executeAD9959SingleBufferedCmd(int cmdNb, unsigned long value, unsigned int DDS_channel_value)
{
    switch (cmdNb)
    {
        case Set_AD9959_Frq:
            SetFrequencyTuningWord_ethernet((unsigned long) (value & 0xFFFFFFFF), DDS_channel_value);
            break;
        case Set_AD9959_amp:
            SetamplValue_ethernet((unsigned short) (value & 0xFFFF), DDS_channel_value);
            break;
        case Set_AD9959_channel:
            SetchannelDDS((unsigned int) (value & 0xFFFF), DDS_channel_value);
            break;
        case Set_AD9959_phase:
            SetpowValue_ethernet((unsigned short) (value & 0xFFFF), DDS_channel_value);
           break;
        case Set_AD9959_Frq2:
            SetFrequencyTuningWord1_ethernet((unsigned long) (value & 0xFFFF), DDS_channel_value);
           break;
        case Set_AD9959_Frq_step:
            Set_frequency_step((unsigned long) (value & 0xFFFF), DDS_channel_value);
           break;
        case Set_AD9959_time_step:
             Set_time_step((unsigned long) (value & 0xFFFF), DDS_channel_value);
           break;

    }
}

