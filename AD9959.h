/*
 * AD9959.h
 *
 *  Created on: 09/10/25
 *      Author: F Wiotte
 */

#ifndef AD9959_H_
#define AD9959_H_

#include <stdbool.h>

//AD9959//
// AD9959 Register Layout
#define FTW_ADRESS             ((int)0x04)            // Channel Frequency Tuning Word 0 (CFTW0)�Address 0x04
#define FTW_NUM_BYTE           ((int)0x04)            // Frequency Tuning Word  number of bytes

#define CSR_ADRESS             ((int)0x00)            // AD9959 Channel Select Register (CSR)
#define CSR_NUM_BYTE           ((int)0x01)            // AD9959 byte number by CSR

#define ACR_ADRESS             ((int)0x06)            // AD9959 Amplitude Control Register (ACR)
#define ACR_NUM_BYTE           ((int)0x03)            // AD9959 byte number by ACR

#define FR1_ADRESS             ((int)0x01)            // FR1 is used to control the mode of operation of the chip
#define FR1_NUM_BYTE           ((int)0x03)            // AD9959 byte number by FR1

#define FR2_ADRESS             ((int)0x02)            // The FR2 is used to control the various functions, features, and modes of the AD9959.
#define FR2_NUM_BYTE           ((int)0x02)            // AD9959 byte number by FR2

#define CFR_ADRESS             ((int)0x03)            // Channel Function Register (CFR)�Address 0x03
#define CFR_NUM_BYTE           ((int)0x03)            // AD9959 byte number by CFR

#define CPOW_ADRESS            ((int)0x05)            // Channel Phase Offset Word (CPOW)
#define CPOW_NUM_BYTE          ((int)0x02)            // AD9959 byte number by CPOW0

#define LSRR_ADRESS            ((int)0x07)            // Linear Sweep Ramp Rate (LSRR) -> Utilis� comme Vitesse du Pas  Falling sweep ramp rate (FSRR)[15:8]  &  Rising sweep ramp rate (RSRR)[7:0]
#define LSRR_NUM_BYTE          ((int)0x02)            // AD9959 byte number by LSRR

#define RDW_ADRESS             ((int)0x08)            // RDW Rising Delta Word (RDW)�Address 0x08 -> Utilis� comme Taille du Pas (Mont�e)
#define RDW_NUM_BYTE           ((int)0x04)            // AD9959 byte number by RDW

#define FDW_ADRESS             ((int)0x09)            // LSR Falling Delta Word (FDW)�Address 0x09
#define FDW_NUM_BYTE           ((int)0x04)            // AD9959 byte number by FDW

#define FTW1_ADRESS            ((int)0x0A)            // Channel Frequency Tuning Word 1 (CFTW1)�Address 0x0A
#define FTW1_NUM_BYTE          ((int)0x04)            // Frequency Tuning Word  number of bytes

#define FTW2_ADRESS            ((int)0x0B)            // Channel Frequency Tuning Word 2 (CFTW2)�Address 0x0B
#define FTW2_NUM_BYTE          ((int)0x04)            // Frequency Tuning Word  number of bytes
/*
 *
 *   DDS1
 *   Default values of the DDS1 after reboot/master reset
 *   Author: F Wiotte
 */
#define FTW0_DDS1          ((unsigned long)   0x51EB852)   //fout = 10MHz
#define FTW1_DDS1          ((unsigned long)   0x51EB852)
#define FTW2_DDS1          ((unsigned long)   0x51EB852)
#define FTW3_DDS1          ((unsigned long)   0x51EB852)

#define CSR0_DDS1  ((int)   0x10)     // AD9959 CH0  MSB first
#define CSR1_DDS1  ((int)   0x20)     // AD9959 CH1  MSB first
#define CSR2_DDS1  ((int)   0x40)     // AD9959 CH2  MSB first
#define CSR3_DDS1  ((int)   0x80)     // AD9959 CH3  MSB first

#define CSR0_4bit_DDS1  ((int)   0x16)        // AD9959 CH0  MSB first  4-bit serial mode
#define CSR1_4bit_DDS1  ((int)   0x26)        // AD9959 CH1  MSB first  4-bit serial mode
#define CSR2_4bit_DDS1  ((int)   0x46)        // AD9959 CH2  MSB first  4-bit serial mode
#define CSR3_4bit_DDS1  ((int)   0x86)        // AD9959 CH3  MSB first  4-bit serial mode

#define ACR0_DDS1  ((unsigned long)   0x1399)    // full DAC 10bits OV output bit 12 � 1 in ACR register
#define ACR1_DDS1  ((unsigned long)   0x1399)
#define ACR2_DDS1  ((unsigned long)   0x1399)
#define ACR3_DDS1  ((unsigned long)   0x1399)

// delta phi = CPOW * 2^14/360�//
#define CPOW0_DDS1  ((unsigned int))    //  Phase Offset Word 0 (CPOW0) 14-bit resolution ($2^{14} = 16384 possible phase values).
#define CPOW1_DDS1  ((unsigned int))    //  Phase Offset Word 0 (CPOW1) 14-bit resolution ($2^{14} = 16384 possible phase values).
#define CPOW2_DDS1  ((unsigned int))    //  Phase Offset Word 0 (CPOW2) 14-bit resolution ($2^{14} = 16384 possible phase values).
#define CPOW3_DDS1  ((unsigned int))    //  Phase Offset Word 0 (CPOW3) 14-bit resolution ($2^{14} = 16384 possible phase values).

/*
 *
 *   DDS2
 *   Default values of the DDS2 after reboot/master reset
 *   Author: F Wiotte
 */
#define FTW0_DDS2          ((unsigned long)   0x51EB852)
#define FTW1_DDS2          ((unsigned long)   0x51EB852)
#define FTW2_DDS2          ((unsigned long)   0x51EB852)
#define FTW3_DDS2          ((unsigned long)   0x51EB852)

#define CSR0_DDS2  ((int)   0x10)     // AD9959 CH0  MSB first
#define CSR1_DDS2  ((int)   0x20)     // AD9959 CH1  MSB first
#define CSR2_DDS2  ((int)   0x40)     // AD9959 CH2  MSB first
#define CSR3_DDS2  ((int)   0x80)     // AD9959 CH3  MSB first

#define CSR0_4bit_DDS2  ((int)   0x16)          // AD9959 CH0  MSB first  4-bit serial mode
#define CSR1_4bit_DDS2  ((int)   0x26)          // AD9959 CH1  MSB first  4-bit serial mode
#define CSR2_4bit_DDS2  ((int)   0x46)          // AD9959 CH2  MSB first  4-bit serial mode
#define CSR3_4bit_DDS2  ((int)   0x86)          // AD9959 CH3  MSB first  4-bit serial mode

#define ACR0_DDS2  ((unsigned long)   0x1399)    // full DAC 10bits OV output bit 12 � 1 in ACR register
#define ACR1_DDS2  ((unsigned long)   0x1399)
#define ACR2_DDS2  ((unsigned long)   0x1399)
#define ACR3_DDS2  ((unsigned long)   0x1399)
/*
 *   DDS3
 *   Default values of the DDS3 after reboot/master reset
 *   Author: F Wiotte
 */
#define FTW0_DDS3          ((unsigned long)   0x51EB852)
#define FTW1_DDS3          ((unsigned long)   0x51EB852)
#define FTW2_DDS3          ((unsigned long)   0x51EB852)
#define FTW3_DDS3          ((unsigned long)   0x51EB852)

#define CSR0_DDS3  ((int)   0x10)     // AD9959 CH0  MSB first
#define CSR1_DDS3  ((int)   0x20)     // AD9959 CH1  MSB first
#define CSR2_DDS3  ((int)   0x40)     // AD9959 CH2  MSB first
#define CSR3_DDS3  ((int)   0x80)     // AD9959 CH3  MSB first

#define CSR0_4bit_DDS3  ((int)   0x16)    // AD9959 CH0  MSB first  4-bit serial mode
#define CSR1_4bit_DDS3 ((int)   0x26)    // AD9959 CH1  MSB first  4-bit serial mode
#define CSR2_4bit_DDS3  ((int)   0x46)    // AD9959 CH2  MSB first  4-bit serial mode
#define CSR3_4bit_DDS3  ((int)   0x86)    // AD9959 CH3  MSB first  4-bit serial mode

#define ACR0_DDS3  ((unsigned long)   0x1399)    // full DAC 10bits OV output bit 12 � 1 in ACR register
#define ACR1_DDS3  ((unsigned long)   0x1399)
#define ACR2_DDS3  ((unsigned long)   0x1399)
#define ACR3_DDS3  ((unsigned long)   0x1399)

// Default values of the DDSs AD9959 after reboot/master reset
#define frequencyValueAD9959DDSOnReboot          ((unsigned long )   0)
#define amplValueAD9959DDSOnReboot                ((unsigned short)   0)
#define powValueAD9959DDSOnReboot                 ((unsigned short)   0)

enum AD9959DDS_commands
{
    Set_AD9959_Frq,
    Set_AD9959_amp,
    Set_AD9959_channel,
    Set_AD9959_phase,
    Set_AD9959_Frq2,
    Set_AD9959_Frq_step,
    Set_AD9959_time_step,
    Get_AD9959_AllData,
    Load_ROM_AD9959,
    Save_ROM_AD9959,
    Save_ROM_Smpl_AD9959,
    MAX_NUMBER_ENUM_AD9959DDS_commands
};

static const char *enum_AD9959DDS_commands_str[] =
        {"Frq1", "Amp","channel","Phase","Frq2","step","time_step","Load_ROM_AD9959",
         "Save_ROM_AD9959",
         "Save_ROM_Smpl_AD9959"
        };

// Set registers for serial communication mode:
bool SetFrequencyTuningWord0_DDS1(unsigned long frequencyValue, unsigned int ChannelDDS_UART);
//bool SetFrequencyTuningWord0_DDS1_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART);

bool SetFrequencyTuningWord0_DDS2(unsigned long frequencyValue, unsigned int ChannelDDS_UART);
//bool SetFrequencyTuningWord0_DDS2_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART);

bool SetFrequencyTuningWord0_DDS3(unsigned long frequencyValue, unsigned int ChannelDDS_UART);
//bool SetFrequencyTuningWord0_DDS3_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART);

bool Set_frequency_step(unsigned long frequencyValue, unsigned int ChannelDDS_UART);
bool Set_time_step(unsigned long time_step_s, unsigned int ChannelDDS_UART);

bool SetFrequencyTuningWord_ethernet(unsigned long frequencyValue, unsigned int ChannelDDS_UART);
//bool SetFrequencyTuningWord_ethernet_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART);

bool SetFrequencyTuningWord1_ethernet(unsigned long frequencyValue, unsigned int ChannelDDS_UART);
bool SetamplValue_ethernet(unsigned short amplValue, unsigned int ChannelDDS_UART);
bool SetpowValue_ethernet(unsigned short powValue, unsigned int ChannelDDS_UART);

bool SetamplValueAD9959DDS1(unsigned short amplValue, unsigned int ChannelDDS_UART);
bool SetamplValueAD9959DDS2(unsigned short amplValue, unsigned int ChannelDDS_UART);
bool SetamplValueAD9959DDS3(unsigned short amplValue, unsigned int ChannelDDS_UART);
bool SetchannelDDS(unsigned int Value, unsigned int ChannelDDS_UART);

bool SetPOW_ValueAD9959DDS1(unsigned short powValue, unsigned int ChannelDDS_UART);
bool SetPOW_ValueAD9959DDS2(unsigned short powValue, unsigned int ChannelDDS_UART);
bool SetPOW_ValueAD9959DDS3(unsigned short powValue, unsigned int ChannelDDS_UART);

// Get registers for serial communication mode:
unsigned long  long GetFrequencyTuningWord0();
unsigned short GetamplitudeValueDDSAD9959();
unsigned short GetpowValueDDSAD9959();

// General functions:
extern void executeAD9959SingleBufferedCmd(int cmdNb, unsigned long value, unsigned int ChannelDDS_UART);

void Apply_Sweep_Configuration_uart(unsigned int NbDDS_UART,unsigned int ChannelDDS_UART,unsigned long start_freq_hz,unsigned long end_freq_hz,unsigned long step_freq_hz,double time_step_s);
void Apply_Sweep_Configuration_ethernet(unsigned int CURRENT_DDS,unsigned int ChannelDDS,unsigned long start_freq_hz,unsigned long end_freq_hz,unsigned long step_freq_hz,double time_step_s);
// Check command from TCP client and transfer it to DDS:
extern bool getAD9959CommandFromTCP(char* msg_cmd, char* msg_val, char* msg_rpl, unsigned int DDS_nb_value, bool replyWanted);
#endif /* AD9959_H_ */
