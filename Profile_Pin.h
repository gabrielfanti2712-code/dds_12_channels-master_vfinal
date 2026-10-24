/*
 * AD9959.h
 *
 *  Created on: 09/10/2025
 *      Author: F Wiotte
 */

#ifndef Profile_Pin_H_
#define Profile_Pin_H_

#include <stdbool.h>

#define PH0 (*((volatile unsigned long *)0x4005F004))  //P3 DDS1
#define PH1 (*((volatile unsigned long *)0x4005F008))  //P2 DDS1
#define PH2 (*((volatile unsigned long *)0x4005F010))  //P1 DDS1
#define PH3 (*((volatile unsigned long *)0x4005F020))  //P0 DDS1

#define PP2 (*((volatile unsigned long *)0x40065010))  //P3 DDS2
#define PP3 (*((volatile unsigned long *)0x40065020))  //P2 DDS2
#define PP4 (*((volatile unsigned long *)0x40065040))  //P1 DDS2
#define PP5 (*((volatile unsigned long *)0x40065080))  //P0 DDS2

#define PM0 (*((volatile unsigned long *)0x40063004))  //P3 DDS3
#define PM1 (*((volatile unsigned long *)0x40063008))  //P2 DDS3
#define PM2 (*((volatile unsigned long *)0x40063010))  //P1 DDS3
#define PM3 (*((volatile unsigned long *)0x40063020))  //P0 DDS3

/*
PH0 = 0x01;  //P3 DDS1
PH1 = 0x02;  //P2 DDS1
PH2 = 0x04;  //P1 DDS1
PH3 = 0x08;  //P0 DDS0

PH0 = 0x00;  //P3 DDS1
PH1 = 0x00;  //P2 DDS1
PH2 = 0x00;  //P1 DDS1
PH3 = 0x00;  //P0 DDS1

PP2 = 0x04;  //P3 DDS2
PP3 = 0x08;  //P2 DDS2
PP4 = 0x10;  //P1 DDS2
PP5 = 0x20;  //P0 DDS2

PP2 = 0x00;  //P3 DDS2
PP3 = 0x00;  //P2 DDS2
PP4 = 0x00;  //P1 DDS2
PP5 = 0x00;  //P0 DDS2

PM0 = 0x01;  //P3 DDS3
PM1 = 0x02;  //P2 DDS3
PM2 = 0x04;  //P1 DDS3
PM3 = 0x08;  //P0 DDS3

PM0 = 0x00;  //P3 DDS3
PM1 = 0x00;  //P2 DDS3
PM2 = 0x00;  //P1 DDS3
PM3 = 0x00;  //P0 DDS3
*/

#endif /* Profile_Pin_H_ */
