/*
 * Table.c
 *
 *  Created on: Nov 6, 2017
 *      Author: KLam
 */

#include <stdint.h>
#include "thermal_print.h"
//Table for strobe and motor timer
#ifdef USE_5NK_MOTOR
const uint16_t u16StbTempComp[TABLE_SIZE]={
		18	,
		18	,
		18	,
		18	,
		17	,
		16	,
		14	,
		12	,
		10	,
		9	,
		8	,
		8	,
		8	,
		7	,
		7	,
		7	,
		7	,
		7
};

const uint16_t u16StbDarkness[TABLE_SIZE]={
		18	,
		18	,
		18	,
		18	,
		17	,
		16	,
		14	,
		12	,
		10	,
		9	,
		9	,
		9	,
		9	,
		9	,
		9	,
		9	,
		9	,
		9
};
const uint16_t u16StrobeTable[TABLE_SIZE]={
		183	,
		185	,
		186	,
		181	,	//3IPS
		173	,
		161	,
		143	,
		123	,	//4IPS
		103	,
		91	,
		83	,
		78	,
		76	,	//5IPS
		75	,
		74	,
		74	,
		74	,
		74		//6IPS
};

const uint16_t u16StepperAccTableHalf[TABLE_SIZE]={
		1124	,
		1124	,
		1124	,
		1087	,	//3IPS
		1031	,
		952	,
		840	,
		719	,	//4IPS
		599	,
		524	,
		474	,
		444	,
		429	,	//5IPS
		420	,
		415	,
		412	,
		410	,
		410		//6IPS
};

const uint8_t u8MTRCurrLimitTable[TABLE_SIZE]={
		199	,
		199	,
		199	,
		180	,	//3IPS
		180	,
		180	,
		180	,
		160	,	//4IPS
		160	,
		160	,
		160	,
		160	,
		150	,	//5IPS
		150	,
		150	,
		150	,
		150	,
		150	//6IPS
};
#else
#endif
//Table for PH NTC temperature calculation
const uint16_t u16PHTempTblM[NTC_TABLE_SIZE] = {
		3085	,
		2919	,
		2741	,
		2557	,
		2369	,
		2181	,
		1996	,
		1816	,
		1644	,
		1482	,
		1332	,
		1193	,
		1066	,
		951	,
		848	,
		755	,
		672	,
		598	,
		533	,
		475	,
		424	,
		379	,
		339	,
		303	,
		272	,
		244
};

const uint16_t u16PHTempTblL[NTC_TABLE_SIZE] = {
		42	,
		44	,
		46	,
		47	,
		47	,
		46	,
		45	,
		43	,
		40	,
		38	,
		35	,
		32	,
		29	,
		26	,
		23	,
		21	,
		18	,
		16	,
		14	,
		13	,
		11	,
		10	,
		9	,
		8	,
		7	,
		6
};
