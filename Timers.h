/*
 * Timers.h
 *
 *  Created on: Sep 25, 2014
 *      Author: DLThrasher
 */

#ifndef TIMERS_H_
#define TIMERS_H_

//============================================================
//Includes
//============================================================
#include "LEDDistanceSynthesizer.h"

//============================================================
//Defines
//============================================================
/*!
 *  \brief Init timer module
 */
#define SYSTEM_TIMER_MHZ 	(16)
#define TIMER0 				(TIMER0_BASE)
#define TIMER0_PRESCALER 	(16)
#define TIMER0_COUNT_US()	(TimerValueGet(TIMER0, TIMER_A)/SYSTEM_TIMER_MHZ)
//============================================================
//Global Functions
//============================================================
extern void Timers__Initialize(void);			//Timers.c

#endif /* TIMERS_H_ */
