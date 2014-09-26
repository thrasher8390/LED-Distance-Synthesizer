/*
 * Timers.c
 *
 *  Created on: Sep 25, 2014
 *      Author: DLThrasher
 */

//*****************************************************************************
//		Includes
//*****************************************************************************
#include "Timers.h"

//*****************************************************************************
//		Global Functions
//*****************************************************************************
void Timers__Initialize(void)
{
	//DisableTimer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerDisable(TIMER0, TIMER_A);

	//Periodic / TAMIE
	TimerConfigure(TIMER0, TIMER_CFG_A_PERIODIC);

	//Set prescaler. Not really sure what clock we are running from
	TimerPrescaleSet(TIMER0, TIMER_A, TIMER0_PRESCALER);

	//Enable Timer
	TimerEnable(TIMER0, TIMER_A);
}
