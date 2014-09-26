/*
 * Interrupts.c
 *
 *  Created on: Sep 25, 2014
 *      Author: DLThrasher
 */

//*****************************************************************************
//		Includes
//*****************************************************************************
#include "Interrupts.h"
#include "Timers.h"
#include "Ultrasonic.h"

//*****************************************************************************
//		Local Data Variables
//*****************************************************************************
//*****************************************************************************
//
// This is the handler for INT_GPIOC.  It triggers INT_GPIOB and saves the
// interrupt sequence number.
//
//*****************************************************************************
static BOOLEAN risingEdgeSeen = FALSE;
static uint16_t beginTime, echoWidth; // Must stay uint16 to do 16bit timer math

//*****************************************************************************
//		Global Functions
//*****************************************************************************
void IntGPIOd(void)
{
	GPIOIntClear(ULTRASONIC_SENSOR_PORT, ECHO_PIN);
	//Rising Edge
	if(risingEdgeSeen == FALSE)
	{
		beginTime = TIMER0_COUNT_US();
		risingEdgeSeen = TRUE;
	}
	else
	{
		//Counts down
		echoWidth = beginTime - TIMER0_COUNT_US();
		UARTprintf("Pulse width = %d\n",echoWidth);
		//Need to read a timer at rising and falling edge to determine the distance of the ping
		//cm = microseconds/29/2
		//in = microseconds/72/2
		Ultrasonic_SetDistanceCM(echoWidth/29/2);
		UARTprintf("Centimeters = %d\n", Ultrasonic_GetDistanceCM());
		risingEdgeSeen = FALSE;

		//We have heard the echo
		Ultrasonic_SetWaitingForEcho(FALSE);
	}
}
