/*
 * Ultrasonic.c
 *
 *  Created on: Sep 25, 2014
 *      Author: DLThrasher
 */


#include "Ultrasonic.h"
#include "GPIO.h"



//============================================================
//Global Variables
//============================================================
BOOLEAN WaitingForEcho = FALSE;
int DistanceCM = 0;
//============================================================
//Global Functions
//============================================================
/*!
 * \brief Will set triger pin high for x ms.
 */
void Ultrasonic__SetTrigger(void)
{
	uint32_t i;
	//Set the trigger pin high
	GPIOPinWrite(ULTRASONIC_SENSOR_PORT, TRIGGER_PIN, TRIGGER_PIN);
	//wait some time
	for(i = 0; i<400; i++)
	{

	}
	//Set the trigger pin back to low
	GPIOPinWrite(ULTRASONIC_SENSOR_PORT, TRIGGER_PIN, CLEAR);
	WaitingForEcho = TRUE;
}

int Ultrasonic_GetDistanceCM(void)
{
	return DistanceCM;
}
void Ultrasonic_SetDistanceCM(int distanceCM)
{
	DistanceCM = distanceCM;
}
void Ultrasonic_SetWaitingForEcho(BOOLEAN isWaiting)
{
	WaitingForEcho = isWaiting;
}
//============================================================
//Local Functions
//============================================================
