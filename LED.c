/*
 * LED.c
 *
 *  Created on: Sep 25, 2014
 *      Author: DLThrasher
 */

//*****************************************************************************
//		Includes
//*****************************************************************************
#include "LED.h"

//============================================================
//Global Functions
//============================================================
void LED_ChangeColor(LED_COLOR color)
{
	switch(color)
	{
		case LED_OFF:
		{
			GPIOPinWrite(LED_PORT, LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN, CLEAR);
			break;
		}
		case LED_RED:
		{
			GPIOPinWrite(LED_PORT, LED_RED_PIN, LED_RED_PIN);
			GPIOPinWrite(LED_PORT, LED_GREEN_PIN | LED_BLUE_PIN, CLEAR);
			break;
		}
		case LED_RED_GREEN:
		{
			GPIOPinWrite(LED_PORT, LED_RED_PIN | LED_GREEN_PIN, LED_RED_PIN | LED_GREEN_PIN);
			GPIOPinWrite(LED_PORT, LED_BLUE_PIN, CLEAR);
			break;
		}
		case LED_GREEN:
		{
			GPIOPinWrite(LED_PORT, LED_GREEN_PIN, LED_GREEN_PIN);
			GPIOPinWrite(LED_PORT, LED_RED_PIN | LED_BLUE_PIN, CLEAR);
			break;
		}
		case LED_GREEN_BLUE:
		{
			GPIOPinWrite(LED_PORT, LED_GREEN_PIN | LED_BLUE_PIN, LED_GREEN_PIN | LED_BLUE_PIN);
			GPIOPinWrite(LED_PORT, LED_RED_PIN, CLEAR);
			break;
		}
		case LED_BLUE:
		{
			GPIOPinWrite(LED_PORT, LED_BLUE_PIN, LED_BLUE_PIN);
			GPIOPinWrite(LED_PORT, LED_RED_PIN | LED_GREEN_PIN, CLEAR);
			break;
		}
		case LED_RED_BLUE:
		{
			GPIOPinWrite(LED_PORT, LED_RED_PIN | LED_BLUE_PIN, LED_RED_PIN | LED_BLUE_PIN);
			GPIOPinWrite(LED_PORT, LED_GREEN_PIN, CLEAR);
			break;
		}
		case LED_RED_GREEN_BLUE:
		{
			GPIOPinWrite(LED_PORT, LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN, LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN);
			break;
		}
		default:
		{
			break;
		}
	}
}

