/*
 * HelperFunctions.c
 *
 *  Created on: Sep 25, 2014
 *      Author: DLThrasher
 */
//*****************************************************************************
//		Includes
//*****************************************************************************
#include "HelperFunctions.h"
#include "LEDDistanceSynthesizer.h"

//*****************************************************************************
//		Global Functions
//*****************************************************************************
//*****************************************************************************
//
// Delay for the specified number of seconds.  Depending upon the current
// SysTick value, the delay will be between N-1 and N seconds (i.e. N-1 full
// seconds are guaranteed, aint32_t with the remainder of the current second).
//
//*****************************************************************************
void Delay(uint32_t ui32Seconds)
{
	//
	// Loop while there are more seconds to wait.
	//
	while (ui32Seconds--) {
		//
		// Wait until the SysTick value is less than 1000.
		//
		while (SysTickValueGet() > 1000) {
		}

		//
		// Wait until the SysTick value is greater than 1000.
		//
		while (SysTickValueGet() < 1000) {
		}
	}
}

void DelayMS(uint32_t ui32MilliSeconds)
{
	//
	// Loop while there are more seconds to wait.
	//
	while (ui32MilliSeconds--)
	{
		//
		// Wait until the SysTick value is less than 1000.
		//
		while (SysTickValueGet() > 1)
		{
		}

		//
		// Wait until the SysTick value is greater than 1000.
		//
		while (SysTickValueGet() < 1)
		{
		}
	}
}


