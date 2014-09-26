//*****************************************************************************
//
// interrupts.c - Interrupt preemption and tail-chaining example.
//
// Copyright (c) 2012-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 1.1 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************
#include "main.h"
#include "LEDDistanceSynthesizer.h"


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Interrupts (interrupts)</h1>
//!
//! This example application demonstrates the interrupt preemption and
//! tail-chaining capabilities of Cortex-M4 microprocessor and NVIC.  Nested
//! interrupts are synthesized when the interrupts have the same priority,
//! increasing priorities, and decreasing priorities.  With increasing
//! priorities, preemption will occur; in the other two cases tail-chaining
//! will occur.  The currently pending interrupts and the currently executing
//! interrupt will be displayed on the display; GPIO pins E1, E2 and E3 will
//! be asserted upon interrupt handler entry and de-asserted before interrupt
//! handler exit so that the off-to-on time can be observed with a scope or
//! logic analyzer to see the speed of tail-chaining (for the two cases where
//! tail-chaining is occurring).
//
//*****************************************************************************


//*****************************************************************************
//		GLOBAL FUNCTION PROTOTYPES
//*****************************************************************************
#include "LED.h"
void IntGPIOd(void);				//Interupts.c
#include "HelperFunctions.h"
#include "Timers.h"
#include "GPIO.h"
#include "Ultrasonic.h"

//*****************************************************************************
//		LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void configureUART(void);

//*****************************************************************************
//		GLOBAL DATA VARIABLES
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif



//*****************************************************************************
//
// This is the main example program.  It checks to see that the interrupts are
// processed in the correct order when they have identical priorities,
// increasing priorities, and decreasing priorities.  This exercises interrupt
// preemption and tail chaining.
//
//*****************************************************************************
int main(void)
{
	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	FPULazyStackingEnable();

	//
	// Set the clocking to run directly from the crystal.
	//
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN| SYSCTL_XTAL_16MHZ);

	//
	// Set up and enable the SysTick timer.  It will be used as a reference
	// for delay loops in the interrupt handlers.  The SysTick timer period
	// will be set up for one second.
	//
	SysTickPeriodSet(SysCtlClockGet());
	SysTickEnable();

	//
	// Initialize the UART.
	//
	configureUART();

	//InitializeTimers
	Timers__Initialize();

	GPIO_Initialization();

	//
	// Indicate that the equal interrupt priority test is beginning.
	//
	UARTprintf("\nUltraSonic LED Synthesizer\n");

	int ledDelayMS = 1;

	LED__ChangeColor(LED_RED);
	Delay(ledDelayMS);
	LED__ChangeColor(LED_RED_GREEN);
	Delay(ledDelayMS);
	LED__ChangeColor(LED_GREEN);
	Delay(ledDelayMS);
	LED__ChangeColor(LED_GREEN_BLUE);
	Delay(ledDelayMS);
	LED__ChangeColor(LED_BLUE);
	Delay(ledDelayMS);
	LED__ChangeColor(LED_RED_BLUE);
	Delay(ledDelayMS);
	LED__ChangeColor(LED_RED_GREEN_BLUE);
	Delay(ledDelayMS);
	LED__ChangeColor(LED_OFF);
	//
	// Loop forever.
	//
	while (1)
	{
		//Set Trigger
		Ultrasonic__SetTrigger();
		//while(WaitingForEcho);
		//Delay(1);
		switch((Ultrasonic_GetDistanceCM() % 14)/2)
		{
		case 0:
			LED__ChangeColor(LED_RED);
					break;
		case 1:
			LED__ChangeColor(LED_RED_GREEN);
					break;
		case 2:
			LED__ChangeColor(LED_GREEN);
					break;
		case 3:
			LED__ChangeColor(LED_GREEN_BLUE);
					break;
		case 4:
			LED__ChangeColor(LED_BLUE);
					break;
		case 5:
			LED__ChangeColor(LED_RED_BLUE);
					break;
		case 6:
			LED__ChangeColor(LED_RED_GREEN_BLUE);
					break;
		default:
			LED__ChangeColor(LED_OFF);
					break;

		}
	}
}



/*!
 * \brief lets make an led blink every 2 seconds
 */
void Blink_LED_2_Seconds_Test()
{

}




/*!
 * \brief setup the timer for capture compare or to fire every 2 seconds
 * \note make sure that the prescale times the 16 MHz clock results in a tick that can be equated to 2 seconds.
 */
void initializeCCRTimer()
{

}







//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
static void configureUART(void)
{
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Enable UART0
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//
	// Configure GPIO Pins for UART mode.
	//
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	//
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioConfig(0, 115200, 16000000);
}
