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
#include "inc/tm4c123gh6pm.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"

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

typedef enum
{
	FALSE,
	TRUE
}BOOLEAN;

//Typedef for LED.h
	typedef enum
	{
		LED_OFF,
		LED_RED,
		LED_RED_GREEN,
		LED_GREEN,
		LED_GREEN_BLUE,
		LED_BLUE,
		LED_RED_BLUE,
		LED_RED_GREEN_BLUE,
		//This number must always be last!!!!!
		LED_MAX_NUMBER_COLORS
	}LED_COLOR;
#define LED_PORT			(GPIO_PORTF_BASE)
#define LED_RED_PIN 		(GPIO_PIN_1)
#define LED_GREEN_PIN		(GPIO_PIN_3)
#define LED_BLUE_PIN		(GPIO_PIN_2)

#define ULTRASONIC_SENSOR_PORT	(GPIO_PORTD_BASE)
#define ECHO_PIN				(GPIO_PIN_3)
#define TRIGGER_PIN				(GPIO_PIN_6)

#define CLEAR	(0)
#define SET		(1)
//*****************************************************************************
//		GLOBAL FUNCTION PROTOTYPES
//*****************************************************************************
void LED__ChangeColor(LED_COLOR);	//LED.c
void IntGPIOd(void);				//Interupts.c
void Delay(uint32_t);   //HelperFunctions.c
void DelayMS(uint32_t);   //HelperFunctions.c
void Ultrasonic__SetTrigger(void);	//Ultrasonic.c
void Timers__Initialize(void);			//Timers.c
void GPIO_Initialization(void);		//GPIO.c
//*****************************************************************************
//		LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void configureUART(void);
void IntGPIOd(void);
//*****************************************************************************
//		GLOBAL DATA VARIABLES
//*****************************************************************************
BOOLEAN WaitingForEcho = FALSE;
int DistanceCM = 0;
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
		switch((DistanceCM % 14)/2)
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

void LED__ChangeColor(LED_COLOR color)
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

/*!
 * \brief lets make an led blink every 2 seconds
 */
void Blink_LED_2_Seconds_Test()
{

}



/*!
 *  \brief Init timer module
 */
#define SYSTEM_TIMER_MHZ 	(16)
#define TIMER0 				(TIMER0_BASE)
#define TIMER0_PRESCALER 	(16)
#define TIMER0_COUNT_US()	(TimerValueGet(TIMER0, TIMER_A)/SYSTEM_TIMER_MHZ)
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
/*!
 * \brief setup the timer for capture compare or to fire every 2 seconds
 * \note make sure that the prescale times the 16 MHz clock results in a tick that can be equated to 2 seconds.
 */
void initializeCCRTimer()
{

}

/*!
 * \brief Init GPIO
 */
void GPIO_Initialization(void)
{
	//
	// Enable the peripherals used by this example.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);


	//Set up Port D Pin 3 (PD3) as a both edge interrupt
	 // Set up our Echo interrupt
	GPIOPinTypeGPIOInput(ULTRASONIC_SENSOR_PORT, ECHO_PIN);
	IntMasterEnable();																// Allow interrupts
	GPIOIntTypeSet(ULTRASONIC_SENSOR_PORT, ECHO_PIN, GPIO_BOTH_EDGES);
	GPIOIntClear(ULTRASONIC_SENSOR_PORT, ECHO_PIN);
	GPIOIntEnable(ULTRASONIC_SENSOR_PORT, ECHO_PIN);
	//
	// Enable Port D at echo timer.
	//
	IntEnable(INT_GPIOD);
	//
	// Set the interrupt priorities so they are all equal.
	//
	IntPrioritySet(INT_GPIOD, 0x01);


	//Set up our Trigger
	GPIOPinTypeGPIOOutput(ULTRASONIC_SENSOR_PORT, TRIGGER_PIN);
	GPIOPinWrite(ULTRASONIC_SENSOR_PORT, 0xFF, CLEAR);

	//
	// Configure the PF1-PF3 to be outputs to indicate entry/exit of one
	// of the interrupt handlers.
	//
	GPIOPinTypeGPIOOutput(LED_PORT, LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN);
}
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
//*****************************************************************************
//
// This is the handler for INT_GPIOC.  It triggers INT_GPIOB and saves the
// interrupt sequence number.
//
//*****************************************************************************
static BOOLEAN risingEdgeSeen = FALSE;
static uint16_t beginTime, echoWidth;
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
		DistanceCM = (echoWidth/29/2);
		UARTprintf("Centimeters = %d\n", DistanceCM);
		risingEdgeSeen = FALSE;

		//We have heard the echo
		WaitingForEcho = FALSE;
	}
}

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
