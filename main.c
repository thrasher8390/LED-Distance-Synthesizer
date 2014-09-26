/*
 * Main.c
 *
 *  Created on: March 25, 2014
 *      Author: DLThrasher
 */
//*****************************************************************************
//		Includes
//*****************************************************************************
#include "main.h"
#include "LEDDistanceSynthesizer.h"





//*****************************************************************************
//		GLOBAL FUNCTION PROTOTYPES
//*****************************************************************************
#include "LED.h"
#include "HelperFunctions.h"
#include "Timers.h"
#include "GPIO.h"
#include "Ultrasonic.h"
#include "UART.h"
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



///Main Loop
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
	UART_Initialize();

	//InitializeTimers
	Timers__Initialize();
	//Initialize GPIO
	GPIO_Initialize();

	//
	// Indicate that the equal interrupt priority test is beginning.
	//
	UARTprintf("\nUltraSonic LED Synthesizer\n");

	int ledDelayMS = 1;
	LED_ChangeColor(LED_RED);
	Delay(ledDelayMS);
	LED_ChangeColor(LED_RED_GREEN);
	Delay(ledDelayMS);
	LED_ChangeColor(LED_OFF);

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
			LED_ChangeColor(LED_RED);
					break;
		case 1:
			LED_ChangeColor(LED_RED_GREEN);
					break;
		case 2:
			LED_ChangeColor(LED_GREEN);
					break;
		case 3:
			LED_ChangeColor(LED_GREEN_BLUE);
					break;
		case 4:
			LED_ChangeColor(LED_BLUE);
					break;
		case 5:
			LED_ChangeColor(LED_RED_BLUE);
					break;
		case 6:
			LED_ChangeColor(LED_RED_GREEN_BLUE);
					break;
		default:
			LED_ChangeColor(LED_OFF);
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
