/*
 * GPIO.c
 *
 *  Created on: Sep 25, 2014
 *      Author: DLThrasher
 */

//*****************************************************************************
//		Includes
//*****************************************************************************
#include "GPIO.h"
#include "LED.h"
#include "Ultrasonic.h"

//*****************************************************************************
//		Global Functions
//*****************************************************************************
/*!
 * \brief Init GPIO
 */
void GPIO_Initialize(void)
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
