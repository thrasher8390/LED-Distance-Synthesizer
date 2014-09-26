/*
 * LED.h
 *
 *  Created on: Sep 25, 2014
 *      Author: DLThrasher
 */

#ifndef LED_H_
#define LED_H_

#include "LEDDistanceSynthesizer.h"

//============================================================
//ENUMS
//============================================================
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


//============================================================
//Defines
//============================================================
#define LED_PORT			(GPIO_PORTF_BASE)
#define LED_RED_PIN 		(GPIO_PIN_1)
#define LED_GREEN_PIN		(GPIO_PIN_3)
#define LED_BLUE_PIN		(GPIO_PIN_2)
extern void LED_ChangeColor(LED_COLOR color);


#endif /* LED_H_ */
