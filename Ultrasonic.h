/*
 * Ultrasonic.h
 *
 *  Created on: Sep 25, 2014
 *      Author: DLThrasher
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_
//*****************************************************************************
//		Includes
//*****************************************************************************
#include "LEDDistanceSynthesizer.h"
//============================================================
//Defines
//============================================================
#define ULTRASONIC_SENSOR_PORT	(GPIO_PORTD_BASE)
#define ECHO_PIN				(GPIO_PIN_3)
#define TRIGGER_PIN				(GPIO_PIN_6)

//============================================================
//Global Functions
//============================================================
extern int Ultrasonic_GetDistanceCM(void);
extern void Ultrasonic_SetDistanceCM(int);
extern void Ultrasonic__SetTrigger(void);
extern void Ultrasonic_SetWaitingForEcho(BOOLEAN);

#endif /* ULTRASONIC_H_ */
