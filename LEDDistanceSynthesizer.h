/*
 * LEDDistanceSynthesizer.h
 *
 *  Created on: Sep 25, 2014
 *      Author: DLThrasher
 */

#ifndef LEDDISTANCESYNTHESIZER_H_
#define LEDDISTANCESYNTHESIZER_H_


//*****************************************************************************
//		Includes
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
//		Global Enums
//*****************************************************************************
/*!
 * \brief lets make an led blink every 2 seconds
 */
typedef enum
{
	FALSE,
	TRUE
}BOOLEAN;

//*****************************************************************************
//		Global Defines
//*****************************************************************************
#define CLEAR	(0)
#define SET		(1)
#endif /* LEDDISTANCESYNTHESIZER_H_ */
