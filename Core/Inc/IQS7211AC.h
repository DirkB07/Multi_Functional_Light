/*
 * IQS7211AC.h
 *
 *  Created on: Apr 27, 2023
 *      Author: wiank
 */

#ifndef INC_IQS7211AC_H_
#define INC_IQS7211AC_H_

#include "stm32f3xx_hal.h"

typedef enum GestureType
	{
		GEST_NONE,
		GEST_TAP,
		GEST_DECIDING,
		GEST_HOLD,
		GEST_SLIDE

	}	GESTURETYPE;


void hwReset(void);
void writeRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[]);
void readRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[]);
void getProductNum(void);
uint8_t checkReset(void);
void acknowledgeReset();
void setEventMode();
void enableGestureEvent();
void enableGestures();
void updateGestures();
void enableTPEvent();
void updateAbsCoordinates();
uint16_t getAbsYCoordinate();
uint16_t getAbsXCoordinate();
void writeMM();
GESTURETYPE getGestureType();
void getGestureCoordinates(uint16_t *xCoordinate, uint16_t *yCoordinate);
void processGestureData(uint8_t *gestureData);


#endif /* INC_IQS7211AC_H_ */
