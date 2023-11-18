/*
 * mode_flashing.h
 *
 *  Created on: 18 Mar 2023
 *      Author: wiank
 */

#ifndef INC_MODE_FLASHING_H_
#define INC_MODE_FLASHING_H_

#include "stm32f3xx_hal.h"
void STATE1_OnEnter();
void STATE1_OnExit();
void STATE1_GetConfig(uint8_t *config);
void STATE1_OnInit();
void STATE1_OnButtonEvt(uint8_t button, uint8_t buttonState);
void STATE1_SetConfig(uint8_t *status);
void STATE1_SliderVal();
void STATE1_Housekeep();
void STATE1_SetIntensity(uint16_t newIntensity);
void STATE1_ApplySettings();
void STATE1_OnHold();
void STATE1_OnTap(uint16_t tapX, uint16_t tapY);
void STATE1_OnSlide(uint16_t currentXCoord);
void STATE1_OnEnterUart();
#endif /* INC_MODE_FLASHING_H_ */



