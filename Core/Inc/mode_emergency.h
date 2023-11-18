/*
 * mode_flashing.h
 *
 *  Created on: 18 Mar 2023
 *      Author: wiank
 */

#ifndef INC_MODE_EMERGENCY_H_
#define INC_MODE_EMERGENCY_H_
#include "stm32f3xx_hal.h"

void STATE3_OnEnter();
void STATE3_OnExit();
void STATE3_OnButtonEvt(uint8_t button, uint8_t buttonState);
void STATE3_SliderVal();
void STATE3_OnInit();
void STATE3_SetConfig(uint8_t *status);
void STATE3_GetConfig(uint8_t *config);
void STATE3_Housekeep();
void STATE3_SetIntensity(uint16_t newIntensity);
void STATE3_ApplySettings();
void STATE3_OnHold();
void STATE3_OnSlide(uint16_t currentXCoord);
void STATE3_OnTap(uint16_t tapX, uint16_t tapY);
void STATE3_OnEnterUart();
#endif /* INC_MODE_EMERGENCY_H_ */


