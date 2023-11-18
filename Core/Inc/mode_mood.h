/*
 * mode_flashing.h
 *
 *  Created on: 18 Mar 2023
 *      Author: wiank
 */

#ifndef INC_MODE_MOOD_H_
#define INC_MODE_MOOD_H_

#include "stm32f3xx_hal.h"

void STATE2_OnEnter();
void STATE2_OnExit();
void STATE2_OnButtonEvt(uint8_t button, uint8_t buttonState);
void STATE2_OnInit();
void STATE2_SetConfig(uint8_t *status);
void STATE2_GetConfig(uint8_t *config);
void STATE2_SliderVal(uint32_t sliderValue);
void STATE2_Housekeep();
void STATE2_SetIntensity(uint16_t newIntensity);
void STATE2_ApplySettings();
void STATE2_OnHold();
void STATE2_OnSlide(uint16_t currentXCoord);
void STATE2_OnTap(uint16_t tapX, uint16_t tapY);
void STATE2_OnEnterUart();
#endif /* INC_MODE_MOOD_H_ */

