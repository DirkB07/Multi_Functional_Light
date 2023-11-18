/*
 * mode_mood.c
 *
 *  Created on: 18 Mar 2023
 *      Author: wiank
 */
#include "mode_mood.h"
#include "main.h"
#include "stm32f3xx_hal.h"
#include "common.h"


typedef struct STATE2_VARS_{

	uint8_t isActive;
	uint8_t rgbState;
	uint16_t redVal;
	uint16_t greenVal;
	uint16_t blueVal;
	uint16_t prevRedVal;
	uint16_t prevGreenVal;
	uint16_t prevBlueVal;



} STATE2_VARS;

STATE2_VARS state2_vars;

void STATE2_OnEnter(){


	//turn on gpio light 1 (pin PA6)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	state2_vars.isActive = 1;
	state2_vars.rgbState = 0;
	state2_vars.prevRedVal = 128;
	state2_vars.prevGreenVal = 128;
	state2_vars.prevBlueVal = 128;
	state2_vars.redVal = 0;
	state2_vars.greenVal = 0;
	state2_vars.blueVal = 0;

}

void STATE2_OnEnterUart(){


}

void STATE2_OnExit(){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	//turn on gpio light 1 (pin PA6)
	state2_vars.isActive = 0;
	state2_vars.redVal = 0;
	state2_vars.greenVal = 0;
	state2_vars.blueVal = 0;
	STATE2_ApplySettings();


}

void STATE2_OnButtonEvt(uint8_t button, uint8_t buttonState){

	if(button == BTN_MIDDLE && buttonState == BTN_STATE_PRESSED)
		{

			if(state2_vars.isActive == 1 && state2_vars.rgbState == 1){

				state2_vars.prevRedVal = state2_vars.redVal;
				state2_vars.prevGreenVal = state2_vars.greenVal;
				state2_vars.prevBlueVal = state2_vars.blueVal;

				state2_vars.blueVal = 0;
				state2_vars.greenVal = 0;
				state2_vars.redVal = 0;
				state2_vars.rgbState = 0;

			} else {
				state2_vars.redVal = state2_vars.prevRedVal;
				state2_vars.greenVal = state2_vars.prevGreenVal;
				state2_vars.blueVal = state2_vars.prevBlueVal;
				state2_vars.rgbState = 1;
			}

		}
	STATE2_ApplySettings();

}

void STATE2_OnInit(){
	state2_vars.isActive = 0;
	state2_vars.rgbState = 0;
	state2_vars.prevRedVal = 128;
	state2_vars.prevGreenVal = 128;
	state2_vars.prevBlueVal = 128;
	state2_vars.redVal = 0;
	state2_vars.greenVal = 0;
	state2_vars.blueVal = 0;

}

void STATE2_SetConfig(uint8_t *status){
	if(status[1] != ':' ||  status[4] != ':' || status[8] != ':' ||	status[12] != ':' || status[16] != ':' || status[17] != '$' || status[18] != '\n'){

			return;
		}





		uint16_t red = (status[5]-0x30)*100 + (status[6]-0x30)*10 + (status[7]-0x30);
		uint16_t green = (status[9]-0x30)*100 + (status[10]-0x30)*10 + (status[11]-0x30);
		uint16_t blue = (status[13]-0x30)*100 + (status[14]-0x30)*10 + (status[15]-0x30);

		//if for intensity range
		if(red < 513 && red >= 0 && green < 513 && green >= 0 && blue < 513 && blue >= 0)
		{
			state2_vars.redVal = red;
			state2_vars.greenVal = green;
			state2_vars.blueVal = blue;
			state2_vars.isActive = 1;
			state2_vars.rgbState = 1;
		}



//		if(state2_vars.isActive == 1){

			STATE2_ApplySettings();

//		}

}

void STATE2_GetConfig(uint8_t *config){
	config[0] = '#';
	config[1] = ':';
	config[2] = 'M';
	config[3] = 'M';
	config[4] = ':';
	config[5] = ((state2_vars.redVal)/100) + 0x30;
	config[6] = ((state2_vars.redVal)%100)/10 + 0x30;
	config[7] = ((state2_vars.redVal)%100)%10 + 0x30;;
	config[8] = ':';
	config[9] = ((state2_vars.greenVal)/100) + 0x30;
	config[10] = ((state2_vars.greenVal)%100)/10 + 0x30;
	config[11] = ((state2_vars.greenVal)%100)%10 + 0x30;
	config[12] = ':';
	config[13] = ((state2_vars.blueVal)/100) + 0x30;
	config[14] = ((state2_vars.blueVal)%100)/10 + 0x30;
	config[15] = ((state2_vars.blueVal)%100)%10 + 0x30;
	config[16] = ':';
	config[17] = '$';
	config[18] = '\n';

}

void STATE2_SliderVal(uint32_t sliderValue){



}

void STATE2_ApplySettings(){
	TIM15->CCR1 = state2_vars.greenVal;
	TIM2->CCR4 = state2_vars.blueVal;
	TIM16->CCR1 = state2_vars.redVal;
}

void STATE2_OnHold(){


				if(state2_vars.isActive == 1 && state2_vars.rgbState == 1){

					state2_vars.prevRedVal = state2_vars.redVal;
					state2_vars.prevGreenVal = state2_vars.greenVal;
					state2_vars.prevBlueVal = state2_vars.blueVal;

					state2_vars.blueVal = 0;
					state2_vars.greenVal = 0;
					state2_vars.redVal = 0;
					state2_vars.rgbState = 0;

				} else {
					state2_vars.redVal = state2_vars.prevRedVal;
					state2_vars.greenVal = state2_vars.prevGreenVal;
					state2_vars.blueVal = state2_vars.prevBlueVal;
					state2_vars.rgbState = 1;
				}
				STATE2_ApplySettings();
}

void STATE2_OnTap(uint16_t tapX, uint16_t tapY){

	uint16_t redXLimit = 598;
	uint16_t greenXLimit = 1196;
	uint16_t blueXLimit = 1792;



	state2_vars.isActive = 1;
	state2_vars.rgbState = 1;
	if(tapX < redXLimit){
	//red
		state2_vars.redVal = ((768 - tapY)*512)/768;
	}

	if(tapX >= redXLimit && tapX <= greenXLimit){

		state2_vars.greenVal = state2_vars.greenVal = ((768 - tapY)*512)/768;
	}

	if(tapX >= greenXLimit && tapX <= blueXLimit){

		state2_vars.blueVal = state2_vars.blueVal = ((768 - tapY)*512)/768;

	}
	STATE2_ApplySettings();


}

void STATE2_OnSlide(uint16_t currentXCoord){


}

void STATE2_SetIntensity(uint16_t newIntensity){


}

void STATE2_Housekeep(){



}

