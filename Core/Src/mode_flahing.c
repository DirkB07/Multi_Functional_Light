/*
 * mood_flahing.c
 *
 *  Created on: 18 Mar 2023
 *      Author: wiank
 */
#include "mode_flashing.h"
#include "main.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_dac.h"
#include "common.h"
#include "stm32f3xx_hal_tim.h"

extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim2;
typedef struct STATE1_VARS_{

	uint8_t isActive;
	uint32_t ledIntensity;
	uint8_t ledState;
	uint32_t storedLedIntensity;

} STATE1_VARS;

static STATE1_VARS state1_vars;

void STATE1_ApplySettings(){

	uint32_t val =   ((state1_vars.ledIntensity)*INPUT_MAX)/512;
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val);
	TIM2->CCR3 = (state1_vars.ledIntensity*1999)/512;
	TIM2->CCR2 = (state1_vars.ledIntensity*1999)/512;
}






void STATE1_OnEnter(){
	state1_vars.ledIntensity = 0;
	state1_vars.storedLedIntensity = 512;
	state1_vars.isActive = 1;
	//turn on gpio light 1 (pin PA6)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	STATE1_ApplySettings();



}


void STATE1_OnExit(){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	//turn on gpio light 1 (pin PA6)
	state1_vars.isActive = 0;
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

}



/* config[0] #,
 * config[1] :,
 * config[2-3] MF,
 * config[4] :,
 * config[5-7] intensity 1-512 with 000 led off,
 * config[8] :,
 * config[9-11] 000,
 * config[12] :,
 * config[13-15] 000,
 * config[16] :,
 * config[17-18] $\n,
*/


void STATE1_SetConfig(uint8_t *status){
	if(status[1] != ':' ||  status[4] != ':' || status[8] != ':' || status[9] != '0' || status[10] != '0' || status[11] != '0' ||
			status[12] != ':' || status[13] != '0' || status[14] != '0' || status[15] != '0' || status[16] != ':' || status[17] != '$' ||
			status[18] != '\n'){

		return;
	}





	uint16_t intensity = (status[5]-0x30)*100 + (status[6]-0x30)*10 + (status[7]-0x30);


	//if for intensity range
	if(intensity < 513 && intensity >= 0)
	{
	state1_vars.ledIntensity = intensity;
	}

	if(state1_vars.isActive == 1){

		STATE1_ApplySettings();

	}
}

void STATE1_GetConfig(uint8_t *config){
	config[0] = '#';
	config[1] = ':';
	config[2] = 'M';
	config[3] = 'F';
	config[4] = ':';
	config[5] = ((state1_vars.ledIntensity)/100) + 0x30;
	config[6] = (((state1_vars.ledIntensity) - (((state1_vars.ledIntensity)/100)*100)))/10 + 0x30;
	config[7] = ((state1_vars.ledIntensity) - (((state1_vars.ledIntensity)/100)*100)) - (((((state1_vars.ledIntensity)-(((state1_vars.ledIntensity)/100)*100))/10)*10)) + 0x30;
	config[8] = ':';
	config[9] = '0';
	config[10] = '0';
	config[11] = '0';
	config[12] = ':';
	config[13] = '0';
	config[14] = '0';
	config[15] = '0';
	config[16] = ':';
	config[17] = '$';
	config[18] = '\n';

}

void STATE1_OnInit(){

	state1_vars.isActive = 0;
	state1_vars.ledState = 0;
	state1_vars.ledIntensity = 0;



}


void STATE1_OnButtonEvt(uint8_t button, uint8_t buttonState){

	if(button == BTN_MIDDLE && buttonState == BTN_STATE_PRESSED)
	{
		if(state1_vars.ledIntensity>0){
			state1_vars.storedLedIntensity = state1_vars.ledIntensity;
			state1_vars.ledIntensity = 0;

		} else {
			state1_vars.ledIntensity = state1_vars.storedLedIntensity;
		}
		STATE1_ApplySettings();
	}


}

void STATE1_OnHold(){

	if(state1_vars.ledIntensity>0){
		state1_vars.storedLedIntensity = state1_vars.ledIntensity;
		state1_vars.ledIntensity = 0;
	} else {
		state1_vars.ledIntensity = state1_vars.storedLedIntensity;
	}
	STATE1_ApplySettings();

}

void STATE1_SliderVal(uint32_t sliderValue){

	state1_vars.ledIntensity = sliderValue;
	STATE1_ApplySettings();
}

void STATE1_Housekeep(){



}

void STATE1_OnTap(uint16_t tapX, uint16_t tapY){




}

void STATE1_OnSlide(uint16_t currentXCoord){

	STATE1_SetIntensity((currentXCoord*512)/1792);

}
void STATE1_OnEnterUart(){


}


void STATE1_SetIntensity(uint16_t newIntensity){

	state1_vars.ledIntensity = newIntensity;
	STATE1_ApplySettings();
}





