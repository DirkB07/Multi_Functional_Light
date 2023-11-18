/*
 * mode_emergency.c
 *
 *  Created on: 18 Mar 2023
 *      Author: wiank
 */
#include "mode_emergency.h"
#include "main.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_dac.h"
#include "common.h"
#include "stm32f3xx_hal_tim.h"
#include "morse_data.h"

#define CAPITAL_MASK			0b01011111
#define MORSE_3					3
#define MORSE_7					7

extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim2;



typedef enum EMERGENCY_MODES_ {

	STROBE,
	SOS,
	CUSTOM

} EMERGENCY_MODES;


typedef struct STATE3_VARS_ {

	uint8_t isActive;
	EMERGENCY_MODES emergencyMode;
	uint16_t ledIntensity;		//what is applied to pin
	uint16_t ledOnTime;
	uint16_t configuredIntensity; //intensity of led received from uart that doesnt vary with strobes;

	//uint32_t storedLedIntensity;	//intensity that toggle with middle button
	//uint32_t strobeLedIntensity;	//toggled with strobe
	uint32_t tickOffTime;
	uint32_t tickEndCycle;
	uint8_t ledStatus;				//used for button when switching led on and off
	uint8_t morseBinaryArray[100];
	uint8_t morseMaskArray[100];
	uint32_t morseTimer;
	uint8_t morseMessage[4];
	uint8_t morseCustomMessage[4];
	uint16_t morseIdx;
	uint8_t morseCurrentValue;
	uint8_t morseLedStatus;

} STATE3_VARS;

static STATE3_VARS state3_vars;

void updateMorseBinary(uint8_t *input);
void activateMorseMode(EMERGENCY_MODES myEmergencyMode);

void STATE3_SetIntensity(uint16_t newIntensity){

		state3_vars.ledIntensity = newIntensity;
		state3_vars.configuredIntensity = newIntensity;



	//STATE3_ApplySettings();
}

void STATE3_ApplySettings(){

	uint32_t val = ((state3_vars.ledIntensity)*INPUT_MAX)/512;
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val);
	TIM2->CCR3 = (state3_vars.ledIntensity*1999)/512;
	TIM2->CCR2 = (state3_vars.ledIntensity*1999)/512;

}

void STATE3_OnEnter(){

	state3_vars.isActive = 1;
	state3_vars.ledOnTime = 512;
	state3_vars.configuredIntensity = 512;
	state3_vars.ledIntensity = 0;
	state3_vars.ledStatus = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	state3_vars.emergencyMode = STROBE;
	state3_vars.morseCustomMessage[0] = 'S';
	state3_vars.morseCustomMessage[1] = 'O';
	state3_vars.morseCustomMessage[2] = 'S';

	STATE3_ApplySettings();

}

void STATE3_OnEnterUart(){
//
//	state3_vars.isActive = 1;
//	state3_vars.ledOnTime = 512;
//	state3_vars.storedLedIntensity = 512;
//	STATE3_SetIntensity(0);
//	state3_vars.ledStatus = 0;
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//	state3_vars.emergencyMode = STROBE;
//
//
//	STATE3_ApplySettings();
}


void STATE3_OnExit(){

	state3_vars.isActive = 0;
	state3_vars.ledStatus = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);


}

void STATE3_OnInit(){
	state3_vars.isActive = 0;
	state3_vars.emergencyMode = STROBE;
	state3_vars.ledIntensity = 0;
	state3_vars.configuredIntensity = 0;
	state3_vars.ledOnTime = 512;
	state3_vars.morseMessage[0] = 'S';
	state3_vars.morseMessage[1] = 'O';
	state3_vars.morseMessage[2] = 'S';
	state3_vars.ledStatus = 1;

}


void STATE3_OnButtonEvt(uint8_t button, uint8_t buttonState){

if(button == BTN_RIGHT && buttonState == BTN_STATE_PRESSED){

	if(state3_vars.emergencyMode == STROBE)
	{

		state3_vars.ledIntensity = 0;
		state3_vars.ledStatus = 0;
		state3_vars.morseMessage[0] = 'S';
		state3_vars.morseMessage[1] = 'O';
		state3_vars.morseMessage[2] = 'S';
		state3_vars.configuredIntensity = 512;
		state3_vars.emergencyMode = SOS;
		STATE3_ApplySettings();


	} else if(state3_vars.emergencyMode == SOS)
	{

		state3_vars.ledIntensity = 0;
		state3_vars.ledStatus = 0;
		state3_vars.configuredIntensity = 512;
		STATE3_ApplySettings();
		state3_vars.emergencyMode = CUSTOM;

	} else if(state3_vars.emergencyMode == CUSTOM){

		state3_vars.ledIntensity = 0;
		state3_vars.ledStatus = 0;
		state3_vars.configuredIntensity = 512;
		STATE3_ApplySettings();
		state3_vars.emergencyMode = STROBE;



	}
}

if(button == BTN_MIDDLE && buttonState == BTN_STATE_PRESSED)
	{

		if(state3_vars.emergencyMode == STROBE){
			if(state3_vars.ledStatus == 1){

						state3_vars.ledIntensity = 0;
						state3_vars.ledStatus = 0;
						STATE3_ApplySettings();
				} else {

					activateStrobeMode();
					state3_vars.ledStatus = 1;

				}


		}

		if(state3_vars.emergencyMode == SOS || state3_vars.emergencyMode == CUSTOM)
		{
			//if the led is currently not flashing, activate morse code mode
			if(state3_vars.ledStatus == 0){
				activateMorseMode(state3_vars.emergencyMode);
			} else
			{
				state3_vars.ledStatus = 0;
				state3_vars.ledIntensity = 0;
				STATE3_ApplySettings();

			}

		}


	}

}


void STATE3_SetConfig(uint8_t *status){
	if(status[1] != ':' ||  status[4] != ':' || status[8] != ':' ||	status[12] != ':' || status[16] != ':' || status[17] != '$' ||	status[18] != '\n'){

			return;
		}


		uint16_t ledIntensity = (status[5]-0x30)*100 + (status[6]-0x30)*10 + (status[7]-0x30);
		uint16_t ledOnTime = (status[9]-0x30)*100 + (status[10]-0x30)*10 + (status[11]-0x30);
		uint8_t morse1 = (status[13]);
		uint8_t morse2 = (status[14]);
		uint8_t morse3 = (status[15]);



		uint8_t customMode = 0;
		//if for intensity range
		if(ledIntensity < 513 && ledIntensity >= 0 && ledOnTime < 513 && ledOnTime >= 0)
		{
			state3_vars.configuredIntensity = ledIntensity;
			state3_vars.ledOnTime = ledOnTime;

			if(state3_vars.ledOnTime == 0){
				if (morse1 == '0' && morse2 == '0' && morse3 == '0'){
					state3_vars.morseMessage[0] = 'S';
					state3_vars.morseMessage[1] = 'O';
					state3_vars.morseMessage[2] = 'S';

				} else
				{
					state3_vars.morseCustomMessage[0] = morse1;
					state3_vars.morseCustomMessage[1] = morse2;
					state3_vars.morseCustomMessage[2] = morse3;
					customMode = 1;
				}
			}

		} else {

			return;
		}

		//strobe
		if(ledOnTime > 0){
			activateStrobeMode();
			if(state3_vars.configuredIntensity > 0){
				state3_vars.ledStatus = 1;
			} else {
				state3_vars.ledStatus = 0;
			}
			state3_vars.emergencyMode = STROBE;

		} else{

			if(customMode == 0){
				state3_vars.emergencyMode = SOS;

			} else {
				state3_vars.emergencyMode = CUSTOM;
			}
			activateMorseMode(state3_vars.emergencyMode);
			if(state3_vars.configuredIntensity > 0){
				state3_vars.ledStatus = 1;
			} else {
				state3_vars.ledStatus = 0;
			}
		}




}

void STATE3_GetConfig(uint8_t *config){

	config[0] = '#';
	config[1] = ':';
	config[2] = 'M';
	config[3] = 'E';
	config[4] = ':';
	if(state3_vars.ledStatus == 1){
		config[5] = ((state3_vars.configuredIntensity)/100) + 0x30;
		config[6] = ((state3_vars.configuredIntensity)%100)/10 + 0x30;
		config[7] = ((state3_vars.configuredIntensity)%100)%10 + 0x30;
	} else {
		config[5] = ((state3_vars.ledIntensity)/100) + 0x30;
		config[6] = ((state3_vars.ledIntensity)%100)/10 + 0x30;
		config[7] = ((state3_vars.ledIntensity)%100)%10 + 0x30;
	}

	config[8] = ':';
	config[9] = ((state3_vars.ledOnTime)/100) + 0x30;
	config[10] = ((state3_vars.ledOnTime)%100)/10 + 0x30;
	config[11] = ((state3_vars.ledOnTime)%100)%10 + 0x30;
	config[12] = ':';

	if(state3_vars.emergencyMode == CUSTOM || state3_vars.emergencyMode == STROBE){
		config[13] = state3_vars.morseCustomMessage[0];
		config[14] = state3_vars.morseCustomMessage[1];
		config[15] = state3_vars.morseCustomMessage[2];
	} else {
		config[13] = state3_vars.morseMessage[0];
		config[14] = state3_vars.morseMessage[1];
		config[15] = state3_vars.morseMessage[2];
	}
	config[16] = ':';
	config[17] = '$';
	config[18] = '\n';

}

void STATE3_SliderVal(uint32_t sliderValue){
	STATE3_SetIntensity(sliderValue);
//	state3_vars.ledIntensity = sliderValue;

}

void STATE3_OnHold(){

	if(state3_vars.emergencyMode == STROBE){
				if(state3_vars.ledStatus == 1){

							state3_vars.ledIntensity = 0;
							state3_vars.ledStatus = 0;
							STATE3_ApplySettings();
					} else {

						activateStrobeMode();
						state3_vars.ledStatus = 1;

					}


			}

			if(state3_vars.emergencyMode == SOS || state3_vars.emergencyMode == CUSTOM)
			{
				//if the led is currently not flashing, activate morse code mode
				if(state3_vars.ledStatus == 0){
					activateMorseMode(state3_vars.emergencyMode);
				} else
				{
					state3_vars.ledStatus = 0;
					state3_vars.ledIntensity = 0;
					STATE3_ApplySettings();

				}

			}


}

void STATE3_OnSlide(uint16_t currentXCoord){

	STATE3_SetIntensity((currentXCoord*512)/1792);
}

void STATE3_OnTap(uint16_t tapX, uint16_t tapY){
	if(state3_vars.emergencyMode == STROBE)
		{

			state3_vars.ledIntensity = 0;
			state3_vars.ledStatus = 0;
			state3_vars.morseMessage[0] = 'S';
			state3_vars.morseMessage[1] = 'O';
			state3_vars.morseMessage[2] = 'S';
			state3_vars.configuredIntensity = 512;
			state3_vars.emergencyMode = SOS;
			STATE3_ApplySettings();


		} else if(state3_vars.emergencyMode == SOS)
		{

			state3_vars.ledIntensity = 0;
			state3_vars.ledStatus = 0;
			state3_vars.configuredIntensity = 512;
			STATE3_ApplySettings();
			state3_vars.emergencyMode = CUSTOM;

		} else if(state3_vars.emergencyMode == CUSTOM){

			state3_vars.ledIntensity = 0;
			state3_vars.ledStatus = 0;
			state3_vars.configuredIntensity = 512;
			STATE3_ApplySettings();
			state3_vars.emergencyMode = STROBE;



		}
}


void STATE3_Housekeep(){

	if(state3_vars.ledStatus == 0){
		return;
	}
	if(state3_vars.isActive == 0){
		return;
	}


	//default 512 brightness & 512 on time

	//when entering 0 brightness 0 on time
	if(state3_vars.ledOnTime > 0 && state3_vars.emergencyMode == STROBE)
	{
		if(state3_vars.ledOnTime > 0)
		{
			if(HAL_GetTick() > state3_vars.tickOffTime && state3_vars.ledIntensity > 0)
			{
				//state3_vars.strobeLedIntensity = state3_vars.ledIntensity;
				state3_vars.ledIntensity = 0;
				STATE3_ApplySettings();
			}
			if(HAL_GetTick() > state3_vars.tickEndCycle)
			{
				state3_vars.ledIntensity = state3_vars.configuredIntensity;
				STATE3_ApplySettings();
				state3_vars.tickOffTime = HAL_GetTick() + state3_vars.ledOnTime;
				state3_vars.tickEndCycle = state3_vars.tickOffTime + state3_vars.ledOnTime;
			}
		}
	}

	if(state3_vars.emergencyMode == SOS || state3_vars.emergencyMode == CUSTOM){
		if(HAL_GetTick() > state3_vars.morseTimer + 512){
			state3_vars.morseIdx++;
			if(state3_vars.morseMaskArray[state3_vars.morseIdx] == 0){
				state3_vars.morseIdx = 0;
			}
			if(state3_vars.morseMaskArray[state3_vars.morseIdx] == 1)
			{
				uint8_t morsePrevValue = state3_vars.morseCurrentValue;
				state3_vars.morseCurrentValue = state3_vars.morseBinaryArray[state3_vars.morseIdx];
				if(state3_vars.morseCurrentValue != morsePrevValue)
				{

					//toggle led on
					if(state3_vars.morseCurrentValue == 1)
					{
						state3_vars.ledIntensity = state3_vars.configuredIntensity;
						state3_vars.morseLedStatus = 1;

					} else {
						state3_vars.configuredIntensity = state3_vars.ledIntensity;
						state3_vars.ledIntensity = 0;
						state3_vars.morseLedStatus = 0;
					}
					STATE3_ApplySettings();

				}
				state3_vars.morseTimer = HAL_GetTick();

			}


		}

	}

}

void activateMorseMode(EMERGENCY_MODES myEmergencyMode){


	if(myEmergencyMode == SOS){
		updateMorseBinary(state3_vars.morseMessage);
	} else {
		updateMorseBinary(state3_vars.morseCustomMessage);
	}
	state3_vars.morseIdx = 0;
	state3_vars.ledStatus = 1;
	// if there is a valid value to set
	if(state3_vars.morseMaskArray[state3_vars.morseIdx] == 1)
	{
		state3_vars.morseCurrentValue = state3_vars.morseBinaryArray[state3_vars.morseIdx];
		//toggle led on
		if(state3_vars.morseCurrentValue == 1)
		{
			state3_vars.ledIntensity = state3_vars.configuredIntensity;
			state3_vars.morseLedStatus = 1;


		} else {

			state3_vars.ledIntensity = 0;
			state3_vars.morseLedStatus = 0;
		}
		STATE3_ApplySettings();
		state3_vars.morseTimer = HAL_GetTick();

	}


}

void activateStrobeMode(){

	state3_vars.ledIntensity = state3_vars.configuredIntensity;
	state3_vars.tickOffTime = HAL_GetTick() + state3_vars.ledOnTime;
	state3_vars.tickEndCycle = state3_vars.ledOnTime + state3_vars.tickOffTime;
	STATE3_ApplySettings();


}






void updateMorseBinary(uint8_t *input) {
	/* reset the morse arrays */
	memset(state3_vars.morseBinaryArray, 0, sizeof(state3_vars.morseBinaryArray));
	memset(state3_vars.morseMaskArray, 0, sizeof(state3_vars.morseMaskArray));

	uint8_t shift = 0;

	/* iterate over input letters */
	for (int i = 0; i < 3; i++) {
		/*
		ensure input letter is uppercase, then get index of char in the asciiMorseBinary array by subtracting 32
		(since "SPACE" in ASCII is 32, and the first element of the asciiMorseBinary is "SPACE")
		*/
		uint8_t index = (input[i] &CAPITAL_MASK) - 32;

		/* Get the binary representation of the morse letter */
		uint32_t charBinary = asciiMorseBinary[index][0];

		/* Get the binary length of the morse letter */
		uint8_t charBinaryLength = asciiMorseBinary[index][1];

		/* Update the morse binary array with the right-most bit, then right-shift the charBinary value and repeat until the for loop finished */
		for (int j = 0; j < charBinaryLength; j++) {
			state3_vars.morseBinaryArray[shift + j] = charBinary &1;
			charBinary >>= 1;
		}

		/* Update the shift amount */
		shift += charBinaryLength;

		/* If the character is not the last in the string, then add the three time-unit space */
		if (i < 2) {
			shift += MORSE_3;
		}

		/* Otherwise make the space seven time-units */
		else {
			shift += MORSE_7;
		}
	}

	/* Create the binary mask */
	memset(state3_vars.morseMaskArray, 1, shift);
}






