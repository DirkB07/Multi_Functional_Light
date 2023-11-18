#include "main.h"
#include "stm32f3xx_it.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <morse.h>
#include <IQS7211AC.h>
#include <IQS7211A_init_AZP1189A3_v0.1.h>
#include <iqs7211a_addresses.h>

#define TRACKPADBUFFERSIZE 10000

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern RDY;
extern gesture;
extern hs;
extern tapRegistered;
extern tapX;
extern tapY;

uint16_t deviceAddress = 0x56;		//device Address
uint16_t xLsb = 0;
uint16_t xMsb = 0;
uint16_t yLsb = 0;
uint16_t yMsb = 0;
GESTURETYPE currentGesture = GEST_NONE;
uint16_t xBuffer[TRACKPADBUFFERSIZE] = {0};
uint8_t xBufEmpty = 1;
uint16_t xCoordinateLatest = 0;
uint16_t yCoordinateLatest = 0;
uint16_t xBufIdx = 0;
uint16_t xCoordProcessed = 0;
uint32_t timeFirstGesture = 0;


uint8_t getReadyPinState(void){

	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);

}

void hwReset(void){

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(500);
}

// writes value and reads into uint8_t array (little endian) so bytes has to be swopped if more than 1 byte is read
void writeRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[]){
	uint16_t data[2];
	data[0] = memoryAddress;

	while (RDY == 0){

	}
	if(RDY == 1){
		HAL_I2C_Mem_Write(&hi2c1, (deviceAddress<<1), data[0], I2C_MEMADD_SIZE_8BIT, bytesArray, (uint16_t)numBytes, 50);
		RDY = 0;
		//write to deviceAddress and bit-shift with 1 to show that it is a read or write operation

	} else {
		writeRandomBytes(memoryAddress, numBytes, bytesArray);

	}

}

// reads value and reads into uint8_t array (little endian) so bytes has to be swopped if more than 1 byte is read
void readRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[]){

	while(getReadyPinState() != 0)
	{

	}

	HAL_I2C_Mem_Read(&hi2c1, (deviceAddress<<1), memoryAddress, I2C_MEMADD_SIZE_8BIT, bytesArray, (uint16_t)numBytes, 50);

}

uint8_t readGestureData(uint8_t *gestureData){

	readRandomBytes(0x11, 10, gestureData);

}

/* process gesture data */
void processGestureData(uint8_t *gestureData){

	GESTURETYPE newGesture = GEST_NONE;

	if((gestureData[0] & 0x01) == 0x01){
		newGesture = GEST_TAP;

	} else if ((gestureData[0] & 0x02) == 0x02){

		newGesture = GEST_HOLD;

	}

	if (newGesture == GEST_NONE && currentGesture != GEST_NONE){

		xBufEmpty = 1;
		xCoordinateLatest = 0;
		yCoordinateLatest = 0;
		xBufIdx = 0;
		xCoordProcessed = 0;
		timeFirstGesture = 0;
		currentGesture = GEST_NONE;
		return;

	}


	xCoordinateLatest = gestureData[6];
	xCoordinateLatest |= (gestureData[7] << 8);
	yCoordinateLatest = gestureData[8];
	yCoordinateLatest |= (gestureData[9] << 8);

	if(newGesture == GEST_TAP){
		currentGesture = GEST_TAP;
		return;
	}

	if(newGesture == GEST_HOLD){
		if(currentGesture == GEST_NONE){
			timeFirstGesture = HAL_GetTick();
			currentGesture = GEST_DECIDING;
		}

		xBuffer[xBufIdx] = xCoordinateLatest;
		xBufIdx++;
		xCoordProcessed++;
		xBufEmpty = 0;
		if(xBufIdx == TRACKPADBUFFERSIZE){
			xBufIdx = 0;
		}

		if(currentGesture == GEST_DECIDING){
			if(xCoordProcessed > 5)
			{
				 if(abs(xBuffer[0] - xCoordinateLatest) > 30){
					 currentGesture = GEST_SLIDE;
					 return;
				 }
			}
			if(HAL_GetTick() > timeFirstGesture + 1500){
				currentGesture = GEST_HOLD;

			}

		}

	}


}

GESTURETYPE getGestureType(){
	GESTURETYPE gestureReturned = currentGesture;

	if(currentGesture == GEST_DECIDING){
		gestureReturned = GEST_NONE;
	}
	return gestureReturned;

}

void getGestureCoordinates(uint16_t *xCoordinate, uint16_t *yCoordinate){
	*xCoordinate = xCoordinateLatest;
	*yCoordinate = yCoordinateLatest;
	return;

}





void getProductNum(void){

	uint8_t transferBytes[2];	// A temporary array to hold the byte to be transferred.
	uint8_t prodNumLow = 0;         // Temporary storage for the counts low byte.
	uint8_t prodNumHigh = 0;        // Temporary storage for the counts high byte.
	uint16_t prodNumReturn = 0;     // The 16bit return value.

	readRandomBytes(IQS7211A_MM_PROD_NUM, 2, transferBytes);

	// Construct the 16bit return value.
	prodNumLow = transferBytes[0];
	prodNumHigh = transferBytes[1];
	prodNumReturn = (uint16_t)(prodNumLow);
	prodNumReturn |= (uint16_t)(prodNumHigh<<8);
	// Return the product number value.
	//char ProdNum[10];
	//uint16_t padNum = prodNumReturn;
	//sprintf(ProdNum, "%u\n", padNum);
	//HAL_UART_Transmit(&huart2, (uint8_t*)ProdNum, strlen(ProdNum), 50);
}

void setEventMode(){

}


uint8_t checkReset(void)
{
	uint8_t bytesArray[2];
	readRandomBytes(IQS7211A_MM_INFOFLAGS, 2, bytesArray);

	//little endian, lsb first so when not swopping the first bit is in second part
	if((bytesArray[0] & 0x80) == 0x80)
	{
		return 1;

	} else
	{
		return 0;
	}

}

void swReset(){

	uint8_t transferByte[2]; // Array to store the bytes transferred.
	  	  	  	  	  	  	   // Use an array to be consistent with other methods in this class.
	readRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 2, transferByte);
	  // Mask the settings with the SW_RESET_BIT.
	transferByte[1] |= 0x02;  // This is the bit required to perform SW Reset.
	  // Write the new byte to the required device.
	  writeRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 2, transferByte);

}

void acknowledgeReset()
{
	uint8_t transferBytes[2];	// A temporary array to hold the bytes to be transferred.
	// Read the System Flags from the IQS7211A, these must be read first in order not to change any settings.
	// We are interested in the 2nd byte at the address location, therefore, we must read and write both bytes.
	readRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 2, transferBytes);
	// SWrite the AAck Reset bit to 1 to clear the Show Reset Flag.
	transferBytes[0] |= 0x80;
	// Write the new byte to the System Flags address.
	writeRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 2, transferBytes);
}

uint8_t getSoftwareMajorNum()
{
	uint8_t transferBytes[1];	// A temporary array to hold the byte to be transferred.
								// Use an array to be consistent with other methods of the library.
	// Read the Device info from the IQS7211A.
	readRandomBytes(IQS7211A_MM_MAJOR_VERSION_NUM, 1, transferBytes);
  return transferBytes[0];
}

uint8_t getSoftwareMinorNum()
{
	uint8_t transferBytes[1];	// A temporary array to hold the byte to be transferred.
								// Use an array to be consistent with other methods of the library.
	// Read the Device info from the IQS7211A.
	readRandomBytes(IQS7211A_MM_MINOR_VERSION_NUM, 1, transferBytes);
	return transferBytes[0];
}

void TP_ReATI()
{
  uint8_t transferByte[1]; // Array to store the bytes transferred.
  	  	  	  	  	  	   // Use an array to be consistent with other methods in this class.
  readRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 1, transferByte);
  // Mask the settings with the REDO_ATI_BIT.
  transferByte[0] |= 0x20;  // This is the bit required to start an ATI routine.
  // Write the new byte to the required device.
  writeRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 1, transferByte);
}




void enableGestureEvent()
{
  uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.

  // First read the bytes at the memory address so that they can be preserved.
  readRandomBytes(IQS7211A_MM_CONFIG_SETTINGS, 2, transferBytes);
  // Set the GESTURE_EVENT_BIT in CONFIG_SETTINGS
  transferBytes[1] |= 0x02;
  // Write the bytes back to the device
  writeRandomBytes(IQS7211A_MM_CONFIG_SETTINGS, 2, transferBytes);
}

void enableGestures()
{
  uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.

  // First read the bytes at the memory address so that they can be preserved.
  readRandomBytes(IQS7211A_MM_GESTURE_ENABLE, 2, transferBytes);
  // Set the GESTURE_EVENT_BIT in CONFIG_SETTINGS
  transferBytes[1] |= 0x1F;
  // Write the bytes back to the device
  writeRandomBytes(IQS7211A_MM_GESTURE_ENABLE, 2, transferBytes);
}

void updateGestures()
{
  uint8_t transferBytes[1]; // The temporary address which will hold the GESTURE byte.

  // Read the byte using the readRandomByte method, only one byte is stored at the IQS7211A_MM_GESTURES address.
  readRandomBytes(IQS7211A_MM_GESTURES, 1, transferBytes);

  //HAL_UART_Transmit(&huart2, transferBytes, 1, 1000);

  //Process which gesture
  if (transferBytes[0] == 0x01) {
	  tapX = getAbsXCoordinate();
	  tapY = getAbsYCoordinate();
	  tapRegistered = 1;


	  //char message2[] = "TAP";
	  //HAL_UART_Transmit(&huart2, message2, strlen(message2), 1000);

  }


  if (transferBytes[0] == 0x02) {
	  //getAbsXCoordinate();

	  gesture = 1;

	  //char message3[] = "HOLD";
	  //HAL_UART_Transmit(&huart2, message3, strlen(message3), 1000);

  }

  if (transferBytes[0] != 0x02) {
  	  //getAbsXCoordinate();

  	  gesture = 0;



    }

}

void enableTPEvent()
{
  uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.

  // First read the bytes at the memory address so that they can be preserved.
  readRandomBytes(IQS7211A_MM_CONFIG_SETTINGS, 2, transferBytes);
  // Set the TP_EVENT_BIT in CONFIG_SETTINGS
  transferBytes[1] |= 0x04;
  // Write the bytes back to the device
  writeRandomBytes(IQS7211A_MM_CONFIG_SETTINGS, 2, transferBytes);
}


void updateAbsCoordinates()
{
  uint8_t transferBytes[4]; // The temporary address which will hold the bytes from the IQS7211A_MM_FINGER_1_X address.

    // Read the bytes using the readRandomBytes method to read bytes at the IQS7211A_MM_FINGER_1_X address.
    readRandomBytes(IQS7211A_MM_FINGER_1_X, 4, transferBytes);
    //  Assign the bytes to the union.
    xLsb = transferBytes[0];
    xMsb = transferBytes[1];
    yLsb = transferBytes[2];
    yMsb = transferBytes[3];

}

uint16_t getAbsYCoordinate()
{
  uint16_t absYCoordReturn = 0;     // The 16bit return value.

  // Construct the 16bit return value.

    absYCoordReturn = yLsb;
    absYCoordReturn |= (yMsb << 8);

//    char yCoordString[6]; // Buffer to hold the decimal string representation of the y coordinate.
//    sprintf(yCoordString, "%u\n", absYCoordReturn); // Convert the y coordinate to a decimal string.
//
//    // Transmit the decimal string representation of the y coordinate via UART.
//    HAL_UART_Transmit(&huart2, (uint8_t*)yCoordString, strlen(yCoordString), 1000);
    return absYCoordReturn;
}

uint16_t getAbsXCoordinate()
{
  uint16_t absXCoordReturn = 0;     // The 16bit return value.

    absXCoordReturn = xLsb;
    absXCoordReturn |= (xMsb << 8);

    // Convert the coordinate value to a decimal string.
    char xCoordString[6]; // Buffer to hold the decimal string representation of the x coordinate.
    //sprintf(xCoordString, " %u\n ", absXCoordReturn);

    // Transmit the decimal string via UART.
    //HAL_UART_Transmit(&huart2, (uint8_t*)xCoordString, strlen(xCoordString), 1000);


    return absXCoordReturn;
}

void writeMM()
{
	uint8_t transferBytes[30];	// Temporary array which holds the bytes to be transferred.

  /* Change the ATI Settings */
  /* Memory Map Position 0x30 - 0x3D */
  transferBytes[0] = TP_ATI_MULTIPLIERS_DIVIDERS_0;
  transferBytes[1] = TP_ATI_MULTIPLIERS_DIVIDERS_1;
  transferBytes[2] = TP_COMPENSATION_DIV_0;
  transferBytes[3] = TP_COMPENSATION_DIV_1;
  transferBytes[4] = TP_ATI_TARGET_0;
  transferBytes[5] = TP_ATI_TARGET_1;
  transferBytes[6] = TP_REF_DRIFT_LIMIT_0;
  transferBytes[7] = TP_REF_DRIFT_LIMIT_1;
  transferBytes[8] = TP_MIN_COUNT_REATI_0;
  transferBytes[9] = TP_MIN_COUNT_REATI_1;
  transferBytes[10] = REATI_RETRY_TIME_0;
  transferBytes[11] = REATI_RETRY_TIME_1;
  transferBytes[12] = ALP_ATI_MULTIPLIERS_DIVIDERS_0;
  transferBytes[13] = ALP_ATI_MULTIPLIERS_DIVIDERS_1;
  transferBytes[14] = ALP_COMPENSATION_DIV_0;
  transferBytes[15] = ALP_COMPENSATION_DIV_1;
  transferBytes[16] = ALP_ATI_TARGET_0;
  transferBytes[17] = ALP_ATI_TARGET_1;
  transferBytes[18] = ALP_LTA_DRIFT_LIMIT_0;
  transferBytes[19] = ALP_LTA_DRIFT_LIMIT_1;

  /* Change the ALP ATI Compensation */
  /* Memory Map Position 0x3A - 0x3D */
  transferBytes[20] = ALP_COMPENSATION_A_0;
  transferBytes[21] = ALP_COMPENSATION_A_1;
  transferBytes[22] = ALP_COMPENSATION_B_0;
  transferBytes[23] = ALP_COMPENSATION_B_1;
  writeRandomBytes(IQS7211A_MM_TP_ATI_MIR, 24, transferBytes);

  /* Change the Report Rates and Timing */
  /* Memory Map Position 0x40 - 0x4A */
  transferBytes[0] = ACTIVE_MODE_REPORT_RATE_0;
  transferBytes[1] = ACTIVE_MODE_REPORT_RATE_1;
  transferBytes[2] = IDLE_TOUCH_MODE_REPORT_RATE_0;
  transferBytes[3] = IDLE_TOUCH_MODE_REPORT_RATE_1;
  transferBytes[4] = IDLE_MODE_REPORT_RATE_0;
  transferBytes[5] = IDLE_MODE_REPORT_RATE_1;
  transferBytes[6] = LP1_MODE_REPORT_RATE_0;
  transferBytes[7] = LP1_MODE_REPORT_RATE_1;
  transferBytes[8] = LP2_MODE_REPORT_RATE_0;
  transferBytes[9] = LP2_MODE_REPORT_RATE_1;
  transferBytes[10] = ACTIVE_MODE_TIMEOUT_0;
  transferBytes[11] = ACTIVE_MODE_TIMEOUT_1;
  transferBytes[12] = IDLE_TOUCH_MODE_TIMEOUT_0;
  transferBytes[13] = IDLE_TOUCH_MODE_TIMEOUT_1;
  transferBytes[14] = IDLE_MODE_TIMEOUT_0;
  transferBytes[15] = IDLE_MODE_TIMEOUT_1;
  transferBytes[16] = LP1_MODE_TIMEOUT_0;
  transferBytes[17] = LP1_MODE_TIMEOUT_1;
  transferBytes[18] = REF_UPDATE_TIME_0;
  transferBytes[19] = REF_UPDATE_TIME_1;
  transferBytes[20] = I2C_TIMEOUT_0;
  transferBytes[21] = I2C_TIMEOUT_1;
  writeRandomBytes(IQS7211A_MM_ACTIVE_MODE_RR, 22, transferBytes);

  /* Change the System Settings */
  /* Memory Map Position 0x50 - 0x5B */
  transferBytes[0] = SYSTEM_CONTROL_0;
  transferBytes[1] = SYSTEM_CONTROL_1;
  transferBytes[2] = CONFIG_SETTINGS0;
  transferBytes[3] = CONFIG_SETTINGS1;
  transferBytes[4] = OTHER_SETTINGS_0;
  transferBytes[5] = OTHER_SETTINGS_1;
  transferBytes[6] = TRACKPAD_TOUCH_SET_THRESHOLD;
  transferBytes[7] = TRACKPAD_TOUCH_CLEAR_THRESHOLD;
  transferBytes[8] = ALP_THRESHOLD_0;
  transferBytes[9] = ALP_THRESHOLD_1;
  transferBytes[10] = OPEN_0_0;
  transferBytes[11] = OPEN_0_1;
  transferBytes[12] = ALP_SET_DEBOUNCE;
  transferBytes[13] = ALP_CLEAR_DEBOUNCE;
  transferBytes[14] = OPEN_1_0;
  transferBytes[15] = OPEN_1_1;
  transferBytes[16] = TP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
  transferBytes[17] = TP_CONVERSION_FREQUENCY_FRACTION_VALUE;
  transferBytes[18] = ALP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
  transferBytes[19] = ALP_CONVERSION_FREQUENCY_FRACTION_VALUE;
  transferBytes[20] = TRACKPAD_HARDWARE_SETTINGS_0;
  transferBytes[21] = TRACKPAD_HARDWARE_SETTINGS_1;
  transferBytes[22] = ALP_HARDWARE_SETTINGS_0;
  transferBytes[23] = ALP_HARDWARE_SETTINGS_1;
  writeRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 24, transferBytes);

  /* Change the Trackpad Settings */
  /* Memory Map Position 0x60 - 0x69 */
  transferBytes[0] = TRACKPAD_SETTINGS_0_0;
  transferBytes[1] = TRACKPAD_SETTINGS_0_1;
  transferBytes[2] = TRACKPAD_SETTINGS_1_0;
  transferBytes[3] = TRACKPAD_SETTINGS_1_1;
  transferBytes[4] = X_RESOLUTION_0;
  transferBytes[5] = X_RESOLUTION_1;
  transferBytes[6] = Y_RESOLUTION_0;
  transferBytes[7] = Y_RESOLUTION_1;
  transferBytes[8] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_0;
  transferBytes[9] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_1;
  transferBytes[10] = XY_DYNAMIC_FILTER_TOP_SPEED_0;
  transferBytes[11] = XY_DYNAMIC_FILTER_TOP_SPEED_1;
  transferBytes[12] = XY_DYNAMIC_FILTER_BOTTOM_BETA;
  transferBytes[13] = XY_DYNAMIC_FILTER_STATIC_FILTER_BETA;
  transferBytes[14] = STATIONARY_TOUCH_MOV_THRESHOLD;
  transferBytes[15] = FINGER_SPLIT_FACTOR;
  transferBytes[16] = X_TRIM_VALUE_0;
  transferBytes[17] = X_TRIM_VALUE_1;
  transferBytes[18] = Y_TRIM_VALUE_0;
  transferBytes[19] = Y_TRIM_VALUE_1;
  writeRandomBytes(IQS7211A_MM_TP_SETTINGS_0, 20, transferBytes);

  /* Change the ALP Settings */
  /* Memory Map Position 0x70 - 0x74 */
  transferBytes[0] = ALP_COUNT_FILTER_BETA_0;
  transferBytes[1] = OPEN_0;
  transferBytes[2] = ALP_LTA_BETA_LP1;
  transferBytes[3] = ALP_LTA_BETA_LP2;
  transferBytes[4] = ALP_SETUP_0;
  transferBytes[5] = ALP_SETUP_1;
  transferBytes[6] = ALP_TX_ENABLE_0;
  transferBytes[7] = ALP_TX_ENABLE_1;

  /* Change the Settings Version Numbers */
  /* Memory Map Position 0x74 - 0x75 */
  transferBytes[8] = MINOR_VERSION;
  transferBytes[9] = MAJOR_VERSION;
  writeRandomBytes(IQS7211A_MM_ALP_COUNT_FILTER_BETA, 10, transferBytes);

  /* Change the Gesture Settings */
  /* Memory Map Position 0x80 - 0x8F */
  transferBytes[0] = GESTURE_ENABLE_0;
  transferBytes[1] = GESTURE_ENABLE_1;
  transferBytes[2] = TAP_TIME_0;
  transferBytes[3] = TAP_TIME_1;
  transferBytes[4] = TAP_DISTANCE_0;
  transferBytes[5] = TAP_DISTANCE_1;
  transferBytes[6] = HOLD_TIME_0;
  transferBytes[7] = HOLD_TIME_1;
  transferBytes[8] = SWIPE_TIME_0;
  transferBytes[9] = SWIPE_TIME_1;
  transferBytes[10] = SWIPE_X_DISTANCE_0;
  transferBytes[11] = SWIPE_X_DISTANCE_1;
  transferBytes[12] = SWIPE_Y_DISTANCE_0;
  transferBytes[13] = SWIPE_Y_DISTANCE_1;
  transferBytes[14] = SWIPE_ANGLE_0;
  transferBytes[15] = GESTURE_OPEN_0;
  writeRandomBytes(IQS7211A_MM_GESTURE_ENABLE, 16, transferBytes);

  /* Change the RxTx Mapping */
  /* Memory Map Position 0x90 - 0x9C */
  transferBytes[0] = RX_TX_MAP_0;
  transferBytes[1] = RX_TX_MAP_1;
  transferBytes[2] = RX_TX_MAP_2;
  transferBytes[3] = RX_TX_MAP_3;
  transferBytes[4] = RX_TX_MAP_4;
  transferBytes[5] = RX_TX_MAP_5;
  transferBytes[6] = RX_TX_MAP_6;
  transferBytes[7] = RX_TX_MAP_7;
  transferBytes[8] = RX_TX_MAP_8;
  transferBytes[9] = RX_TX_MAP_9;
  transferBytes[10] = RX_TX_MAP_10;
  transferBytes[11] = RX_TX_MAP_11;
  transferBytes[12] = RX_TX_MAP_12;
  writeRandomBytes(IQS7211A_MM_RXTX_MAPPING_1_0, 13, transferBytes);

  /* Change the Allocation of channels into cycles 0-9 */
  /* Memory Map Position 0xA0 - 0xBD */
  transferBytes[0] = PLACEHOLDER_0;
  transferBytes[1] = CH_1_CYCLE_0;
  transferBytes[2] = CH_2_CYCLE_0;
  transferBytes[3] = PLACEHOLDER_1;
  transferBytes[4] = CH_1_CYCLE_1;
  transferBytes[5] = CH_2_CYCLE_1;
  transferBytes[6] = PLACEHOLDER_2;
  transferBytes[7] = CH_1_CYCLE_2;
  transferBytes[8] = CH_2_CYCLE_2;
  transferBytes[9] = PLACEHOLDER_3;
  transferBytes[10] = CH_1_CYCLE_3;
  transferBytes[11] = CH_2_CYCLE_3;
  transferBytes[12] = PLACEHOLDER_4;
  transferBytes[13] = CH_1_CYCLE_4;
  transferBytes[14] = CH_2_CYCLE_4;
  transferBytes[15] = PLACEHOLDER_5;
  transferBytes[16] = CH_1_CYCLE_5;
  transferBytes[17] = CH_2_CYCLE_5;
  transferBytes[18] = PLACEHOLDER_6;
  transferBytes[19] = CH_1_CYCLE_6;
  transferBytes[20] = CH_2_CYCLE_6;
  transferBytes[21] = PLACEHOLDER_7;
  transferBytes[22] = CH_1_CYCLE_7;
  transferBytes[23] = CH_2_CYCLE_7;
  transferBytes[24] = PLACEHOLDER_8;
  transferBytes[25] = CH_1_CYCLE_8;
  transferBytes[26] = CH_2_CYCLE_8;
  transferBytes[27] = PLACEHOLDER_9;
  transferBytes[28] = CH_1_CYCLE_9;
  transferBytes[29] = CH_2_CYCLE_9;
  writeRandomBytes(IQS7211A_MM_CYCLE_SETUP_0_9, 30, transferBytes);

  /* Change the Allocation of channels into cycles 10-17 */
  /* Memory Map Position 0xB0 - 0xCA */
  transferBytes[0] = PLACEHOLDER_10;
  transferBytes[1] = CH_1_CYCLE_10;
  transferBytes[2] = CH_2_CYCLE_10;
  transferBytes[3] = PLACEHOLDER_11;
  transferBytes[4] = CH_1_CYCLE_11;
  transferBytes[5] = CH_2_CYCLE_11;
  transferBytes[6] = PLACEHOLDER_12;
  transferBytes[7] = CH_1_CYCLE_12;
  transferBytes[8] = CH_2_CYCLE_12;
  transferBytes[9] = PLACEHOLDER_13;
  transferBytes[10] = CH_1_CYCLE_13;
  transferBytes[11] = CH_2_CYCLE_13;
  transferBytes[12] = PLACEHOLDER_14;
  transferBytes[13] = CH_1_CYCLE_14;
  transferBytes[14] = CH_2_CYCLE_14;
  transferBytes[15] = PLACEHOLDER_15;
  transferBytes[16] = CH_1_CYCLE_15;
  transferBytes[17] = CH_2_CYCLE_15;
  transferBytes[18] = PLACEHOLDER_16;
  transferBytes[19] = CH_1_CYCLE_16;
  transferBytes[20] = CH_2_CYCLE_16;
  transferBytes[21] = PLACEHOLDER_17;
  transferBytes[22] = CH_1_CYCLE_17;
  transferBytes[23] = CH_2_CYCLE_17;
  writeRandomBytes(IQS7211A_MM_CYCLE_SETUP_10_17, 24, transferBytes);

}
