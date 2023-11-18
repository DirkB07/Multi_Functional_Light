/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


/**
 * THIS CODE UP TO DEMO 3 WAS TAKEN FROM WIAN VAN WYK
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "mode_flashing.h"
#include "mode_mood.h"
#include "mode_emergency.h"
#include "IQS7211AC.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define RX_BUF_SIZE 30
#define ADC_SAMPLES 12
#define ADC_THRESHOLD 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct buttons
	{
	uint8_t button_state_change;
	uint32_t press_tick;
	uint8_t new_button_state;
	uint8_t button_state;
	uint16_t gpio;
	GPIO_TypeDef* group;
	};


typedef enum States
  {
  	STATE1,	//mf
  	STATE2,	//mm
  	STATE3,	//me
  	STATE_MAX

  } STATE;



typedef struct StateFuncs_
    {
     void(*handleOnEnterEvt)();
     void(*handleOnExitEvt)();
     void(*handleOnButtonEvt)(uint8_t button, uint8_t buttonState);
     void(*handleOnInit)();
     void(*handleSetConfig)(uint8_t *config);
     void(*handleGetConfig)(uint8_t *status);
     void(*handleSliderVal)(uint32_t value);
     void(*handleHousekeep)();
     void(*handleApplySetting)();
     void(*handleSetIntensity)();
     void(*handleOnHold)();
     void(*handleOnEnterUart)();
     void(*handleOnSlide)(uint16_t currentXCoord);
     void(*handleOnTap)(uint16_t tapX, uint16_t tapY);


    } StateFuncs;


static StateFuncs stateCallback[STATE_MAX] =
     {
          {
          .handleOnEnterEvt = &STATE1_OnEnter,
          .handleOnExitEvt = &STATE1_OnExit,
		  .handleSetConfig = &STATE1_SetConfig,
		  .handleGetConfig = &STATE1_GetConfig,
		  .handleOnInit = &STATE1_OnInit,
		  .handleOnButtonEvt = &STATE1_OnButtonEvt,
		  .handleSliderVal = &STATE1_SliderVal,
		  .handleHousekeep = &STATE1_Housekeep,
		  .handleApplySetting = &STATE1_ApplySettings,
		  .handleSetIntensity = &STATE1_SetIntensity,
		  .handleOnHold = &STATE1_OnHold,
		  .handleOnTap = &STATE1_OnTap,
		  .handleOnSlide = &STATE1_OnSlide,
		  .handleOnEnterUart = &STATE1_OnEnterUart,
          },
		  {
		  .handleOnEnterEvt = &STATE2_OnEnter,
		  .handleOnExitEvt = &STATE2_OnExit,
		  .handleSetConfig = &STATE2_SetConfig,
		  .handleGetConfig = &STATE2_GetConfig,
		  .handleOnInit = &STATE2_OnInit,
		  .handleOnButtonEvt = &STATE2_OnButtonEvt,
	  	  .handleSliderVal = &STATE2_SliderVal,
		  .handleHousekeep = &STATE2_Housekeep,
		  .handleApplySetting = &STATE2_ApplySettings,
		  .handleSetIntensity = &STATE2_SetIntensity,
		  .handleOnHold = &STATE2_OnHold,
		  .handleOnTap = &STATE2_OnTap,
		  .handleOnSlide = &STATE2_OnSlide,
		  .handleOnEnterUart = &STATE2_OnEnterUart,

		  },
		  {
		  .handleOnEnterEvt = &STATE3_OnEnter,
		  .handleOnExitEvt = &STATE3_OnExit,
		  .handleSetConfig = &STATE3_SetConfig,
		  .handleGetConfig = &STATE3_GetConfig,
		  .handleOnInit = &STATE3_OnInit,
		  .handleOnButtonEvt = &STATE3_OnButtonEvt,
	  	  .handleSliderVal = &STATE3_SliderVal,
		  .handleHousekeep = &STATE3_Housekeep,
		  .handleApplySetting = &STATE3_ApplySettings,
		  .handleSetIntensity = &STATE3_SetIntensity,
		  .handleOnHold = &STATE3_OnHold,
		  .handleOnTap = &STATE3_OnTap,
		  .handleOnSlide = &STATE3_OnSlide,
		  .handleOnEnterUart = &STATE3_OnEnterUart,

		  }
         };


struct buttons LEFT, MIDDLE, RIGHT;

struct buttons btns[NUM_BTN];

uint8_t rxBuffer[RX_BUF_SIZE] = {0};
uint8_t rxIndex = 0;
uint8_t recievedUartMessage = 0;	//if the uart rx buffer has a valid message
uint8_t rxMessageLength = 0;

uint8_t txBuffer[RX_BUF_SIZE] = {0};

// slider var
uint32_t adcSum;
uint32_t adcNumSamples;
uint32_t adcPreviousAvg;
uint32_t adcAvg;

// rdy flag
volatile uint8_t RDY = 1;

volatile uint8_t hs = 0;
uint16_t prevX = 0;
uint16_t newX = 0;

uint16_t xCounter = 0;
uint32_t nowTick = 0;				//added volatile
volatile uint16_t trackpadIntensity = 0;
volatile uint32_t myStartTick = 0;
uint16_t holdDetected = 0;			//added volatile
volatile uint16_t slideDetected = 0;			//added variable

volatile uint16_t tapX = 0;
volatile uint16_t tapY = 0;
volatile uint8_t tapRegistered = 0;
uint32_t tickAtStart = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  tickAtStart = HAL_GetTick();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  uint8_t studentNumber[] = "#:25137018:$\n";
  uint8_t message[] = "button pressed \n";
  uint8_t stdMessageSize[] = "#:MM:512:000:000:$\n"; //19

//first adc sample set
  uint8_t adcStart = 1;

  //I2C buffers

  //ADD WAIT FOR UART TRANSMIT
  while(HAL_GetTick() - tickAtStart < 100){

  }
  uint32_t tickNow = HAL_GetTick();
  HAL_UART_Transmit(&huart2, studentNumber, 13, 500);


  btns[BTN_LEFT].button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
  btns[BTN_LEFT].new_button_state = btns[BTN_LEFT].button_state;
  btns[BTN_LEFT].gpio = GPIO_PIN_9;
  btns[BTN_LEFT].group = GPIOB;
  btns[BTN_LEFT].press_tick = 0;

  btns[BTN_RIGHT].button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
  btns[BTN_RIGHT].new_button_state = btns[BTN_RIGHT].button_state;
  btns[BTN_RIGHT].gpio = GPIO_PIN_7;
  btns[BTN_RIGHT].group = GPIOC;
  btns[BTN_RIGHT].press_tick = 0;


  btns[BTN_MIDDLE].button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
  btns[BTN_MIDDLE].new_button_state = btns[BTN_MIDDLE].button_state;
  btns[BTN_MIDDLE].gpio = GPIO_PIN_8;
  btns[BTN_MIDDLE].group = GPIOA;
  btns[BTN_MIDDLE].press_tick = 0;


  //ADD STATE INIT FOR ALL STATES

  STATE currentState = STATE1;
  STATE newState = STATE_MAX;
  STATE targetedState = STATE_MAX;
  int s = 0;
  for(s = 0; s < STATE_MAX; s++)
  	  {
	  stateCallback[s].handleOnInit();
  	  }

  stateCallback[STATE1].handleOnEnterEvt();

  //Code for byte recieve
  HAL_UART_Receive_IT(&huart2, rxBuffer, 1);

  //adc calibration
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

  initTrackpad();


  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);



  char message2[] = "SLIDE\n";
  char messageIntSetTo[] = "INTENSITYSET";
  char message3[] = "";
  uint8_t gestureData[10];
  slideDetected = 0;

  GESTURETYPE currentGesture = GEST_NONE;
  GESTURETYPE readGesture = GEST_NONE;
  uint16_t currentXCoord = 0;
  uint16_t currentYCoord = 0;
  uint32_t trackInitTime = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (RDY == 1)
	  {

		  readGestureData(gestureData);
		  if (HAL_GetTick()- tickAtStart > 5000)
		  {

			  processGestureData(gestureData);
			  readGesture = getGestureType();
			  getGestureCoordinates(&currentXCoord, &currentYCoord);
			  if(readGesture != currentGesture){
				  currentGesture = readGesture;
				  if(currentGesture == GEST_TAP)
				  {
					  stateCallback[currentState].handleOnTap(currentXCoord, currentYCoord);
				  } else if (currentGesture == GEST_HOLD)
				  {
					  stateCallback[currentState].handleOnHold();

				  }
			  }
			  if (currentGesture == GEST_SLIDE){

					 stateCallback[currentState].handleOnSlide(currentXCoord);

				  }
		  }
	  	  RDY = 0;

	  	}

	  //adc
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1);
	  uint32_t adcVal = HAL_ADC_GetValue(&hadc1);
	  // if sample >= 12 && average changed with certain amount then read slider value to current status handle slider

	  //scaling
	  adcVal = (adcVal/INPUT_MAX)*OUTPUT_MAX;

	  adcSum = adcSum +adcVal;
	  adcNumSamples++;

	  if(adcNumSamples == ADC_SAMPLES){
		  adcAvg = adcSum/ADC_SAMPLES;
		  if(adcAvg > 480){
			  adcAvg = 512;
		  }
		  if(adcAvg <= 18)
		  {
			  adcAvg = 1;
		  }
		  adcNumSamples = 0;
		  if((adcAvg > adcPreviousAvg && adcAvg - adcPreviousAvg > ADC_THRESHOLD) || (adcAvg < adcPreviousAvg && adcPreviousAvg - adcAvg > ADC_THRESHOLD))
		  {
			  if(adcStart != 1)
			  {

			  stateCallback[currentState].handleSliderVal(adcAvg);

			  if(currentState == STATE1 || currentState == STATE3)
			  {
				  stateCallback[currentState].handleGetConfig(txBuffer);
				  //HAL_UART_Transmit(&huart2, txBuffer, 19, 500);
			  }

			  }
			  adcPreviousAvg = adcAvg;
			  adcStart = 0;

		  }
		  adcSum = 0;
	  }

	  //Button code
	  	  int j;
	  	  for (j = 0; j < NUM_BTN; j++) {
	  		if(HAL_GetTick() - btns[j].press_tick > 50 && btns[j].button_state_change == 1)
	  			  	{
	  					btns[j].new_button_state = HAL_GPIO_ReadPin(btns[j].group, btns[j].gpio);
	  					btns[j].button_state_change = 0;									//button isn't changing anymore
	  			  	}

	  		if(btns[j].new_button_state != btns[j].button_state){
	  			  if(btns[j].new_button_state == BTN_STATE_PRESSED)
	  			  {
	  				  if(btns[BTN_LEFT].new_button_state == BTN_STATE_PRESSED)
	  				  {
	  					stateCallback[currentState].handleOnExitEvt();
	  					  //HAL_UART_Transmit(&huart2, message, 16, 1000);
	  					  	  if(currentState == STATE1)
	  					  		{

	  					  			newState = STATE3;

	  					  		}
	  					  		if(currentState == STATE2)
	  					  		{

	  					  			newState = STATE1;

	  					  		}
	  					  		if(currentState == STATE3)
	  					  		{

	  					  			newState = STATE2;

	  					  		}
	  					  		currentState = newState;
	  					  		stateCallback[currentState].handleOnEnterEvt();

	  				  }
	  				  else {
	  					stateCallback[currentState].handleOnButtonEvt(j, BTN_STATE_PRESSED);

	  				  }

	  			  }
	  			  btns[j].button_state = btns[j].new_button_state;
	  			  }
	  	  	 }

	  	  //test

	  	  if(recievedUartMessage == 1){
	  		targetedState = STATE_MAX;

	  		  if(rxMessageLength == 19 || rxMessageLength == 7){
	  			  // Flasing state
	  			  if(rxBuffer[2] == 'M' && rxBuffer[3] == 'F'){
	  				  targetedState = STATE1;

	  			}
	  			  else if(rxBuffer[2] == 'M' && rxBuffer[3] == 'M'){
	  				targetedState = STATE2;

	  			}
	  			else if(rxBuffer[2] == 'M' && rxBuffer[3] == 'E'){
	  				targetedState = STATE3;

	  			}


	  			  //if there was a valid state entered
	  			if(targetedState != STATE_MAX)
	  			{
	  				if(rxMessageLength == 19)
	  				{


	  					if(targetedState != currentState)
	  					{
	  						stateCallback[currentState].handleOnExitEvt();
	  						currentState = targetedState;
	  						stateCallback[currentState].handleOnEnterEvt();
	  					}

						stateCallback[targetedState].handleSetConfig(rxBuffer);
	  					stateCallback[currentState].handleGetConfig(txBuffer);
	  					HAL_UART_Transmit(&huart2, txBuffer, 19, 500);
	  				}
	  				else if(rxMessageLength == 7)
	  				{
	  					stateCallback[targetedState].handleGetConfig(txBuffer);
	  					HAL_UART_Transmit(&huart2, txBuffer, 19, 500);
	  				}

	  			}

	  		  }

	  		recievedUartMessage = 0;
	  		rxMessageLength = 0;
	  	  }

	  	  stateCallback[currentState].handleHousekeep();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM15
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x200009FE;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 71;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 71;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RED_LED_2_Pin|RED_LED_1_Pin|RED_LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCLR_GPIO_Port, MCLR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_LED_2_Pin RED_LED_1_Pin RED_LED_3_Pin */
  GPIO_InitStruct.Pin = RED_LED_2_Pin|RED_LED_1_Pin|RED_LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RDY_Pin */
  GPIO_InitStruct.Pin = RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCLR_Pin */
  GPIO_InitStruct.Pin = MCLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCLR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Right_Button_Pin */
  GPIO_InitStruct.Pin = Right_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Right_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Middle_Button_Pin */
  GPIO_InitStruct.Pin = Middle_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Middle_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Left_Button_Pin */
  GPIO_InitStruct.Pin = Left_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Left_Button_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int i;

	for (i = 0; i < NUM_BTN; i++)
	{
		if(btns[i].gpio == GPIO_Pin)
		{
			if(btns[i].button_state_change == 0)
				{
					btns[i].press_tick = HAL_GetTick();
					btns[i].button_state_change = 1;
				}

		}
	}

	if(GPIO_Pin == GPIO_PIN_15){

			RDY = 1;

	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // if incorrect first byte

  if(rxIndex == 0 && rxBuffer[0] != '#')
  {
	  HAL_UART_Receive_IT(&huart2, rxBuffer, 1);
	  return;
  }

  // if end is reached

  if(rxBuffer[rxIndex] == '\n')
  {
	  HAL_UART_Receive_IT(&huart2, rxBuffer, 1);

	  if(rxIndex>0)
	  {
	  rxMessageLength = rxIndex + 1;
	  rxIndex = 0;
	  recievedUartMessage = 1;
	  }

	  return;

  }
  // if reading middle bytes in
  rxIndex++;
  if(rxIndex >= RX_BUF_SIZE)
  {
	rxIndex = 0;
  }

  HAL_UART_Receive_IT(&huart2, &rxBuffer[rxIndex], 1);

}


void initTrackpad(){

	hwReset();
	//getProductNum();
	while(RDY == 0){

	}
	//Reading of info flags
	 while(checkReset() == 0){
		 //soft reset
		 swReset();
	 }

	  acknowledgeReset();
	  HAL_Delay(100);
	  getProductNum();
	  HAL_Delay(100);
	  getSoftwareMajorNum();
	  HAL_Delay(100);
	  getSoftwareMinorNum();
	  HAL_Delay(100);
	  writeMM();
	  HAL_Delay(100);
	  enableTPEvent();
	  HAL_Delay(100);
	  TP_ReATI();
	  HAL_Delay(100);
	  enableGestureEvent();
	  HAL_Delay(100);
	  enableGestures();
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
