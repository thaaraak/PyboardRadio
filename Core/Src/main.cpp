/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "coeffs.h"
#include <arm_math.h>
#include <string.h>
#include <stdio.h>

#include "si5351.h"
#include "OLED_Driver.h"
#include "OLED_GFX.h"
#include "FrequencyDisplay.h"
#include "FrequencyDisplayLCD.h"
#include "LiquidCrystal_I2C.h"
#include "Encoder.h"
#include "Bounce2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_i2s2_ext_tx;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

Si5351 synth;

#define SAMPLES			2048				// Total number of samples left and right
#define	BUF_SAMPLES		SAMPLES * 4		// Size of DMA tx/rx buffer samples * left/right * 2 for 32 bit samples

// DMA Buffers
uint16_t rxBuf[BUF_SAMPLES];
uint16_t txBuf[BUF_SAMPLES];

volatile int halfComplete = 0;
volatile int fullComplete = 0;

// Processing Buffers

float32_t	*coeffsLeft;
float32_t	*coeffsRight;
float32_t	stateLeft[SAMPLES/2 + NUM_TAPS - 1];
float32_t	stateRight[SAMPLES/2 + NUM_TAPS - 1];

float32_t	srcLeft[SAMPLES/2];
float32_t	srcRight[SAMPLES/2];
float32_t	destLeft[SAMPLES/2];
float32_t	destRight[SAMPLES/2];

arm_status stat;
arm_fir_instance_f32 arm_inst_left;
arm_fir_instance_f32 arm_inst_right;

OLED_GFX oled = OLED_GFX();
Encoder encoder( GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_14 );
FrequencyDisplayLCD *display;

Bounce	ssbButton;
Bounce	radixButton;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */

void doPassthru(int b);
void doFIR(int b);
void changeFreqency( int f );
void changeSideband();
void displaySSB();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MODE_USB	0
#define MODE_LSB	1

int lastMult = 0;
int ssbMode = MODE_LSB;


void changeFrequency( int currentFrequency )
{
	  int mult = 0;

	  if ( currentFrequency < 8000000 )
		  mult = 100;
	  else if ( currentFrequency < 11000000 )
		  mult = 80;
	  else if ( currentFrequency < 15000000 )
		  mult = 50;

	  uint64_t freq = currentFrequency * 100ULL;
	  uint64_t pllFreq = freq * mult;

	  synth.set_freq_manual(freq, pllFreq, SI5351_CLK0);
	  synth.set_freq_manual(freq, pllFreq, SI5351_CLK2);

	  if ( mult != lastMult )
	  {
		  synth.set_phase(SI5351_CLK0, 0);
		  synth.set_phase(SI5351_CLK2, mult);
		  synth.pll_reset(SI5351_PLLA);
		  lastMult = mult;
	  }
}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */

  coeffsLeft = minus45Coeffs;
  coeffsRight = plus45Coeffs;

  arm_fir_init_f32(
		  &arm_inst_left,
		  NUM_TAPS,
		  coeffsLeft,
		  &stateLeft[0],
		  SAMPLES/2
  );

  arm_fir_init_f32(
		  &arm_inst_right,
		  NUM_TAPS,
		  coeffsRight,
		  &stateRight[0],
		  SAMPLES/2
  );

  synth.init( &hi2c1, SI5351_CRYSTAL_LOAD_8PF, 25000000, 0 );
  changeFrequency( 7200000 );
  HAL_I2SEx_TransmitReceive_DMA(&hi2s2, txBuf, rxBuf, SAMPLES*2 );

  LiquidCrystal_I2C lcd(&hi2c1);
  lcd.init();
  lcd.backlight();

  display = new FrequencyDisplayLCD( &lcd, &encoder, 7200000 );
  displaySSB();

  ssbButton.attach( GPIOB, GPIO_PIN_0 );
  ssbButton.interval(20);

  radixButton.attach( GPIOB, GPIO_PIN_15 );
  radixButton.interval(20);

  int lastFrequency = 0;
  unsigned int lastSSBRead = 0;
  unsigned int lastModeRead = 0;
  int value = 0;

  while (1)
  {
	  if ( halfComplete )
	  {
		  doFIR(0);
		  halfComplete = 0;
	  }

	  else if ( fullComplete )
	  {
		  doFIR(1);
		  fullComplete = 0;
	  }

	  ssbButton.update();
	  value = ssbButton.read();

	  if ( value != lastSSBRead )
	  {
	    lastSSBRead = value;

	    if ( value == 0 )
	      changeSideband();
	  }

	  radixButton.update();
	  value = radixButton.read();

	  if ( value != lastModeRead )
	  {
	    lastModeRead = value;

	    if ( value == 0 )
	      display->changeMode();
	  }

	  if ( display->getFrequency() != lastFrequency )
	  {
		  lastFrequency = display->getFrequency();
		  changeFrequency( lastFrequency );
	  }

	  if ( encoder.hasChanged() )
	  {
		  display->change();
		  encoder.reset();
	  }

	  HAL_GPIO_TogglePin( GPIOC, GPIO_PIN_5 );

  }


}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 164;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 172;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}


static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 99;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}


/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OLED_RST_Pin|OLED_DC_Pin|OLED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_RST_Pin OLED_DC_Pin OLED_CS_Pin */
  GPIO_InitStruct.Pin = OLED_RST_Pin|OLED_DC_Pin|OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SSB_SW_Pin ENC_SW_Pin */
  GPIO_InitStruct.Pin = SSB_SW_Pin|ENC_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_CLK_Pin ENC_DT_Pin */
  GPIO_InitStruct.Pin = ENC_CLK_Pin|ENC_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}


/* USER CODE BEGIN 4 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  UNUSED(htim);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  UNUSED(GPIO_Pin);

  if ( GPIO_Pin == ENC_CLK_Pin )
	  encoder.clkInterrupt();
  else if ( GPIO_Pin == ENC_DT_Pin )
	  encoder.dtInterrupt();

}


void doPassthru( int b )
{

//	  	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_13, GPIO_PIN_SET );


		int startBuf = b * BUF_SAMPLES / 2;
		int endBuf = startBuf + BUF_SAMPLES / 2;

		int i = 0;
		for ( int pos = startBuf ; pos < endBuf ; pos+=4 )
		{
			  srcLeft[i] = ( (rxBuf[pos]<<16)|rxBuf[pos+1] );
			  srcRight[i] = ( (rxBuf[pos+2]<<16)|rxBuf[pos+3] );
			  i++;
		}

		i = 0;

		for ( int pos = startBuf ; pos < endBuf ; pos+=4 )
		{
			  int lval = srcLeft[i];
			  int rval = srcRight[i];

			  txBuf[pos] = (lval>>16)&0xFFFF;
			  txBuf[pos+1] = lval&0xFFFF;
			  txBuf[pos+2] = (rval>>16)&0xFFFF;
			  txBuf[pos+3] = rval&0xFFFF;

			  i++;
		}

//	  	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_13, GPIO_PIN_RESET );

}

void doFIR( int b )
{

	  	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_13, GPIO_PIN_SET );


		int startBuf = b * BUF_SAMPLES / 2;
		int endBuf = startBuf + BUF_SAMPLES / 2;

		int i = 0;
		for ( int pos = startBuf ; pos < endBuf ; pos+=4 )
		{
			  srcLeft[i] = ( (rxBuf[pos]<<16)|rxBuf[pos+1] );
			  srcRight[i] = ( (rxBuf[pos+2]<<16)|rxBuf[pos+3] );
			  i++;
		}

		i = 0;

		arm_fir_f32	(
				&arm_inst_left,
				srcLeft,
				destLeft,
				SAMPLES/2
		);

		arm_fir_f32	(
				&arm_inst_right,
				srcRight,
				destRight,
				SAMPLES/2
		);

		for ( int pos = startBuf ; pos < endBuf ; pos+=4 )
		{
			  int lval = (destLeft[i] + destRight[i]) * 8;
			  int rval = (destLeft[i] + destRight[i]) * 8;

			  txBuf[pos] = (lval>>16)&0xFFFF;
			  txBuf[pos+1] = lval&0xFFFF;
			  txBuf[pos+2] = (rval>>16)&0xFFFF;
			  txBuf[pos+3] = rval&0xFFFF;

			  i++;
		}

	  	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_13, GPIO_PIN_RESET );

}

void displaySSB()
{
	  if ( ssbMode == MODE_USB )
		  display->displaySSB("USB");
	  else
		  display->displaySSB("LSB");
}

void changeSideband()
{

  if ( ssbMode == MODE_USB )
	  ssbMode = MODE_LSB;
  else
	  ssbMode = MODE_USB;

  displaySSB();

  // Swap upper/lower sideband
  if ( coeffsLeft == plus45Coeffs )
  {
	  coeffsLeft = minus45Coeffs;
	  coeffsRight = plus45Coeffs;
  }
  else
  {
	  coeffsRight = minus45Coeffs;
	  coeffsLeft = plus45Coeffs;
  }

	  arm_fir_init_f32(
			  &arm_inst_left,
			  NUM_TAPS,
			  coeffsLeft,
			  &stateLeft[0],
			  SAMPLES/2
	  );

	  arm_fir_init_f32(
			  &arm_inst_right,
			  NUM_TAPS,
			  coeffsRight,
			  &stateRight[0],
			  SAMPLES/2
	  );
}

/*
 * 		NOTE: There appears to be a bug with

static void I2SEx_TxRxDMACplt(DMA_HandleTypeDef *hdma)

in stm32f4xx_hal_i2s_ex.c. The original function checked if DMA mode is NORMAL
and did nothing in the case of DMA mode == CIRCULAR. The following lines had to be
added to the function at the bottom

  else
  {
#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
        hi2s->TxRxCpltCallback(hi2s);
#else
        HAL_I2SEx_TxRxCpltCallback(hi2s);
#endif

 */

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	UNUSED(hi2s);
	fullComplete = 1;

}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	UNUSED(hi2s);
  	halfComplete = 1;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
