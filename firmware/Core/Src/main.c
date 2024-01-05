/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "at24cxx_hal.h"
#include "ts_config.h"
#include "maf.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

AT24CXX_HandleTypeDef eeprom_chip;

extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
extern uint16_t received_data_length;
extern bool received_data;

extern TS_Config ts_config;
extern TS_Signature ts_signature;

uint16_t livedata[8];
int16_t mcu_temp = 0;
int16_t vref = 0;
uint16_t hfm1_in_v = 0;
uint16_t hfm2_in_v = 0;
uint16_t hfm1_air = 0;
uint16_t hfm2_air = 0;
uint16_t hfm1_out_v = 0;
uint16_t hfm2_out_v = 0;
uint16_t hfm1_out_dac = 0;
uint16_t hfm2_out_dac = 0;

uint16_t adc_values[5] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

static void AT24CXX_Init(void);
static int16_t calc_temperature (uint16_t data);
static int16_t calc_vref (uint16_t data);
static uint16_t calc_voltage (uint16_t data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DAC1_Init();
  MX_I2C2_Init();
  MX_USB_Device_Init();
  MX_ADC1_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 5);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim17);

  AT24CXX_Init();


  uint8_t configCheck;

  if (at24cxx_connected(eeprom_chip) == HAL_OK)
  {
	  at24cxx_read(eeprom_chip, 0, 0, (uint8_t*)&configCheck, sizeof (configCheck));

	  if (configCheck == 0xff)
	  {
		  // chip is not initialised, so write a default configuration...
		  create_initial_config(&ts_config, &ts_signature);

		  at24cxx_write(eeprom_chip, TS_SIGNATURE_PAGE, 0, (uint8_t*)&ts_signature, sizeof(ts_signature));
		  at24cxx_write(eeprom_chip, TS_CONFIG_PAGE, 0, (uint8_t*)&ts_config, sizeof(ts_config));
	  }
	  else
	  {
		  at24cxx_read(eeprom_chip, TS_SIGNATURE_PAGE, 0, (uint8_t*)&ts_signature, sizeof(ts_signature));
		  at24cxx_read(eeprom_chip, TS_CONFIG_PAGE, 0, (uint8_t*)&ts_config, sizeof(ts_config));
	  }


	  // at24cxx_erase_chip(eeprom_chip);

	  /*
	  at24cxx_write(eeprom_chip, 1, 0, (uint8_t*)&dataw, strlen((char *)dataw));
	  at24cxx_read(eeprom_chip, 1, 0, (uint8_t*)&datar, sizeof (datar));


	  at24cxx_write_16(eeprom_chip, 0, 0, data16_w);
	  data16_r = at24cxx_read_16(eeprom_chip, 0, 0);
	  */
  }

  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2284);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 3999);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (received_data)
	  {
		  // do stuff with new received data....

		  if (received_data_length == 1)
		  {
			  // we have got a single byte command...

			  switch (UserRxBufferFS[0])
			  {
				  case 'Z':
					  at24cxx_erase_chip(eeprom_chip);

					  create_initial_config(&ts_config, &ts_signature);

					  at24cxx_write(eeprom_chip, TS_SIGNATURE_PAGE, 0, (uint8_t*)&ts_signature, sizeof(ts_signature));
					  at24cxx_write(eeprom_chip, TS_CONFIG_PAGE, 0, (uint8_t*)&ts_config, sizeof(ts_config));
					  break;
				  case 'F':
					  CDC_Transmit_FS((uint8_t*) ts_signature.version, sizeof(ts_signature.version));
					  break;
				  case 'Q':
					  CDC_Transmit_FS((uint8_t*) ts_signature.signature, sizeof(ts_signature.signature));
					  break;
				  case 'S':
					  CDC_Transmit_FS((uint8_t*) ts_signature.version_info, sizeof(ts_signature.version_info));
					  break;
				  case 'A':
					  livedata[0] = hfm1_in_v;
					  livedata[1] = hfm2_in_v;
					  livedata[2] = hfm1_air;
					  livedata[3] = hfm2_air;
					  livedata[4] = hfm1_out_v;
					  livedata[5] = hfm2_out_v;
					  livedata[6] = mcu_temp;
					  livedata[7] = vref;
					  CDC_Transmit_FS((uint8_t*) livedata, sizeof(livedata));
					  break;

				  default:
					  CDC_Transmit_FS((uint8_t*) "Not implemented", 15);
					  break;
			  }
		  }

		  if (received_data_length > 1)
		  {
			  // we have got a multibyte command...
			  uint8_t cmd[received_data_length];
			  memcpy(cmd, UserRxBufferFS, received_data_length);

			  switch(cmd[0])
			  {
			  	  case 'B':
					  {
						  uint16_t page_indentifier = word(cmd[2], cmd[1]);
						  at24cxx_write(eeprom_chip, page_indentifier, 0, (uint8_t*)&ts_config, sizeof(ts_config));
					  }
					  break;
			  	  case 'R':
					  {
					  /* reading page data, 6 bytes expected
					   * 2 bytes: page identifier
					   * 2 bytes: offset
					   * 2 bytes: length
					  */
						  uint16_t page_indentifier = word(cmd[2], cmd[1]);
						  uint16_t offset = word(cmd[4], cmd[3]);
						  uint16_t lenght = word(cmd[6], cmd[5]);
						  uint8_t config[lenght];

						  at24cxx_read(eeprom_chip, page_indentifier, offset, (uint8_t*)&config, lenght);
						  CDC_Transmit_FS((uint8_t*) &config, lenght);
					  }
					  break;
			  	  case 'W':
					  {
						  /* we expect 9 bytes
						   * 1 byte: W
						   * 2 bytes: pageIdentifier
						   * 2 bytes: offset / memory address, value: parameterStartOffset + offset
						   * 2 bytes: length of bytes to be written
						   * 2 bytes: byte array
						   */

						  // the real config is starts at offset 64, 0-63 is signature...

						  uint16_t page_indentifier = word(cmd[2], cmd[1]);
						  uint16_t offset = word(cmd[4], cmd[3]);
						  uint16_t lenght = word(cmd[6], cmd[5]);
						  uint16_t value = 0;

						  if (offset == lenght)
						  {
							  // single byte value
							  value = cmd[7];
						  }
						  else
						  {
							  // two byte value
							  value = word(cmd[8], cmd[7]);
						  }

						  set_page_value(offset, value, &ts_config);
					  }
			  		  break;
			  }

		  }

		  received_data = false;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
	  Error_Handler();
  }

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
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  if (HAL_DACEx_SelfCalibrate(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
   {
     Error_Handler();
   }

  if (HAL_DACEx_SelfCalibrate(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
   {
     Error_Handler();
   }

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00E057FD;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 14400-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USER_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USER_Pin LD2_Pin */
  GPIO_InitStruct.Pin = USER_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void AT24CXX_Init(void)
{
	eeprom_chip.at24cxx_address = 0xA0;
	eeprom_chip.at24cxx_page_number = 512;
	eeprom_chip.at24cxx_page_size = 64;
	eeprom_chip.i2c_device = hi2c2;
}

static int16_t calc_temperature (uint16_t data)
{

	/*
	 * Formula:
	 * temp in °C = ((TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1)) * (TS_DATA - TS_CAL1) + 30°C
	 *
	 * TS_DATA is value from ADC
	 *
	 */

	float pre = ((float) data / 3) * 3.3;	// rescale from 3.0 volt to 3.3 volts, since the reference values are taken at 3.0volts
	float first = (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)/(float)(*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR);
	float second = (float)(pre - *TEMPSENSOR_CAL1_ADDR);
	float third = first * second + 30.0;

	return (int16_t)(third * 100);
}

static int16_t calc_vref (uint16_t data)
{
	/*
	 * Formula:
	 * Vref = Vref_char * VREFINT_CAL / VREFINT_DATA
	 */

	float first = (float)(VREFINT_CAL_VREF * *VREFINT_CAL_ADDR);
	float second = first / (float)(data);	// vdda...
	float third = second / (float) 4096 * (float)(data);	// vref


	return (int16_t)(third);
}

static uint16_t calc_voltage (uint16_t data)
{
	float first = 3.3 / (float) 4096;
	float second = first * (float) data;
	float third = 0;

	if (ts_config.showRealADCVoltages)
	{
		third = second;
	}
	else
	{
		third = second * 1.5151515151;
	}

	return (uint16_t)(third*1000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim17) {
		mcu_temp = calc_temperature(adc_values[3]);
		vref = calc_vref(adc_values[4]);

		if (ts_config.isChannel1Enabled)
		{
			hfm1_in_v = calc_voltage(adc_values[0]);
			hfm1_air = interpolate_voltage2airMass(hfm1_in_v);
			hfm1_out_v = interpolate_airMass2voltage(hfm1_air);
			hfm1_out_dac = calculate_dac_value_for_voltage(hfm1_out_v);
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, hfm1_out_dac);
		}

		if (ts_config.isChannel2Enabled)
		{
			hfm2_in_v = calc_voltage(adc_values[1]);
			hfm2_air = interpolate_voltage2airMass(hfm2_in_v);
			hfm2_out_v = interpolate_airMass2voltage(hfm2_air);
			hfm2_out_dac = calculate_dac_value_for_voltage(hfm2_out_v);
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, hfm2_out_dac);
		}

		int i = 10;

	}
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
