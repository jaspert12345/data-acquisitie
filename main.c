/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

#define BMP390_ADDR 0x77
#define BMP390_CHIP_ID_REG 0x00
#define BMP390_TEMP_DATA_REG 0x07  // Starting register for temperature data
#define BMP390_PRESS_DATA_REG 0x04 // Starting register for pressure data
#define BMP390_CTRL_MEAS_REG 0x1B
#define BMP390_CONFIG_REG 0x1F
#define BMP390_STATUS_REG 0x03
#define BMP390_SENSOR_TIME_0_REG 0x0C
#define BMP390_SENSOR_TIME_1_REG 0x0D
#define BMP390_SENSOR_TIME_2_REG 0x0E
#define BMP390_INT_CTRL_REG 0x19
#define BMP390_PWR_CTRL_REG 0x1B
#define BMP390_OSR_REG 0x1C
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float par_t1;
	float par_t2;
	float par_t3;
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;
	float t_lin;
} BMP390_CalibData;

BMP390_CalibData calib_data;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//CO2-sensor//
uint8_t CO2_ADDR = 0x70 << 1;
uint8_t CO2_REG = 0x0C;
//light-sensor//
uint16_t lightLevel = 0;
char lightLevelBuf[20];
//temperature and humidity//
static const uint8_t HIH7120_ADDR =  0x27 << 1;
//static const uint8_t REG_TEMP = 0x00;

//druksensor//
void BMP390_init();
void BMP390_ReadCalibrationData();
float BMP390_CompensateTemperature(uint32_t uncomp_temp);
float BMP390_CompensatePressure(uint32_t uncomp_press);
uint32_t BMP390_Read24BitData(uint8_t reg_addr);
void USART_Print(char *string);
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

//spi connection//
uint8_t txData[100];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef SPI_SendData(uint8_t* data, uint16_t size);
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
	//temperature and humidity//
	float humidity;
	float temperature;
	HAL_StatusTypeDef ret;
	int16_t val;
	uint16_t temp_c;
	uint8_t inputbuff[4];
	uint8_t buf[50];
	uint8_t reg; reg = 0x00;


	//CO2-sensor//
	uint16_t valueCO2 = 0;
	uint16_t valueVOC = 0;
	uint8_t CO2Read[34];

	//light-sensor//
	uint8_t buf2[12];
	uint16_t lightLevel = 0;
	char lightLevelBuf[20];
	uint8_t tx_Data[100];
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //spi-connection//
	  sprintf((char*)txData, "Temp:%u°C,Hum:%u,CO2:%uppm,VOC:%uppb,Pressure:%uPa,Light:%ulux\r\n", (unsigned int)temperature, (unsigned int)humidity, (unsigned int)valueCO2, (unsigned int)valueVOC, (unsigned int)lightLevel);
	  if (SPI_SendData(txData, sizeof(txData)) != HAL_OK) {
		  Error_Handler();
	  }


	  //temperature and humidity sensor//
	  // Step 1: Transmit the register address (0x00) to the sensor
	  ret = HAL_I2C_Master_Transmit(&hi2c3, HIH7120_ADDR, &reg, 1, HAL_MAX_DELAY);
	  HAL_Delay(250);
	  if ( ret != HAL_OK ) {
		  strcpy((char*)buf, "Error Tx\r\n");
	  }
	  else {// Step 2: Receive 4 bytes of data from the sensor into inputbuf
		  ret = HAL_I2C_Master_Receive(&hi2c3, HIH7120_ADDR, inputbuff, 4, HAL_MAX_DELAY);
		  HAL_Delay(250);
		  if ( ret != HAL_OK ){
			  strcpy((char*)buf, "Error Rx\r\n");
		  }
		  else {
			  uint16_t raw_humidity = (((uint16_t)(inputbuff[0] & 0x3f) << 8) | inputbuff[1]);// Humidity is in the first 2 bytes
			  uint16_t raw_temperature = (((uint16_t)inputbuff[2] << 6) | (inputbuff[3] >> 2));// Temperature is in the next 2 bytes
			  humidity = ((float)raw_humidity / 16382.0) * 100.0; // Convert raw data to actual humidity and temperature values
			  temperature = ((float)raw_temperature / 16382.0) * 165.0 - 40.0;
			  snprintf(buf, sizeof(buf), "Humidity: %.2f%%, Temperature: %.2f°C\r\n", humidity, temperature);// Format the data into the buffer
			  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);// Transmit the formatted string over UART
		  }
		  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
	  }
	  //light-sensor//
	  strcpy((char*)buf2, "Hello!\r\n");
	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf2), HAL_MAX_DELAY);
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 20);
	  lightLevel = HAL_ADC_GetValue(&hadc1);
	  lightLevel = lightLevel / 4.095;
	  sprintf(lightLevelBuf, "lightLevel: %hu \r\n", lightLevel);
	  HAL_UART_Transmit(&huart2, (uint8_t *)lightLevelBuf, strlen(lightLevelBuf), HAL_MAX_DELAY);
	  HAL_Delay(500);

	  //CO2-sensor//
	  CO2Read[0] = CO2_REG;
	  ret = HAL_I2C_Master_Transmit(&hi2c1, CO2_ADDR, &CO2_REG, 1, HAL_MAX_DELAY);
	  HAL_Delay(250);
	  if (ret != HAL_OK){
	   strcpy((char*)CO2Read, "Error Tx\r\n");
	  } else {
	  ret = HAL_I2C_Master_Receive(&hi2c1, CO2_ADDR, CO2Read, 2, HAL_MAX_DELAY);
	  if (ret != HAL_OK){
	  strcpy((char*)CO2Read, "Error Rx\r\n");
	  } else {
	  valueVOC = ((uint16_t)CO2Read[0] - 13) * (1000/229); //in ppb
	  valueCO2 = ((uint16_t)CO2Read[1] - 13) * (1600/229) + 400; //in ppm
	  sprintf((char*)CO2Read, "CO2: %u ppm \r\nVOC: %u ppb \r\n", (unsigned int)valueCO2, (unsigned int)valueVOC);
	  }
	  }
	  HAL_UART_Transmit(&huart2, CO2Read, strlen((char*)CO2Read), HAL_MAX_DELAY);
	  HAL_Delay(250);

	  //druksensor//
	  uint32_t temp_raw = BMP390_Read24BitData(BMP390_TEMP_DATA_REG);
	  uint32_t press_raw = BMP390_Read24BitData(BMP390_PRESS_DATA_REG);

	  float temperature = BMP390_CompensateTemperature(temp_raw);
	  float pressure = BMP390_CompensatePressure(press_raw);

	  char output[100];
	  snprintf(output, sizeof(output), "Temperature: %.2f C, Pressure: %.2f hPa\r\n", temperature, pressure);
	  USART_Print(output);

	  HAL_Delay(1000);

  }


    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//druksensor//
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    I2C_HandleTypeDef *i2c = (I2C_HandleTypeDef *)intf_ptr;
    if (HAL_I2C_Mem_Read(i2c, BMP390_ADDR , reg_addr, 1, reg_data, len, HAL_MAX_DELAY)) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    I2C_HandleTypeDef *i2c = (I2C_HandleTypeDef *)intf_ptr;
    if (HAL_I2C_Mem_Write(i2c, BMP390_ADDR, reg_addr, 1, (uint8_t *)reg_data, len, HAL_MAX_DELAY)) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

void BMP390_init() {
    uint8_t data;

    data = 0x33;
    i2c_write(BMP390_CTRL_MEAS_REG, &data, 1, &hi2c1);


    data = 0x00;
    i2c_write(BMP390_CONFIG_REG, &data, 1, &hi2c1);
    BMP390_ReadCalibrationData();
}

void BMP390_ReadCalibrationData() {
    uint8_t calib_buffer[21];
    uint8_t calib_start_reg = 0x31;


    HAL_I2C_Mem_Read(&hi2c1, BMP390_ADDR, calib_start_reg, I2C_MEMADD_SIZE_8BIT, calib_buffer, sizeof(calib_buffer), HAL_MAX_DELAY);


    // Temperature parameters
    calib_data.par_t1 = (float)((calib_buffer[1] << 8) | calib_buffer[0]) / pow(2, 8);
    calib_data.par_t2 = (float)((calib_buffer[3] << 8) | calib_buffer[2]) / pow(2, 30);
    calib_data.par_t3 = (float)(calib_buffer[4]) / pow(2, 48);

    // Pressure parameters
    calib_data.par_p1 = (float)(((calib_buffer[6] << 8) | calib_buffer[5]) - pow(2, 14)) / pow(2, 20);
    calib_data.par_p2 = (float)(((calib_buffer[8] << 8) | calib_buffer[7]) - pow(2, 14)) / pow(2, 29);
    calib_data.par_p3 = (float)(calib_buffer[9]) / pow(2, 32);
    calib_data.par_p4 = (float)(calib_buffer[10]) / pow(2, 37);
    calib_data.par_p5 = (float)((calib_buffer[11])) / pow(2, -3);
    calib_data.par_p6 = (float)((calib_buffer[12])) / pow(2, 6);
    calib_data.par_p7 = (float)((calib_buffer[13])) / pow(2, 8);
    calib_data.par_p8 = (float)((calib_buffer[14])) / pow(2, 15);
    calib_data.par_p9 = (float)((calib_buffer[15])) / pow(2, 48);
    calib_data.par_p10 = (float)((calib_buffer[16])) / pow(2, 48);
    calib_data.par_p11 = (float)((calib_buffer[17])) / pow(2, 65);


    printf("Calibration Data:\n");
    printf("par_t1: %f, par_t2: %f, par_t3: %f\n", calib_data.par_t1, calib_data.par_t2, calib_data.par_t3);
    printf("par_p1: %f, par_p2: %f, par_p3: %f\n", calib_data.par_p1, calib_data.par_p2, calib_data.par_p3);

}

float BMP390_CompensateTemperature(uint32_t uncomp_temp) {
	float partial_data1 = ((float)uncomp_temp - calib_data.par_t1);
	float partial_data2 = partial_data1 * calib_data.par_t2;

	calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data.par_t3;

	printf("Temperature Debug:\n");
	printf("uncomp_temp: %lu, partial_data1: %f, partial_data2: %f, t_lin: %f\n",
			uncomp_temp, partial_data1, partial_data2, calib_data.t_lin);
	return calib_data.t_lin;
}

float BMP390_CompensatePressure(uint32_t uncomp_press) {
	float partial_data1 = calib_data.par_p6 * calib_data.t_lin;
	float partial_data2 = calib_data.par_p7 * calib_data.t_lin * calib_data.t_lin;
	float partial_data3 = calib_data.par_p8 * calib_data.t_lin * calib_data.t_lin * calib_data.t_lin;
	float partial_out1 = calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = calib_data.par_p2 * calib_data.t_lin;
	partial_data2 = calib_data.par_p3 * calib_data.t_lin * calib_data.t_lin;
	partial_data3 = calib_data.par_p4 * calib_data.t_lin * calib_data.t_lin * calib_data.t_lin;
	float partial_out2 = (float)uncomp_press * (calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = calib_data.par_p9 + calib_data.par_p10 * calib_data.t_lin;
	partial_data3 = partial_data1 * partial_data2;
	float partial_data4 = partial_data1 * (float)uncomp_press * calib_data.par_p11;

	float comp_press = partial_out1 + partial_out2 + partial_data3 + partial_data4;

	printf("Pressure Debug:\n");
	printf("uncomp_press: %lu, partial_out1: %f, partial_out2: %f, comp_press: %f\n",
			uncomp_press, partial_out1, partial_out2, comp_press);

	return comp_press / 100.0f;
}

uint32_t BMP390_Read24BitData(uint8_t reg_addr) {
	uint8_t buffer[3];
	i2c_read(reg_addr, buffer, 3, &hi2c1);
	return ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
}

void USART_Print(char *string) {
    HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), HAL_MAX_DELAY);
}
//SPI-connection//
HAL_StatusTypeDef SPI_SendData(uint8_t* data, uint16_t size) {
	return HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
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
