/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "logger.h"
#include "menu.h"
#include "bme280.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	TEMPERATURE_CONTROL_HEATING = 0, TEMPERATURE_CONTROL_COOLING
} TemperatureControlMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_BUFFER_SIZE              128
#define RX_BUFFER_SIZE              64

#define CONTROL_PERIOD_MS           1000U

#define DEFAULT_SETPOINT_C          25.0f
#define DEFAULT_KP                  8.0f
#define DEFAULT_KI                  0.05f
#define DEFAULT_KD                  0.0f

#define PID_MIN_OUTPUT              0.0f
#define PID_MAX_OUTPUT              100.0f

#define BME280_DEVICE_ADDRESS       BME280_I2C_ADDR_GND

#define FAN_PWM_TIMER               htim3
#define FAN_PWM_CHANNEL             TIM_CHANNEL_1

#define CMD_SETPOINT_LONG           "SETPOINT:"
#define CMD_SETPOINT_SHORT          "SET_SP:"

#define CMD_KP_LONG                 "KP:"
#define CMD_KP_SHORT                "SET_KP:"

#define CMD_KI_LONG                 "KI:"
#define CMD_KI_SHORT                "SET_KI:"

#define CMD_KD_LONG                 "KD:"
#define CMD_KD_SHORT                "SET_KD:"

#define CMD_START                   "START"
#define CMD_STOP                    "STOP"

#define CMD_MODE_COOLING            "MODE:COOLING"
#define CMD_MODE_HEATING            "MODE:HEATING"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
PID_Controller_t pid;

BME280_Handle_t bme280;
BME280_Config_t bme280Config;
BME280_Data_t bme280Data;
BME280_Status_t bme280Status;

TemperatureControlMode_t controlMode = TEMPERATURE_CONTROL_COOLING;

float temperatureSetPoint = DEFAULT_SETPOINT_C;

float temperature = 0.0f;
float humidity = 0.0f;
float pressure = 0.0f;

char uartTXBuffer[TX_BUFFER_SIZE];

uint8_t rxChar;
char messageBuffer[RX_BUFFER_SIZE];
uint8_t bufferIndex = 0;

volatile uint8_t pidEnabled = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void ApplyTemperatureSetPoint(void);
static float GetPIDInputFromTemperature(float measuredTemperature);
static const char* GetControlModeString(void);
static void SetFanPWMPercent(float percent);
static void SendTelemetry(void);
static void ClearUARTBuffer(void);
static void ProcessUARTCommand(char *command);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void ApplyTemperatureSetPoint(void) {
	if (controlMode == TEMPERATURE_CONTROL_COOLING) {
		PID_SetPoint(&pid, -temperatureSetPoint);
	} else {
		PID_SetPoint(&pid, temperatureSetPoint);
	}
}

static float GetPIDInputFromTemperature(float measuredTemperature) {
	if (controlMode == TEMPERATURE_CONTROL_COOLING) {
		return -measuredTemperature;
	}

	return measuredTemperature;
}

static const char* GetControlModeString(void) {
	if (controlMode == TEMPERATURE_CONTROL_COOLING) {
		return "COOLING";
	}

	return "HEATING";
}

static void SetFanPWMPercent(float percent) {
	if (percent > 100.0f) {
		percent = 100.0f;
	}

	if (percent < 0.0f) {
		percent = 0.0f;
	}

	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&FAN_PWM_TIMER);
	uint32_t compare = (uint32_t) ((percent / 100.0f) * (float) (arr + 1U));

	if (compare > arr) {
		compare = arr;
	}

	__HAL_TIM_SET_COMPARE(&FAN_PWM_TIMER, FAN_PWM_CHANNEL, compare);
}

static void SendTelemetry(void) {
	snprintf(uartTXBuffer, sizeof(uartTXBuffer),
			"SetPoint: %.2f Temperature: %.2f PIDOutput: %.2f Humidity: %.2f Pressure: %.2f Mode: %s\r\n",
			temperatureSetPoint, temperature, pid.output, humidity, pressure,
			GetControlModeString());

	HAL_UART_Transmit(&huart2, (uint8_t*) uartTXBuffer, strlen(uartTXBuffer),
	HAL_MAX_DELAY);
}

static void ClearUARTBuffer(void) {
	bufferIndex = 0;
	memset(messageBuffer, 0, RX_BUFFER_SIZE);
}

static void ProcessUARTCommand(char *command) {
	if (strncmp(command, CMD_SETPOINT_LONG, strlen(CMD_SETPOINT_LONG)) == 0) {
		temperatureSetPoint = atof(command + strlen(CMD_SETPOINT_LONG));
		ApplyTemperatureSetPoint();
		PID_Reset(&pid);
	} else if (strncmp(command, CMD_SETPOINT_SHORT, strlen(CMD_SETPOINT_SHORT))
			== 0) {
		temperatureSetPoint = atof(command + strlen(CMD_SETPOINT_SHORT));
		ApplyTemperatureSetPoint();
		PID_Reset(&pid);
	} else if (strncmp(command, CMD_KP_LONG, strlen(CMD_KP_LONG)) == 0) {
		float userKp = atof(command + strlen(CMD_KP_LONG));
		PID_SetKp(&pid, userKp);
		PID_Reset(&pid);
	} else if (strncmp(command, CMD_KP_SHORT, strlen(CMD_KP_SHORT)) == 0) {
		float userKp = atof(command + strlen(CMD_KP_SHORT));
		PID_SetKp(&pid, userKp);
		PID_Reset(&pid);
	} else if (strncmp(command, CMD_KI_LONG, strlen(CMD_KI_LONG)) == 0) {
		float userKi = atof(command + strlen(CMD_KI_LONG));
		PID_SetKi(&pid, userKi);
		PID_Reset(&pid);
	} else if (strncmp(command, CMD_KI_SHORT, strlen(CMD_KI_SHORT)) == 0) {
		float userKi = atof(command + strlen(CMD_KI_SHORT));
		PID_SetKi(&pid, userKi);
		PID_Reset(&pid);
	} else if (strncmp(command, CMD_KD_LONG, strlen(CMD_KD_LONG)) == 0) {
		float userKd = atof(command + strlen(CMD_KD_LONG));
		PID_SetKd(&pid, userKd);
		PID_Reset(&pid);
	} else if (strncmp(command, CMD_KD_SHORT, strlen(CMD_KD_SHORT)) == 0) {
		float userKd = atof(command + strlen(CMD_KD_SHORT));
		PID_SetKd(&pid, userKd);
		PID_Reset(&pid);
	} else if (strcmp(command, CMD_MODE_COOLING) == 0) {
		controlMode = TEMPERATURE_CONTROL_COOLING;
		ApplyTemperatureSetPoint();
		PID_Reset(&pid);
	} else if (strcmp(command, CMD_MODE_HEATING) == 0) {
		controlMode = TEMPERATURE_CONTROL_HEATING;
		ApplyTemperatureSetPoint();
		PID_Reset(&pid);
	} else if (strcmp(command, CMD_START) == 0) {
		pidEnabled = 1;
		PID_Reset(&pid);
	} else if (strcmp(command, CMD_STOP) == 0) {
		pidEnabled = 0;
		pid.output = 0.0f;
		SetFanPWMPercent(0.0f);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	loggerInit(LOG_LEVEL_INFO);

	ssd1306_Init();
	LOG_INFO("OLED initialized");

	displayMainMenu(temperatureSetPoint, 0.0f, 0);

	BME280_GetDefaultConfig(&bme280Config);

	bme280Config.mode = BME280_NORMAL_MODE;
	bme280Config.standby_time = BME280_STANDBY_TIME_1000;
	bme280Config.osrs_t = BME280_OVERSAMPLING_2;
	bme280Config.osrs_p = BME280_OVERSAMPLING_1;
	bme280Config.osrs_h = BME280_OVERSAMPLING_1;
	bme280Config.filter = BME280_FILTER_4;

	bme280Status = BME280_InitWithConfig(&bme280, &hi2c1,
	BME280_DEVICE_ADDRESS, &bme280Config);

	if (bme280Status != BME280_OK) {
		LOG_ERROR("BME280 initialization failed. Status: %d", bme280Status);

		ssd1306_Fill(Black);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("BME280 ERROR", Font_7x10, White);
		ssd1306_SetCursor(0, 20);
		ssd1306_WriteString("Check I2C/Addr", Font_7x10, White);
		ssd1306_UpdateScreen();

		Error_Handler();
	}

	LOG_INFO("BME280 initialized");

	PID_Init(&pid,
	DEFAULT_KP,
	DEFAULT_KI,
	DEFAULT_KD, (float) CONTROL_PERIOD_MS / 1000.0f,
	PID_MIN_OUTPUT,
	PID_MAX_OUTPUT);

	ApplyTemperatureSetPoint();

	HAL_TIM_PWM_Start(&FAN_PWM_TIMER, FAN_PWM_CHANNEL);
	SetFanPWMPercent(0.0f);

	HAL_UART_Receive_IT(&huart2, &rxChar, 1);

	uint32_t lastControlTime = HAL_GetTick();

	LOG_INFO("PID fan temperature controller started in %s mode",
			GetControlModeString());
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		uint32_t currentTime = HAL_GetTick();

		if (currentTime - lastControlTime >= CONTROL_PERIOD_MS) {
			bme280Status = BME280_ReadSensorData(&bme280, &bme280Data);

			if (bme280Status == BME280_OK) {
				temperature = bme280Data.temperature;
				humidity = bme280Data.humidity;
				pressure = bme280Data.pressure;

				if (pidEnabled) {
					float pidInput = GetPIDInputFromTemperature(temperature);
					pid.output = PID_Compute(&pid, pidInput);
				} else {
					pid.output = 0.0f;
				}

				SetFanPWMPercent(pid.output);

				uint8_t pwmPercent = (uint8_t) (pid.output + 0.5f);

				displayMainMenu(temperatureSetPoint, temperature, pwmPercent);

				SendTelemetry();

				LOG_INFO(
						"Mode: %s | SP: %.2f C | Temp: %.2f C | Fan PWM: %.2f %% | Hum: %.2f %% | Press: %.2f hPa",
						GetControlModeString(), temperatureSetPoint,
						temperature, pid.output, humidity, pressure);
			} else {
				pid.output = 0.0f;
				SetFanPWMPercent(0.0f);

				LOG_ERROR("BME280 read failed. Status: %d", bme280Status);
			}

			lastControlTime = currentTime;
		}

		HAL_Delay(5);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {

		if (rxChar == '\r' || rxChar == '\n') {
			if (bufferIndex > 0) {
				messageBuffer[bufferIndex] = '\0';
				ProcessUARTCommand(messageBuffer);
			}

			ClearUARTBuffer();
		} else {
			if (bufferIndex < RX_BUFFER_SIZE - 1) {
				messageBuffer[bufferIndex++] = (char) rxChar;
				messageBuffer[bufferIndex] = '\0';
			} else {
				ClearUARTBuffer();
			}
		}

		HAL_UART_Receive_IT(&huart2, &rxChar, 1);
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
