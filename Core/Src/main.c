/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "accel.h"
#include "attitude.h"
#include "baro.h"
#include "control_motor.h"
#include "gyro.h"
#include "mag.h"
#include "iks01a2_env_sensors.h"
#include "iks01a2_motion_sensors.h"
#include <stdio.h>
#include <string.h>
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

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for tControlMotor */
osThreadId_t tControlMotorHandle;
const osThreadAttr_t tControlMotor_attributes = {
  .name = "tControlMotor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for tAttitude */
osThreadId_t tAttitudeHandle;
const osThreadAttr_t tAttitude_attributes = {
  .name = "tAttitude",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for tAltitude */
osThreadId_t tAltitudeHandle;
const osThreadAttr_t tAltitude_attributes = {
  .name = "tAltitude",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for tGetAccLSM6DSL */
osThreadId_t tGetAccLSM6DSLHandle;
const osThreadAttr_t tGetAccLSM6DSL_attributes = {
  .name = "tGetAccLSM6DSL",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tGetAccLSM303AG */
osThreadId_t tGetAccLSM303AGHandle;
const osThreadAttr_t tGetAccLSM303AG_attributes = {
  .name = "tGetAccLSM303AG",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tGetGyrLSM6DSL */
osThreadId_t tGetGyrLSM6DSLHandle;
const osThreadAttr_t tGetGyrLSM6DSL_attributes = {
  .name = "tGetGyrLSM6DSL",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tGetMagnetLSM30 */
osThreadId_t tGetMagnetLSM30Handle;
const osThreadAttr_t tGetMagnetLSM30_attributes = {
  .name = "tGetMagnetLSM30",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tGetPressLPS22H */
osThreadId_t tGetPressLPS22HHandle;
const osThreadAttr_t tGetPressLPS22H_attributes = {
  .name = "tGetPressLPS22H",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tPrintUART */
osThreadId_t tPrintUARTHandle;
const osThreadAttr_t tPrintUART_attributes = {
  .name = "tPrintUART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for semPressLPS22HB */
osSemaphoreId_t semPressLPS22HBHandle;
const osSemaphoreAttr_t semPressLPS22HB_attributes = {
  .name = "semPressLPS22HB"
};
/* Definitions for semAccLSM303AGR */
osSemaphoreId_t semAccLSM303AGRHandle;
const osSemaphoreAttr_t semAccLSM303AGR_attributes = {
  .name = "semAccLSM303AGR"
};
/* Definitions for semAccLSM6DSL */
osSemaphoreId_t semAccLSM6DSLHandle;
const osSemaphoreAttr_t semAccLSM6DSL_attributes = {
  .name = "semAccLSM6DSL"
};
/* Definitions for semMagnetLSM303AGR */
osSemaphoreId_t semMagnetLSM303AGRHandle;
const osSemaphoreAttr_t semMagnetLSM303AGR_attributes = {
  .name = "semMagnetLSM303AGR"
};
/* Definitions for semGyrLSM6DSL */
osSemaphoreId_t semGyrLSM6DSLHandle;
const osSemaphoreAttr_t semGyrLSM6DSL_attributes = {
  .name = "semGyrLSM6DSL"
};
/* USER CODE BEGIN PV */
volatile float pressure_LPS22HB;
volatile IKS01A2_MOTION_SENSOR_Axes_t axesAcc_LSM303AG;
volatile IKS01A2_MOTION_SENSOR_Axes_t axesAcc_LSM6DSL;
volatile IKS01A2_MOTION_SENSOR_Axes_t axesGyr_LSM6DSL;
volatile IKS01A2_MOTION_SENSOR_Axes_t axesMag_LSM303AGR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void startTaskControlMotor(void *argument);
void startTaskAttitude(void *argument);
void startTaskAltitude(void *argument);
void StartTaskGetAccLSM6DSL(void *argument);
void StartTaskGetAccLSM303AGR(void *argument);
void StartTaskGetGyrLSM6DSL(void *argument);
void StartTaskGetMagnetLSM303AGR(void *argument);
void StartTaskGetPressLPS22HB(void *argument);
void StartTaskPrintUART(void *argument);

/* USER CODE BEGIN PFP */
static void sensorGyroInit(struct gyroDev_s *gyro);
static bool sensorGyroRead(struct gyroDev_s *gyro);
static void sensorAccInit(struct accDev_s *acc);
static bool sensorAccRead(struct accDev_s *acc);
static void sensorMagInit(struct magDev_s *mag);
static bool sensorMagRead(struct magDev_s *mag);
static void sensorBaroInit(struct baroDev_s *baro);
static bool sensorBaroRead(struct baroDev_s *baro);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
 for (int i = 0; i < len; ++i) {
 ITM_SendChar(*ptr++);
 }
 return len;
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  /* initialization of control task */
  gyroInit(sensorGyroInit, sensorGyroRead);
  controlMotorInit();

  /* initialization of attitude task */
  accInit(sensorAccInit, sensorAccRead);
  magInit(sensorMagInit, sensorMagRead);
  attitudeInit();

  /* initialization of altitude task */
  baroInit(sensorBaroInit, sensorBaroRead);

  /* initialization of sensors */
  	  /* initialization of sensor LSM6DSL*/
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);
  IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);

  	  /* initialization of sensor LSM303AGR*/
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);

  IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);
  IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
  	  /* initialization of pressure sensor LPS22HB*/
  IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_PRESSURE);
  IKS01A2_ENV_SENSOR_Enable(IKS01A2_LPS22HB_0, ENV_PRESSURE);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semPressLPS22HB */
  semPressLPS22HBHandle = osSemaphoreNew(1, 1, &semPressLPS22HB_attributes);

  /* creation of semAccLSM303AGR */
  semAccLSM303AGRHandle = osSemaphoreNew(1, 1, &semAccLSM303AGR_attributes);

  /* creation of semAccLSM6DSL */
  semAccLSM6DSLHandle = osSemaphoreNew(1, 1, &semAccLSM6DSL_attributes);

  /* creation of semMagnetLSM303AGR */
  semMagnetLSM303AGRHandle = osSemaphoreNew(1, 1, &semMagnetLSM303AGR_attributes);

  /* creation of semGyrLSM6DSL */
  semGyrLSM6DSLHandle = osSemaphoreNew(1, 1, &semGyrLSM6DSL_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of tControlMotor */
  tControlMotorHandle = osThreadNew(startTaskControlMotor, NULL, &tControlMotor_attributes);

  /* creation of tAttitude */
  tAttitudeHandle = osThreadNew(startTaskAttitude, NULL, &tAttitude_attributes);

  /* creation of tAltitude */
  tAltitudeHandle = osThreadNew(startTaskAltitude, NULL, &tAltitude_attributes);

  /* creation of tGetAccLSM6DSL */
  tGetAccLSM6DSLHandle = osThreadNew(StartTaskGetAccLSM6DSL, NULL, &tGetAccLSM6DSL_attributes);

  /* creation of tGetAccLSM303AG */
  tGetAccLSM303AGHandle = osThreadNew(StartTaskGetAccLSM303AGR, NULL, &tGetAccLSM303AG_attributes);

  /* creation of tGetGyrLSM6DSL */
  tGetGyrLSM6DSLHandle = osThreadNew(StartTaskGetGyrLSM6DSL, NULL, &tGetGyrLSM6DSL_attributes);

  /* creation of tGetMagnetLSM30 */
  tGetMagnetLSM30Handle = osThreadNew(StartTaskGetMagnetLSM303AGR, NULL, &tGetMagnetLSM30_attributes);

  /* creation of tGetPressLPS22H */
  tGetPressLPS22HHandle = osThreadNew(StartTaskGetPressLPS22HB, NULL, &tGetPressLPS22H_attributes);

  /* creation of tPrintUART */
  tPrintUARTHandle = osThreadNew(StartTaskPrintUART, NULL, &tPrintUART_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/*uint32_t start = HAL_GetTick();
	for (int i = 0; i < 100000; ++i) {*/
		/* first task: control */
	    /*gyroUpdate();
		controlMotorUpdate();*/

		/* second task: attitude */
		/*accUpdate();
		magUpdate();
		attitudeUpdate();*/

		/* third task: altitude */
		/*baroUpdate();
	}*/
	/*volatile uint32_t last = HAL_GetTick() - start;
	Error_Handler();*/
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void sensorGyroInit(struct gyroDev_s *gyro) { }

static bool sensorGyroRead(struct gyroDev_s *gyro) {
	gyro->gyroADC[0] = 0;
	gyro->gyroADC[1] = -12345;
	gyro->gyroADC[2] = -28414;
	return true;
}

static void sensorAccInit(struct accDev_s *acc) { }

static bool sensorAccRead(struct accDev_s *acc) {
	IKS01A2_MOTION_SENSOR_Axes_t axes_LSM303AG;
	IKS01A2_MOTION_SENSOR_Axes_t axes_LSM6DSL;

	//LSM303AG sensor accelerometer
	osSemaphoreAcquire(semAccLSM303AGRHandle, osWaitForever );
	axes_LSM303AG = axesAcc_LSM303AG;
	osSemaphoreRelease(semAccLSM303AGRHandle);
	//LSM303AG sensor accelerometer
	osSemaphoreAcquire(semAccLSM6DSLHandle, osWaitForever );
	axes_LSM6DSL = axesAcc_LSM6DSL;
	osSemaphoreRelease(semAccLSM6DSLHandle);

	acc->accADC[0] = (axes_LSM303AG.x + axes_LSM6DSL.x ) / 2;
	acc->accADC[1] = (axes_LSM303AG.y + axes_LSM6DSL.y ) / 2;
	acc->accADC[2] = (axes_LSM303AG.z + axes_LSM6DSL.z ) / 2;
	return true;
}

static void sensorMagInit(struct magDev_s *mag) { }

static bool sensorMagRead(struct magDev_s *mag) {
	mag->magADC[0] = 2464;
	mag->magADC[1] = -3257;
	mag->magADC[2] = 1588;
	return true;
}

static void sensorBaroInit(struct baroDev_s *baro) {
	baro->baroADC = 0;
}

static bool sensorBaroRead(struct baroDev_s *baro) {
	//baro->baroADC = 12000;
	osSemaphoreAcquire(semPressLPS22HBHandle, osWaitForever );
	baro->baroADC = pressure_LPS22HB;
	//LPS22HB_Pressure = pressure;
	osSemaphoreRelease(semPressLPS22HBHandle);
	return true;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startTaskControlMotor */
/**
  * @brief  Function implementing the tControlMotor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startTaskControlMotor */
void startTaskControlMotor(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	gyroUpdate();
	controlMotorUpdate();
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startTaskAttitude */
/**
* @brief Function implementing the tAttitude thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskAttitude */
void startTaskAttitude(void *argument)
{
  /* USER CODE BEGIN startTaskAttitude */
  /* Infinite loop */
  for(;;)
  {
    accUpdate();
    magUpdate();
    attitudeUpdate();
    osDelay(1);
  }
  /* USER CODE END startTaskAttitude */
}

/* USER CODE BEGIN Header_startTaskAltitude */
/**
* @brief Function implementing the tAltitude thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskAltitude */
void startTaskAltitude(void *argument)
{
  /* USER CODE BEGIN startTaskAltitude */
  /* Infinite loop */
  for(;;)
  {
    baroUpdate();
    osDelay(1);
  }
  /* USER CODE END startTaskAltitude */
}

/* USER CODE BEGIN Header_StartTaskGetAccLSM6DSL */
/**
* @brief Function implementing the tGetAccLSM6DSL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGetAccLSM6DSL */
void StartTaskGetAccLSM6DSL(void *argument)
{
  /* USER CODE BEGIN StartTaskGetAccLSM6DSL */
  /* Infinite loop */
  for(;;)
  {
	IKS01A2_MOTION_SENSOR_Axes_t axes;
	IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, &axes);
	osSemaphoreAcquire(semAccLSM6DSLHandle, osWaitForever );
	axesAcc_LSM6DSL = axes;
	osSemaphoreRelease(semAccLSM6DSLHandle);
	osDelay(100);
  }
  /* USER CODE END StartTaskGetAccLSM6DSL */
}

/* USER CODE BEGIN Header_StartTaskGetAccLSM303AGR */
/**
* @brief Function implementing the tGetAccLSM303AG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGetAccLSM303AGR */
void StartTaskGetAccLSM303AGR(void *argument)
{
  /* USER CODE BEGIN StartTaskGetAccLSM303AGR */
  /* Infinite loop */
  for(;;)
  {
	IKS01A2_MOTION_SENSOR_Axes_t axes;
	IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO, &axes);
	osSemaphoreAcquire(semAccLSM303AGRHandle, osWaitForever );
    axesAcc_LSM303AG = axes;
	osSemaphoreRelease(semAccLSM303AGRHandle);
	osDelay(100);
  }
  /* USER CODE END StartTaskGetAccLSM303AGR */
}

/* USER CODE BEGIN Header_StartTaskGetGyrLSM6DSL */
/**
* @brief Function implementing the tGetGyrLSM6DSL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGetGyrLSM6DSL */
void StartTaskGetGyrLSM6DSL(void *argument)
{
  /* USER CODE BEGIN StartTaskGetGyrLSM6DSL */
  /* Infinite loop */
  for(;;)
  {
	IKS01A2_MOTION_SENSOR_Axes_t axes;
	IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM303AGR_MAG_0, MOTION_GYRO, &axes);
	osSemaphoreAcquire(semGyrLSM6DSLHandle, osWaitForever );
	axesGyr_LSM6DSL = axes;
	osSemaphoreRelease(semGyrLSM6DSLHandle);
    osDelay(1);
  }
  /* USER CODE END StartTaskGetGyrLSM6DSL */
}

/* USER CODE BEGIN Header_StartTaskGetMagnetLSM303AGR */
/**
* @brief Function implementing the tGetMagnetLSM30 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGetMagnetLSM303AGR */
void StartTaskGetMagnetLSM303AGR(void *argument)
{
  /* USER CODE BEGIN StartTaskGetMagnetLSM303AGR */
  /* Infinite loop */
  for(;;)
  {
	IKS01A2_MOTION_SENSOR_Axes_t axes;
	IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO, &axes);
	osSemaphoreAcquire(semMagnetLSM303AGRHandle, osWaitForever );
	axesMag_LSM303AGR = axes;
	osSemaphoreRelease(semMagnetLSM303AGRHandle);
    osDelay(10);
  }
  /* USER CODE END StartTaskGetMagnetLSM303AGR */
}

/* USER CODE BEGIN Header_StartTaskGetPressLPS22HB */
/**
* @brief Function implementing the tGetPressLPS22H thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGetPressLPS22HB */
void StartTaskGetPressLPS22HB(void *argument)
{
  /* USER CODE BEGIN StartTaskGetPressLPS22HB */
  /* Infinite loop */
  float pressure;
  for(;;)
  {
	//int res =
    IKS01A2_ENV_SENSOR_GetValue(IKS01A2_HTS221_0, ENV_PRESSURE, &pressure);
	osSemaphoreAcquire(semPressLPS22HBHandle, osWaitForever );
	pressure_LPS22HB = pressure;
	osSemaphoreRelease(semPressLPS22HBHandle);
    osDelay(20);
  }
  /* USER CODE END StartTaskGetPressLPS22HB */
}

/* USER CODE BEGIN Header_StartTaskPrintUART */
/**
* @brief Function implementing the tPrintUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPrintUART */
void StartTaskPrintUART(void *argument)
{
  /* USER CODE BEGIN StartTaskPrintUART */
  /* Infinite loop */
  for(;;)
  {
	printf("Magnet: \n");
	printf("x:%d\n", (int)axesMag_LSM303AGR.x);
	printf("y:%d\n", (int)axesMag_LSM303AGR.y);
	printf("z:%d\n", (int)axesMag_LSM303AGR.z);
    osDelay(1000);
  }
  /* USER CODE END StartTaskPrintUART */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
