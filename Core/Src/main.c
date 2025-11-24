/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lis3dh_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union
{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float Ax, Ay, Az;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LIS3DH_ADDR (0x19U << 1) // Adjust according to your SA0 wiring
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void LIS3DH_ContextInit(void);
static int32_t LIS3DH_Init(void);
static int32_t LIS3DH_Read_Accel(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static stmdev_ctx_t lis3dh_ctx;

int32_t platform_write(void *handle, uint8_t reg,
                       const uint8_t *bufp, uint16_t len)
{
  // Multi-byte write: uses auto-increment mode
  if (HAL_I2C_Mem_Write(&hi2c1,
                        LIS3DH_ADDR,
                        reg,
                        I2C_MEMADD_SIZE_8BIT,
                        (uint8_t *)bufp,
                        len,
                        100) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

int32_t platform_read(void *handle, uint8_t reg,
                      uint8_t *bufp, uint16_t len)
{
  if (HAL_I2C_Mem_Read(&hi2c1,
                       LIS3DH_ADDR,
                       reg,
                       I2C_MEMADD_SIZE_8BIT,
                       bufp,
                       len,
                       100) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

void platform_delay(uint32_t millisec)
{
  HAL_Delay(millisec);
}

static void LIS3DH_ContextInit(void)
{
  lis3dh_ctx.write_reg = platform_write;
  lis3dh_ctx.read_reg = platform_read;
  lis3dh_ctx.mdelay = platform_delay;
  lis3dh_ctx.handle = &hi2c1;
  lis3dh_ctx.priv_data = NULL;
}

static int32_t LIS3DH_Init(void)
{
  uint8_t whoamI = 0;
  int32_t ret;

  ret = lis3dh_device_id_get(&lis3dh_ctx, &whoamI);
  if (ret != 0 || whoamI != LIS3DH_ID)
  {
    return -1;
  }

  ret = lis3dh_block_data_update_set(&lis3dh_ctx, PROPERTY_ENABLE);
  ret |= lis3dh_data_rate_set(&lis3dh_ctx, LIS3DH_ODR_400Hz);
  ret |= lis3dh_full_scale_set(&lis3dh_ctx, LIS3DH_2g);
  ret |= lis3dh_operating_mode_set(&lis3dh_ctx, LIS3DH_HR_12bit); // High-res: 12 effective bits in 16-bit registers
  ret |= lis3dh_high_pass_on_outputs_set(&lis3dh_ctx, PROPERTY_DISABLE);
  if (ret != 0)
  {
    return ret;
  }

  return 0;
}

static int32_t LIS3DH_Read_Accel(void)
{
  axis3bit16_t data_raw_acceleration;
  uint8_t reg;

  do
  {
    if (lis3dh_xl_data_ready_get(&lis3dh_ctx, &reg) != 0)
    {
      return -1;
    }
  } while (!reg);

  if (lis3dh_acceleration_raw_get(&lis3dh_ctx, data_raw_acceleration.i16bit) != 0)
  {
    return -1;
  }

  const float mg_to_g = 1.0f / 1000.0f;
  Ax = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[0]) * mg_to_g;
  Ay = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[1]) * mg_to_g;
  Az = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[2]) * mg_to_g;

  return 0;
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
  /* USER CODE BEGIN 2 */

  LIS3DH_ContextInit();
  if (LIS3DH_Init() != 0)
  {
    const char *msg = "LIS3DH init failed\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  const uint32_t poll_interval_ms = 2U;

  while (1)
  {
    /* USER CODE END WHILE */

    if (LIS3DH_Read_Accel() == 0)
    {
      char buffer[80];
      int len = snprintf(buffer, sizeof(buffer),
                         "Ax: %.3f g, Ay: %.3f g, Az: %.3f g\r\n",
                         Ax, Ay, Az);
      HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
    }
    HAL_Delay(poll_interval_ms); // Poll sensor close to the 400 Hz ODR

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA12 */
  GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
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
