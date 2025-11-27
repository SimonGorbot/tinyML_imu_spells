/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body for NanoEdge AI Inference with MPU9250
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
#include "driver_mpu9250.h"
#include "driver_mpu9250_interface.h"
#include "NanoEdgeAI.h"
#include "knowledge.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU9250_ADDR MPU9250_ADDRESS_AD0_LOW
#define MPU9250_REG_WHO_AM_I 0x75U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* NanoEdge AI variables */
uint16_t id_class = 0; 
float input_user_buffer[DATA_INPUT_USER * AXIS_NUMBER]; // Buffer of input values
float output_class_buffer[CLASS_NUMBER]; // Buffer of class probabilities
const char *id2class[CLASS_NUMBER + 1] = { // Buffer for mapping class id to class name
	"unknown",
	"verticalshake",
	"horizontalshake",
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void MPU9250_Init(void);
static void MPU9250_Print_WhoAmI(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static mpu9250_handle_t s_mpu9250_handle;

static void MPU9250_Print_WhoAmI(void)
{
  uint8_t who_am_i = 0U;

  (void)mpu9250_interface_iic_init();
  if (mpu9250_interface_iic_read(MPU9250_ADDR,
                                 MPU9250_REG_WHO_AM_I,
                                 &who_am_i,
                                 1U) != 0U)
  {
    const char *msg = "MPU9250: WHO_AM_I read failed\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

    return;
  }

  char buffer[64];
  int len = snprintf(buffer,
                     sizeof(buffer),
                     "MPU9250 WHO_AM_I = 0x%02X\r\n",
                     who_am_i);
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
}

static void MPU9250_Init(void)
{
  DRIVER_MPU9250_LINK_INIT(&s_mpu9250_handle, mpu9250_handle_t);
  DRIVER_MPU9250_LINK_IIC_INIT(&s_mpu9250_handle, mpu9250_interface_iic_init);
  DRIVER_MPU9250_LINK_IIC_DEINIT(&s_mpu9250_handle, mpu9250_interface_iic_deinit);
  DRIVER_MPU9250_LINK_IIC_READ(&s_mpu9250_handle, mpu9250_interface_iic_read);
  DRIVER_MPU9250_LINK_IIC_WRITE(&s_mpu9250_handle, mpu9250_interface_iic_write);
  DRIVER_MPU9250_LINK_SPI_INIT(&s_mpu9250_handle, mpu9250_interface_spi_init);
  DRIVER_MPU9250_LINK_SPI_DEINIT(&s_mpu9250_handle, mpu9250_interface_spi_deinit);
  DRIVER_MPU9250_LINK_SPI_READ(&s_mpu9250_handle, mpu9250_interface_spi_read);
  DRIVER_MPU9250_LINK_SPI_WRITE(&s_mpu9250_handle, mpu9250_interface_spi_write);
  DRIVER_MPU9250_LINK_DELAY_MS(&s_mpu9250_handle, mpu9250_interface_delay_ms);
  DRIVER_MPU9250_LINK_DEBUG_PRINT(&s_mpu9250_handle, mpu9250_interface_debug_print);
  DRIVER_MPU9250_LINK_RECEIVE_CALLBACK(&s_mpu9250_handle, mpu9250_interface_receive_callback);

  if (mpu9250_set_interface(&s_mpu9250_handle, MPU9250_INTERFACE_IIC) != 0 ||
      mpu9250_set_addr_pin(&s_mpu9250_handle, MPU9250_ADDR) != 0)
  {
    const char *msg = "MPU9250: interface config failed\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    Error_Handler();
  }

  if (mpu9250_init(&s_mpu9250_handle) != 0)
  {
    const char *msg = "MPU9250: init failed\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    Error_Handler();
  }

  /* Configuration for 100 Hz Sample Rate (consistent with NEAI typically) */
  /* 1 kHz / (1 + 9) = 100 Hz */
  (void)mpu9250_set_sample_rate_divider(&s_mpu9250_handle, 9);
  (void)mpu9250_set_low_pass_filter(&s_mpu9250_handle, MPU9250_LOW_PASS_FILTER_3);
  (void)mpu9250_set_accelerometer_range(&s_mpu9250_handle, MPU9250_ACCELEROMETER_RANGE_2G);
  (void)mpu9250_set_gyroscope_range(&s_mpu9250_handle, MPU9250_GYROSCOPE_RANGE_250DPS);
  
  // Note: Magnetometer init kept if needed, but NEAI usually just uses accel
  if (mpu9250_mag_init(&s_mpu9250_handle) != 0)
  {
    const char *msg = "MPU9250: mag init failed\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    Error_Handler();
  }
  (void)mpu9250_mag_set_bits(&s_mpu9250_handle, MPU9250_MAGNETOMETER_BITS_16);
  (void)mpu9250_mag_set_mode(&s_mpu9250_handle, MPU9250_MAGNETOMETER_MODE_CONTINUOUS2);
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
  
  // Initialize MPU9250
  MPU9250_Print_WhoAmI();
  MPU9250_Init();
  
  // Initialize NanoEdge AI
  enum neai_state neai_res = neai_classification_init(knowledge);
  if (neai_res != NEAI_OK) {
      char err_buf[50];
      snprintf(err_buf, sizeof(err_buf), "NEAI Init Error: %d\r\n", neai_res);
      HAL_UART_Transmit(&huart2, (uint8_t *)err_buf, strlen(err_buf), HAL_MAX_DELAY);
  } else {
      const char *msg = "NEAI Init Success\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  // Buffers for MPU9250 reading
  int16_t accel_raw[1][3];
  float accel_g[1][3];
  int16_t gyro_raw[1][3];
  float gyro_dps[1][3];
  int16_t mag_raw[1][3];
  float mag_ut[1][3];
  uint16_t len = 1;
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // 1. Fill the Input Buffer
    // We need to collect DATA_INPUT_USER samples (defined as 19 in NanoEdgeAI.h)
    for (int i = 0; i < DATA_INPUT_USER; i++) {
        if (mpu9250_read(&s_mpu9250_handle,
                         accel_raw,
                         accel_g,
                         gyro_raw,
                         gyro_dps,
                         mag_raw,
                         mag_ut,
                         &len) != 0 || len == 0)
        {
             // Handle read error
             continue;
        }
        
        // Convert RAW int16_t to float for the buffer
        // Note: Using RAW values because data collection exports RAW
        input_user_buffer[AXIS_NUMBER * i]     = (float)accel_raw[0][0];
        input_user_buffer[AXIS_NUMBER * i + 1] = (float)accel_raw[0][1];
        input_user_buffer[AXIS_NUMBER * i + 2] = (float)accel_raw[0][2];
        
        // Wait 10ms to match the 100Hz ODR configured in Init
        HAL_Delay(10); 
    }

    // 2. Perform Classification
    neai_classification(input_user_buffer, output_class_buffer, &id_class);

    // 3. Output Result
    char out_buf[64];
    // Print class name from the id2class mapping
    int printed_len = snprintf(out_buf, sizeof(out_buf), "Class detected: %s (Prob: %.2f)\r\n", 
             id2class[id_class], output_class_buffer[id_class]);
    HAL_UART_Transmit(&huart2, (uint8_t *)out_buf, (uint16_t)printed_len, HAL_MAX_DELAY);
    
    // Optional: Toggle LED to show activity
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
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
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */