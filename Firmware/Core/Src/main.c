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
#include "mlp_model.h"
#include "mlp_scaler.h"
#include "driver_mpu9250.h"
#include "driver_mpu9250_interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Last IMU sample read (for convenience/logging)
float Ax, Ay, Az, Gx, Gy, Gz;

// We are using 3 accel + 3 gyro axes
#define AXIS_NUMBER 6U

// MLP model feature dimension: 100 samples * 6 axes
#define MLP_FEATURE_DIM 600U

/* USER CODE END PD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MPU9250_ADDR MPU9250_ADDRESS_AD0_HIGH
#define MPU9250_REG_WHO_AM_I 0x75U
#define MAX_RAW_SAMPLES 1000U
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Raw data buffer: up to MAX_RAW_SAMPLES samples, each with AXIS_NUMBER channels.
// Layout: [sample0_ax, sample0_ay, ..., sample0_gz, sample1_ax, ...]
float raw_data[MAX_RAW_SAMPLES * AXIS_NUMBER];
uint16_t raw_count = 0;

// Feature buffer for the MLP model
float mlp_features[MLP_FEATURE_DIM];

// Class label mapping for model outputs [0..5]
static const char *mlp_class_names[6] = {
    "circle",
    "downup",
    "leftright",
    "lightning",
    "rightleft",
    "updown",
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void normalize_buffer(float *source, uint16_t source_len, float *dest, uint16_t dest_len);
static void MPU9250_Init(void);
static void MPU9250_Print_WhoAmI(void);
static uint8_t MPU9250_ReadRaw(void);

// Build the MLP feature vector from the recorded raw IMU samples
static void build_mlp_features(const float *source, uint16_t source_len,
                               float *dest, uint16_t dest_len);

// Run the MLP classifier on the latest recording and print result over UART
static void classify_with_mlp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static mpu9250_handle_t s_mpu9250_handle;

static void normalize_buffer(float *source, uint16_t source_len, float *dest, uint16_t dest_len)
{
  if (source_len == 0 || dest_len == 0)
    return;

  for (int i = 0; i < dest_len; i++)
  {
    // Calculate the floating point index in the source array
    float pos = (float)i * (source_len - 1) / (dest_len - 1);
    int idx = (int)pos;
    float frac = pos - idx;

    // Interpolate for all axes
    for (int axis = 0; axis < AXIS_NUMBER; axis++)
    {
      float val0 = source[idx * AXIS_NUMBER + axis];
      float val1 = val0;
      if ((idx + 1) < source_len)
      {
        val1 = source[(idx + 1) * AXIS_NUMBER + axis];
      }

      dest[i * AXIS_NUMBER + axis] = val0 + frac * (val1 - val0);
    }
  }
}

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

  /* Example configuration: 1 kHz / (1 + div) = 166 Hz (matches 906 samples ≈5.4 s) */
  (void)mpu9250_set_sample_rate_divider(&s_mpu9250_handle, 5);
  (void)mpu9250_set_low_pass_filter(&s_mpu9250_handle, MPU9250_LOW_PASS_FILTER_3);
  (void)mpu9250_set_accelerometer_range(&s_mpu9250_handle, MPU9250_ACCELEROMETER_RANGE_2G);
  (void)mpu9250_set_gyroscope_range(&s_mpu9250_handle, MPU9250_GYROSCOPE_RANGE_250DPS);
}

static uint8_t MPU9250_ReadRaw(void)
{
  int16_t accel_raw[1][3];
  float accel_g[1][3];
  int16_t gyro_raw[1][3];
  float gyro_dps[1][3];
  int16_t mag_raw[1][3];
  float mag_ut[1][3];
  uint16_t len = 1;

  if (mpu9250_read(&s_mpu9250_handle,
                   accel_raw,
                   accel_g,
                   gyro_raw,
                   gyro_dps,
                   mag_raw,
                   mag_ut,
                   &len) != 0 ||
      len == 0)
  {
    return 1; // Error
  }

  // Use raw integer samples to stay consistent with NanoEdge training data
  Ax = (float)accel_raw[0][0];
  Ay = (float)accel_raw[0][1];
  Az = (float)accel_raw[0][2];
  Gx = (float)gyro_raw[0][0];
  Gy = (float)gyro_raw[0][1];
  Gz = (float)gyro_raw[0][2];

  return 0; // Success
}

static void build_mlp_features(const float *source, uint16_t source_len,
                               float *dest, uint16_t dest_len)
{
  // The training data uses 100 samples * 6 axes flattened to float (600 columns).
  const uint16_t target_samples = 100U;
  float resampled[target_samples * AXIS_NUMBER];

  // Clamp source_len to available buffer and avoid div-by-zero
  if (source_len == 0)
  {
    memset(dest, 0, dest_len * sizeof(float));
    return;
  }
  uint16_t used_samples = source_len;
  if (used_samples > MAX_RAW_SAMPLES)
  {
    used_samples = MAX_RAW_SAMPLES;
  }

  normalize_buffer((float *)source, used_samples, resampled, target_samples);

  uint32_t total_feats = target_samples * AXIS_NUMBER;
  if (total_feats > dest_len)
  {
    total_feats = dest_len; // truncate extra features
  }

  // Copy and scale to match Python StandardScaler used in training
  for (uint32_t i = 0; i < total_feats; i++)
  {
    float centered = resampled[i] - MLP_MEAN[i];
    float scale = MLP_SCALE[i];
    if (scale != 0.0f)
    {
      dest[i] = centered / scale;
    }
    else
    {
      dest[i] = centered;
    }
  }

  // Zero any unused tail
  for (uint32_t i = total_feats; i < dest_len; i++)
  {
    dest[i] = 0.0f;
  }
}

static void classify_with_mlp(void)
{
  if (raw_count == 0)
  {
    // Nothing recorded
    const char *msg = "MLP: no samples recorded, skipping classification\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    return;
  }

  // Build feature vector for the MLP model
  // source_len here is "raw_count" samples, each of AXIS_NUMBER floats
  build_mlp_features(raw_data, raw_count, mlp_features, MLP_FEATURE_DIM);

  // --- Measure Inference Time START ---
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0; // Reset cycle counter to 0

  // Run the MLP model (6-class classifier)
  float probs[6] = {0};
  const int n_classes = (int)(sizeof(probs) / sizeof(probs[0]));
  int32_t err = mlp_perf__32_32__1e_05_regress(mlp_features, MLP_FEATURE_DIM, probs, n_classes);

  uint32_t cycle_count = DWT->CYCCNT; // Read cycle counter
  float inference_time_us = (float)cycle_count * 1000000.0f / HAL_RCC_GetHCLKFreq();

  int32_t cls = -1;
  float prob = 0.0f;
  if (err == 0)
  {
    for (int i = 0; i < n_classes; i++)
    {
      if (probs[i] > prob)
      {
        prob = probs[i];
        cls = i;
      }
    }
  }

  const char *label = "out_of_range";
  if (cls >= 0 && cls < n_classes)
  {
    label = mlp_class_names[cls];
  }

  char buffer[128];
  int len = snprintf(buffer, sizeof(buffer),
                     "Inference: %.2f us | Class: %s (Prob: %.2f)\r\n",
                     inference_time_us, label, prob);
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
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
  MPU9250_Print_WhoAmI();
  MPU9250_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  const uint32_t poll_interval_ms = 5U;
  GPIO_PinState btn_prev = GPIO_PIN_SET; // Assume button is initially released
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Read the current state of the User Button (B1)
    // B1 is active Low (GPIO_PIN_RESET when pressed)
    GPIO_PinState btn_curr = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

    // 1. Detect the start of a press (Falling Edge: Released -> Pressed)
    if (btn_prev == GPIO_PIN_SET && btn_curr == GPIO_PIN_RESET)
    {
      char *header = "Recording...\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)header, strlen(header), HAL_MAX_DELAY);
      raw_count = 0; // Reset counter for new recording
    }

    // 2. Collect data while the button is held down
    if (btn_curr == GPIO_PIN_RESET)
    {
      if (raw_count < MAX_RAW_SAMPLES)
      {
        if (MPU9250_ReadRaw() == 0)
        {
          // Store raw values
          raw_data[raw_count * AXIS_NUMBER + 0] = Ax;
          raw_data[raw_count * AXIS_NUMBER + 1] = Ay;
          raw_data[raw_count * AXIS_NUMBER + 2] = Az;
          raw_data[raw_count * AXIS_NUMBER + 3] = Gx;
          raw_data[raw_count * AXIS_NUMBER + 4] = Gy;
          raw_data[raw_count * AXIS_NUMBER + 5] = Gz;
          raw_count++;
        }
      }
    }

    // 3. Detect the end of a press (Rising Edge: Pressed -> Released)
    if (btn_prev == GPIO_PIN_RESET && btn_curr == GPIO_PIN_SET)
    {
      if (raw_count > 0)
      {
        // Run MLP classifier on the recorded gesture
        classify_with_mlp();

        // Optionally clear raw_count so the next gesture starts fresh
        // raw_count = 0;
      }
    }

    // Update previous state for the next iteration
    btn_prev = btn_curr;

    // Poll delay (acts as a simple debounce)
    HAL_Delay(poll_interval_ms);
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
