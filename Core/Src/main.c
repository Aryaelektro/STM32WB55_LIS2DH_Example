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
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include "lis2dh12_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUS hi2c1
#define BUFFER_SIZE 128
char buffer[BUFFER_SIZE];  // to store strings..

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define  BOOT_TIME 5 //ms
#define  SAMPLES  500
#define  MAX_SENSOR_DATA 18
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
static int16_t data_raw_acceleration[3];
//static int16_t data_raw_temperature;
//static float acceleration_mg[3];
static float acceleration_mg[3];
//static float temperature_degC;
static uint8_t whoamI;
static char tx_buffer[20];
char *text;
FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
int i = 0;
char dummy[500];
char data_buffer[SAMPLES][MAX_SENSOR_DATA];  // 200 rows, each holding up to 18 characters
char sd_write_buffer[SAMPLES * MAX_SENSOR_DATA]; // A single buffer for all data (200 * 18 = 3600 bytes max)

size_t total_data_length = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static void platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void send_uart(char *string);
static void platform_delay(uint32_t ms);
void lis2dh12_read_data_polling(void);
static void initialize_sd_card();
void clear_buffer(void);
int bufsize(char *buf);
void write_to_sd(char *data, size_t total_size);  // Function prototype
//static void platform_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int32_t file, uint8_t *ptr, int32_t len)
{
  for (int i = 0; i < len; i++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

/* Main Example --------------------------------------------------------------*/
static void initialize_sd_card(void)
{
  // Mount SDCard
  send_uart("MOUNTING_SD_CARD\n");
  HAL_Delay(1000);

  fresult = f_mount(&fs, "", 1); /* Mode option 0:Do not mount (delayed mount), 1:Mount immediately */
  if (fresult != FR_OK)
  {
    send_uart("*************************************\n");
    send_uart("ERR_MOUNTING_SD_CARD\n");
    send_uart("*************************************\n");
    for (int x = 0; x < 6; x++)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
      HAL_Delay(500);
    }
  }
  else
  {
    send_uart("*************************************\n");
    send_uart("SUCC_MOUNTING_SD_CARD\n");
    send_uart("*************************************\n");
  }

  /*************** Card capacity details ********************/
  /*Checking free space*/
  f_getfree("", &fre_clust, &pfs);
  total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
  sprintf(buffer, "SD_CARD_TOTAL_SIZE: \t%lu\n", total);
  send_uart(buffer);
  clear_buffer();
  free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
  sprintf(buffer, "SD_CARD_FREE_SPACE: \t%lu\n", free_space);
  send_uart(buffer);
  clear_buffer();
}

void lis2dh12_read_data_polling(void)
{
  int counter = 0;
  int file_exists = 0;
  char filename[20];

  do
  {
    if (counter == 0)
    {
      snprintf(filename, sizeof(filename), "test.csv");
    }
    else
    {
      snprintf(filename, sizeof(filename), "test_%d.csv", counter);
    }

    if (f_open(&fil, filename, FA_OPEN_EXISTING) == FR_OK)
    {
      f_close(&fil);
      file_exists = 1;
      counter++;
    }
    else
    {
      file_exists = 0;
    }
  }
  while (file_exists);
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hi2c1;
  /* Wait boot time and initialize platform specific hardware */
  //  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lis2dh12_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LIS2DH12_ID)
  {
    while (1)
    {
      /* manage here device not found */
    }
  }
  /* Enable Block Data Update. */
  lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate to 1Hz. */
  lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_1Hz);
  /* Set full scale to 2g. */
  lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);
  /* Enable temperature sensor. */
  //  lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_ENABLE);
  /* Set device in continuous mode with 12 bit resol. */
  lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_HR_12bit);

  for (int i = 0; i < SAMPLES; i++)
  { //sampling 200 data = 2 detik (i.0n 100Hz)
    uint32_t start_time = HAL_GetTick(); //HAL_GetTick(); resolution in ms
    lis2dh12_reg_t reg;
    lis2dh12_xl_data_ready_get(&dev_ctx, &reg.byte);
    memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
    acceleration_mg[0] = lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration[0]);
    acceleration_mg[1] = lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration[1]);
    acceleration_mg[2] = lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration[2]);
    // Append formatted data directly to the single buffer (max 18 characters per entry)
    // Store formatted data in data_buffer (max 18 characters)
//    snprintf(
//        data_buffer[i],
//        sizeof(data_buffer[i]),
//        "%d,%d,%d\n",
//        (int) acceleration_mg[0],
//        (int) acceleration_mg[1],
//        (int) acceleration_mg[2]);
//
//    // Concatenate each line to sd_write_buffer and update total length
//    size_t line_length = strlen(data_buffer[i]);
//    memcpy(sd_write_buffer + total_data_length, data_buffer[i], line_length);
//    total_data_length += line_length;
    sprintf(
        (char*) tx_buffer,
        "%d,%d,%d\n",
        (int) acceleration_mg[0],
        (int) acceleration_mg[1],
        (int) acceleration_mg[2]);
//  send_uart((char*)tx_buffer);
//*****************************************************
    // Check if snprintf wrote successfully and there is enough space
//    if (written > 0 && (total_data_length + written) < sizeof(data_buffer))
//    {
//      total_data_length += written;  // Update total length of valid data
//    }
//    else
//    {
//      // Handle buffer overflow or snprintf failure
//      break;
//    }
//**************************************************************
    if (f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE) == FR_OK)
    {
      f_write(&fil, tx_buffer, bufsize(tx_buffer), 0);
      f_close(&fil);
    }

    // Wait until 10ms have passed
    while (HAL_GetTick() < start_time + 10)
    {
      //wait for 1ms
    }
  }
  sprintf(buffer, "finish polling sensor data and saved to %s\n", filename);
  send_uart(buffer);
}

// Function to write the global data_buffer to SD card
void write_to_sd(char *data, size_t total_size)
{

  int counter = 0;
  int file_exists = 0;
  char filename[100];

  do
  {
    if (counter == 0)
    {
      snprintf(filename, sizeof(filename), "result.csv");
    }
    else
    {
      snprintf(filename, sizeof(filename), "result_%d.csv", counter);
    }

    if (f_open(&fil, filename, FA_OPEN_EXISTING) == FR_OK)
    {
      f_close(&fil);
      file_exists = 1;
      counter++;
    }
    else
    {
      file_exists = 0;
    }
  }
  while (file_exists);

  if (f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE) == FR_OK)
  {
    fresult = f_write(&fil, data, total_size, &br);
    if (fresult != FR_OK || br != total_size)
    {
      // Handle write error
    }
    f_close(&fil);
  }
  else
  {
    // Handle file open error
  }
  /* Unmount SDCARD */
  fresult = f_mount(NULL, "", 0);
  if (fresult == FR_OK)
  {
    send_uart("SD CARD UNMOUNTED successfully...\n");
  }
  memset(data_buffer, 0, sizeof(data_buffer));
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static void platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  reg |= 0x80;
  HAL_I2C_Mem_Write(handle, LIS2DH12_I2C_ADD_L, reg,
  I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
}

static void platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(handle, LIS2DH12_I2C_ADD_L, reg,
  I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
}

void send_uart(char *string)
{
  uint8_t len = strlen(string);
  HAL_UART_Transmit(&huart1, (uint8_t*) string, len, 1000);
}

static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

int bufsize(char *buf)
{
  int i = 0;
  while (*buf++ != '\0')
    i++;
  return i;
}

void clear_buffer(void)
{
  for (int i = 0; i < BUFFER_SIZE; i++)
    buffer[i] = '\0';
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  if (MX_FATFS_Init() != APP_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN 2 */
  //  logging_sdcard("################\n");
  initialize_sd_card();
  lis2dh12_read_data_polling();
//  write_to_sd(sd_write_buffer, total_data_length);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    lis2dh12_read_data_polling();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE
      | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4 | RCC_CLOCKTYPE_HCLK2 | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
   */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
  hi2c1.Init.Timing = 0x00B07CB4;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin | LD3_Pin | LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin | LD3_Pin | LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin | B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
