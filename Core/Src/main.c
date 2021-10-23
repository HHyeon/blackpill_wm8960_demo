/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"

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

I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_tx;

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE_PUT int __io_putchar(int ch)
#define PUTCHAR_PROTOTYPE_GET int __io_getchar(int ch)
#else
#define PUTCHAR_PROTOTYPE_PUT int fputc(int ch, FILE *f)
#define PUTCHAR_PROTOTYPE_GET int fgetc(FILE *f)
#endif 

PUTCHAR_PROTOTYPE_PUT
{
  if(ch == '\n')
  {
    ch = '\r';
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 1000);
    ch = '\n';
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 1000);
  }
  else
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 1000);
  return ch;
}

PUTCHAR_PROTOTYPE_GET
{
  uint8_t ch;
  while( HAL_UART_Receive(&huart1, (uint8_t*)&ch, 1, 1000) != HAL_OK);
  return ch;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2S1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

enum
{
  I2S_BUFFER_SELECT_A=0,
  I2S_BUFFER_SELECT_B
};

#define i2s_buffer_size 10240
uint8_t i2s_buffer_a[i2s_buffer_size];
uint8_t i2s_buffer_b[i2s_buffer_size];
uint8_t i2s_buffer_switch = I2S_BUFFER_SELECT_A;

uint8_t i2s_buffer_txcplt = 0;
uint8_t i2s_buffer_txhfcplt = 0;

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if(hi2s->Instance == hi2s1.Instance)
  {
    i2s_buffer_txhfcplt = 1;
  }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if(hi2s->Instance == hi2s1.Instance)
  {
    i2s_buffer_txcplt = 1;
  }  
}

#define WM8960_ADDRESS  0x1a

void wm8960_write_reg(uint8_t addr, uint16_t data)
{
//  addr<<=1;
//  addr |= ((data>>8)&0x0001);
//  if(HAL_I2C_Mem_Write(&hi2c1, WM8960_ADDRESS, addr, 1, (uint8_t*)(data&0xff), 1, 1000) != HAL_OK)
//    printf("wm8960 write failed\n");
  
  uint8_t I2C_Data[2];
  
  I2C_Data[0] = (addr<<1)|((uint8_t)((data>>8)&0x0001));  //RegAddr
  I2C_Data[1] = (uint8_t)(data&0x00FF);                  //RegValue
  
  if(HAL_I2C_Master_Transmit(&hi2c1,(WM8960_ADDRESS<<1),I2C_Data,2,1000) != HAL_OK)
    printf("wm8960 write failed\n");
  
}


void wm8960_initial()
{
  wm8960_write_reg(0x0f, 0x00);  
  wm8960_write_reg(0x19, 1<<8 | 1<<7 | 1<<6);
  wm8960_write_reg(0x1A, 1<<8 | 1<<7 | 1<<6 | 1<<5 | 1<<4 | 1<<3);
  wm8960_write_reg(0x2F, 1<<3 | 1<<2);
  
  //Configure clock
  //MCLK->div1->SYSCLK->DAC/ADC sample Freq = 25MHz(MCLK)/2*256 = 48.8kHz
  wm8960_write_reg(0x04, 0x0000);
  
  //Configure ADC/DAC
  wm8960_write_reg(0x05, 0x0000);
  
  //Configure audio interface
  //I2S format 16 bits word length
  wm8960_write_reg(0x07, 0x0002);
  
  //Configure HP_L and HP_R OUTPUTS
  wm8960_write_reg(0x02, 0x006F | 0x0100);  //LOUT1 Volume Set
  wm8960_write_reg(0x03, 0x006F | 0x0100);  //ROUT1 Volume Set
  
  //Configure SPK_RP and SPK_RN
  wm8960_write_reg(0x28, 0x007F | 0x0100); //Left Speaker Volume
  wm8960_write_reg(0x29, 0x007F | 0x0100); //Right Speaker Volume
  
  //Enable the OUTPUTS
  wm8960_write_reg(0x31, 0x00F7); //Enable Class D Speaker Outputs
  
  //Configure DAC volume
  wm8960_write_reg(0x0a, 0x00FF | 0x0100);
  wm8960_write_reg(0x0b, 0x00FF | 0x0100);
  
  //3D
//  wm8960_write_reg(0x10, 0x001F);
  
  //Configure MIXER
  wm8960_write_reg(0x22, 1<<8 | 1<<7);
  wm8960_write_reg(0x25, 1<<8 | 1<<7);
  
  //Jack Detect
  wm8960_write_reg(0x18, 1<<6 | 0<<5);
  wm8960_write_reg(0x17, 0x01C3);
  wm8960_write_reg(0x30, 0x0009);//0x000D,0x0005
}


char sdfilepath[128];
uint32_t filetotalsz=0;
uint32_t readbytes;

FRESULT res;
DIR dir;
FILINFO fno;
FIL wavfile;
FRESULT res;

uint8_t play_ended = 0;


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
  MX_USART1_UART_Init();
  MX_I2S1_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_DMA_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  printf("F411CE Started\n");
  
  wm8960_initial();

  retSD = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
  
  if(retSD != FR_OK)
  {
    printf("mount failed\n");
  }
  else
  {
    
    res = f_opendir(&dir, SDPath);
    if(res == FR_OK) {
      while(1)
      {
        res = f_readdir(&dir, &fno);
        if(res != FR_OK) break;
        if(fno.fname[0] == 0) break;
        
        printf("%s\n", fno.fname);
      }
      f_closedir(&dir);
      
    }
    else
    {
      printf("drive open failed\n");
    }
  }
  strcpy(sdfilepath, "0:/BILLEV~1.WAV");
  printf("opening %s\n", sdfilepath);
  
  
  
  
//  res = f_open(&wavfile, (TCHAR *)sdfilepath, FA_READ);
//  
//  filetotalsz=0;
//  if(res == FR_OK)
//  {
//    res = f_read(&wavfile, i2s_buffer_a, i2s_buffer_size, &readbytes);
//    printf("\rread %5d bytes / total %8d", readbytes, filetotalsz);
//    
//    
//    do
//    {
//      i2s_buffer_txcplt = 0;
//      HAL_I2S_Transmit_DMA(&hi2s1,(uint16_t*) i2s_buffer_a, readbytes);
//      
//      res = f_read(&wavfile, i2s_buffer_b, i2s_buffer_size, &readbytes);
//      filetotalsz += readbytes;
//      printf("\rread %5d bytes / total %8d", readbytes, filetotalsz);
//      
//      while(i2s_buffer_txcplt == 0);
//      
//      i2s_buffer_txcplt = 0;
//      HAL_I2S_Transmit_DMA(&hi2s1,(uint16_t*) i2s_buffer_b, readbytes);
//      
//      res = f_read(&wavfile, i2s_buffer_a, i2s_buffer_size, &readbytes);
//      filetotalsz += readbytes;
//      printf("\rread %5d bytes / total %8d", readbytes, filetotalsz);
//      
//      while(i2s_buffer_txcplt == 0);
//      
//    } while(readbytes == i2s_buffer_size);
//    
//    printf("\n");
//    
//    printf("total file size %d\n", filetotalsz);
//   
//    f_close(&wavfile);
//    
//    successed_cnt++;
//  }
//  else
//  {
//    printf("open failed %s\n", sdfilepath);
//  }
//  
//  while(1);
  
  
/*
  play_ended = 0;
  filetotalsz=0;
  
  res = f_open(&wavfile, (TCHAR *)sdfilepath, FA_READ);
  if(res == FR_OK)
  {
    f_read(&wavfile, i2s_buffer_a, i2s_buffer_size, &readbytes);
    filetotalsz += readbytes;
    
    i2s_buffer_txcplt = 0;
    HAL_I2S_Transmit_IT(&hi2s1,(uint16_t*) i2s_buffer_a, readbytes);
    
    f_read(&wavfile, i2s_buffer_b, i2s_buffer_size, &readbytes);
    i2s_buffer_switch = I2S_BUFFER_SELECT_B;
    filetotalsz += readbytes;
  }
  else
  {
    printf("not found %s\n", sdfilepath);
  }
  */
  
  i2s_buffer_txcplt=0;
  HAL_I2S_Transmit_DMA(&hi2s1,(uint16_t*) i2s_buffer_a, i2s_buffer_size);
  
  
  uint32_t tick=0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    if(HAL_GetTick() - tick > 1000)
    {
      tick = HAL_GetTick();
      
      HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    }
    
    if(i2s_buffer_txcplt)
    {
      i2s_buffer_txcplt = 0;
      
//      if(play_ended == 0)
//      {
//        
//        if(i2s_buffer_switch == I2S_BUFFER_SELECT_A)
//        {
//          i2s_buffer_txcplt = 0;
//          HAL_I2S_Transmit_IT(&hi2s1,(uint16_t*) i2s_buffer_a, readbytes);
//          
//          if(readbytes != i2s_buffer_size)
//          {
//            play_ended = 1;
//            printf("play ended\n");
//          }
//          
//          f_read(&wavfile, i2s_buffer_b, i2s_buffer_size, &readbytes);
//          i2s_buffer_switch = I2S_BUFFER_SELECT_B;
//          filetotalsz += readbytes;
//        }
//        else
//        {
//          i2s_buffer_txcplt = 0;
//          HAL_I2S_Transmit_IT(&hi2s1,(uint16_t*) i2s_buffer_b, readbytes);
//          
//          if(readbytes != i2s_buffer_size)
//          {
//            play_ended = 1;
//            printf("play ended\n");
//          }
//          
//          f_read(&wavfile, i2s_buffer_a, i2s_buffer_size, &readbytes);
//          i2s_buffer_switch = I2S_BUFFER_SELECT_A;
//          filetotalsz += readbytes;
//        }
//        
//        printf("\r%8d", filetotalsz);
//      }
      
      i2s_buffer_txcplt=0;
      HAL_I2S_Transmit_DMA(&hi2s1,(uint16_t*) i2s_buffer_a, i2s_buffer_size);
      
      printf("full cplt\n");
    }
    
    if(i2s_buffer_txhfcplt)
    {
      i2s_buffer_txhfcplt = 0;
      
      printf("half cplt\n");
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDCard_Detect_Pin */
  GPIO_InitStruct.Pin = SDCard_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDCard_Detect_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
