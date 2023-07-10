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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "hmi_driver.h"
#include "hmi_user_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_LENGTH 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  uint8_t Counter_Freq = 1; // 4 pins
  uint8_t Selecter_Voltage = 1; // 3 pins
  uint8_t Output_Mode = 0; // 0 for 4 pins, 1 for 3 pins
  uint16_t ADC_1, ADC_2;
  uint16_t ADC_Value[ADC_BUFFER_LENGTH];
  // uint8_t i;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/** 按键按下标置宏
* 按键按下为高电平，设置 KEY_ON=1， KEY_OFF=0
* 若按键按下为低电平，把宏设置成KEY_ON=0 ，KEY_OFF=1 即可
*/
#define KEY_ON  1
#define KEY_OFF 0

/**
* @brief   检测是否有按键按下
* @param  GPIOx:具体的端口, x可以是（A...G）
* @param  GPIO_PIN:具体的端口位， 可以是GPIO_PIN_x（x可以是0...15）
* @retval  按键的状态
*     @arg KEY_ON:按键按下
*     @arg KEY_OFF:按键没按下
*/
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
    /*检测是否有按键按下 */
    if (HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON ) {
        /*等待按键释放 */
        while (HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON);
        return  KEY_ON;
    } else
        return KEY_OFF;
}
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}

void lissajous_figures(uint16 *xdata, uint16 *ydata, uint16 x0, uint16 y0, uint16 N)
{

  // for (int i = 0;i < N-2; i++)
  // {
  //   GUI_Line(xdata[i]/16 + x0, ydata[i]/16 + y0, xdata[i + 2]/16 + x0, ydata[i + 2]/16 + y0);
  // }
  for (int i = 0; i < N - 3; i += 2)
  {
    uint16 x_start = ADC_Value[i] / 16 + x0;
    uint16 y_start = ADC_Value[i + 1] / 16 + y0;
    // uint16 x_end = ADC_Value[i + 2] / 16 + x0;
    // uint16 y_end = ADC_Value[i + 3] / 16 + y0;
    // GUI_Line(x_start, y_start, x_end, y_end);
    GUI_Dot(x_start, y_start);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  * 需求：根据ADC_Value[ADC_BUFFER_LENGTH] 得到的一个数组，实现模拟xy模式双通道示波器画图。其中，x通道采样的值存放在ADC_Value[0],ADC_Value[2]这样的偶数下标位置，y通道采样得到数值存放在奇数下标位置。
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  TPL0401A_Init();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, ADC_BUFFER_LENGTH);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  SetBcolor(0x0000);
  SetFcolor(0x0000); // black
  GUI_RectangleFill(50,200,50+300,200+300);
  SetFcolor(0x07E0); // light green
	
  while (1)
  {
    if ((Key_Scan(GPIOA, GPIO_PIN_0) == KEY_ON) && (Output_Mode == 0))
    {
      Counter_Freq = (Counter_Freq + 1) % 5;
    }
    if ((Key_Scan(GPIOA, GPIO_PIN_0) == KEY_ON) && (Output_Mode == 1))
    {
      Selecter_Voltage = (Selecter_Voltage + 1) % 3;
    }
    if (Key_Scan(GPIOC, GPIO_PIN_13) == KEY_ON) // Key2, Set Mode
    {
      Output_Mode = 1 - Output_Mode;
    }
    switch (Counter_Freq)
    {
    case 1:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
      break;
    case 2:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
      break;
    case 3:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
      break;
    case 4:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
      break;
    case 5:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); 
      break;  
    }
    switch (Selecter_Voltage)
    {
    case 1:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);
      break;
    case 2:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);
      break;
    case 3:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);
    }
    // ADC_Value = malloc(ADC_BUFFER_LENGTH * sizeof(uint16_t));
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_Value, ADC_BUFFER_LENGTH);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); // LED_Blue
    // Set_TPL0401A_Value(0x7F);
    // printf("welcome to a new world! \r\n");
    // SetTextValue(0,1,(uchar *)"hello,world!");
    // SetTextInt32(0,5,ADC_Value[0],0,4);
    for (int i = 0; i < ADC_BUFFER_LENGTH;)
    {
        ADC_1 = ADC_Value[i++];
        ADC_2 = ADC_Value[i++];
    }
    // printf("  double channel ADC test\r\n");
    // printf("ADC_Value[0] is %d\r\n", ADC_Value[0]);
    // printf("ADC_Value[1] is %d\r\n", ADC_Value[1]);
    // printf("PC0 = %1.4f V\r\n", ADC_1*3.3f/4096);
    // printf("PC1 = %1.4f V\r\n", ADC_2*3.3f/4096);
    // MX_DMA_DeInit();
    // HAL_ADC_MspDeInit(&hadc1);
    HAL_ADC_Stop_DMA(&hadc1);
    // HAL_ADC_DeInit(&hadc1);
    // printf("ADC is stopped!\r\n\r\n");
    // for (int i = 0; i < ADC_BUFFER_LENGTH; i += 2)
    // {
    //   printf("ADC_Value[%4d] is %4d, ADC_Value[%4d] is %4d\r\n", i, ADC_Value[i], i + 1, ADC_Value[i + 1]);
    // }
    lissajous_figures(&ADC_Value[0],&ADC_Value[1],50,200,ADC_BUFFER_LENGTH);
    
    // free(ADC_Value);
    // void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
