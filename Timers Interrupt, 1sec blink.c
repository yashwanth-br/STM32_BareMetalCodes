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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCC_BASE_ADDR 0x040023800
#define GPIOA_BASE_ADDR 0x040020000
#define GPIOD_BASE_ADDR 0x040020C00
#define EXTI_BASE_ADDR 0x040013C00
#define SYSCFG_BASE_ADDR 0x040013800
#define ARM_NVIC_ISERx_BASE_ADDR 0xE000E100
#define TIMER2_BASE_ADDR 0x40000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct
{
	volatile uint32_t RCC_CR;
	volatile uint32_t RCC_PLLCFGR;
	volatile uint32_t RCC_CFGR;
	volatile uint32_t RCC_CIR;
	volatile uint32_t RCC_AHB1RSTR;
	volatile uint32_t RCC_AHB2RSTR;
	volatile uint32_t RCC_Reserved1;
	volatile uint32_t RCC_Reserved2;
	volatile uint32_t RCC_APB1RSTR;
	volatile uint32_t RCC_APB2RSTR;
	volatile uint32_t RCC_Reserved3;
	volatile uint32_t RCC_Reserved4;
	volatile uint32_t RCC_AHB1ENR;
	volatile uint32_t RCC_AHB2ENR;
	volatile uint32_t RCC_Reserved5;
	volatile uint32_t RCC_Reserved6;
	volatile uint32_t RCC_APB1ENR;
	volatile uint32_t RCC_APB2ENR;
	volatile uint32_t RCC_Reserved7;
	volatile uint32_t RCC_Reserved8;
}RCC_Reg_struct;

typedef struct
{
  volatile uint32_t GPIOx_MODER;
  volatile uint32_t GPIOx_OTYPER;
  volatile uint32_t GPIOx_OSPEEDR;
  volatile uint32_t GPIOx_PUPDR;
  volatile uint32_t GPIOx_IDR;
  volatile uint32_t GPIOx_ODR;
  volatile uint32_t GPIOx_BSRR;
  volatile uint32_t GPIOx_LCKR;
  volatile uint32_t GPIOx_AFRL;
  volatile uint32_t GPIOx_AFRH;
}GPIO_Reg_struct;

typedef struct
{
  volatile uint32_t EXTI_IMR;
  volatile uint32_t EXTI_EMR;
  volatile uint32_t EXTI_RTSR;
  volatile uint32_t EXTI_FTSR;
  volatile uint32_t EXTI_SWIER;
  volatile uint32_t EXTI_PR;
}EXTI_Reg_struct;

typedef struct
{
  volatile uint32_t SYSCFG_MEMRMP;
  volatile uint32_t SYSCFG_PMC;
  volatile uint32_t SYSCFG_EXTICR1;
  volatile uint32_t SYSCFG_EXTICR2;
  volatile uint32_t SYSCFG_EXTICR3;
  volatile uint32_t SYSCFG_EXTICR4;
  volatile uint32_t SYSCFG_CMPCR;
}SYSCFG_Reg_Struct;

typedef struct
{
  volatile uint32_t NVIC_ISER0;
  volatile uint32_t NVIC_ISER1;
  volatile uint32_t NVIC_ISER2;
  volatile uint32_t NVIC_ISER3;
  volatile uint32_t NVIC_ISER4;
  volatile uint32_t NVIC_ISER5;
  volatile uint32_t NVIC_ISER6;
  volatile uint32_t NVIC_ISER7;
}NVIC_ISERx_Reg_struct;

typedef struct
{
  volatile uint32_t TIMx_CR1;
  volatile uint32_t TIMx_CR2;
  volatile uint32_t TIMx_SMCR;
  volatile uint32_t TIMx_DIER;
  volatile uint32_t TIMx_SR;
  volatile uint32_t TIMx_EGR;
  volatile uint32_t TIMx_CCMR1;
  volatile uint32_t TIMx_CCMR2;
  volatile uint32_t TIMx_CCER;
  volatile uint32_t TIMx_CNT;
  volatile uint32_t TIMx_PSC;
  volatile uint32_t TIMx_ARR;
  volatile uint32_t TIMx_CCR1;
}TIMER_Reg_struct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE BEGIN Init */
	((RCC_Reg_struct *) RCC_BASE_ADDR) -> RCC_CR |= (1 << 16); //Enable HSE
	while(!(((RCC_Reg_struct *) RCC_BASE_ADDR) -> RCC_CR & (1 << 17))); //Wait until HSE stable
	((RCC_Reg_struct *) RCC_BASE_ADDR) -> RCC_CFGR &= ~(3 << 0); //Select HSE for main clock
	((RCC_Reg_struct *) RCC_BASE_ADDR) -> RCC_CFGR |= (1 << 0); //Select HSE for main clock
	((RCC_Reg_struct *) RCC_BASE_ADDR) -> RCC_CFGR |= (10 << 4); //System clock divided by 8 (for 1Mhz -> 1us Time Period)
	((RCC_Reg_struct *) RCC_BASE_ADDR) -> RCC_APB2ENR |= (1 << 14); //Enable the System configuration Register

	((RCC_Reg_struct *) RCC_BASE_ADDR) -> RCC_AHB1ENR |= (1 << 3); //GPIOD Enable
	((RCC_Reg_struct *) RCC_BASE_ADDR) -> RCC_AHB1ENR |= (1 << 0); //GPIOB Enable
	((RCC_Reg_struct *) RCC_BASE_ADDR) -> RCC_APB1ENR |= (1 << 0); //Timer 2 Enable

	((GPIO_Reg_struct *) GPIOD_BASE_ADDR) -> GPIOx_MODER &= ~(3 << 24); //Set GPIO12 mode to General purpose output
	((GPIO_Reg_struct *) GPIOD_BASE_ADDR) -> GPIOx_MODER |= (1 << 24); //Set GPIO12 mode to General purpose output
	((GPIO_Reg_struct *) GPIOD_BASE_ADDR) -> GPIOx_OTYPER &= ~(1 << 12); //Set no pullup/down for GPIO12

	((TIMER_Reg_struct *) TIMER2_BASE_ADDR) -> TIMx_CR1 |= (1 << 0); //Enable Timer 2
	((TIMER_Reg_struct *) TIMER2_BASE_ADDR) -> TIMx_DIER |= (1 << 0); //Enable Timer 2 interrupt
	((TIMER_Reg_struct *) TIMER2_BASE_ADDR) -> TIMx_ARR = 1000000; //Set Auto Reload Register to 1000000, so 1000000 * 1us = 1sec
  ((NVIC_ISERx_Reg_struct *) ARM_NVIC_ISERx_BASE_ADDR) -> NVIC_ISER0 |= (1 << 28); //Enable the Timer Interrupt in Internal NVIC
	// NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void TIM2_IRQHandler()
{
   if(!((((GPIO_Reg_struct *) GPIOD_BASE_ADDR) -> GPIOx_ODR) & (1 << 12))) //Check whether the LED is not ON
    ((GPIO_Reg_struct *) GPIOD_BASE_ADDR) -> GPIOx_ODR |= (1 << 12); //Turn the LED ON
   else
    ((GPIO_Reg_struct *) GPIOD_BASE_ADDR) -> GPIOx_ODR &= ~(1 << 12); //Turn the LED OFF
	((TIMER_Reg_struct *) TIMER2_BASE_ADDR) -> TIMx_SR &= ~(1 << 0); //Reset the Timer Interrupt Pending Register
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
