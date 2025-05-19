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
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"

#include "lcd.h"

const char keymap[4][4] = {
	    {'1', '2', '3', 'A'},
	    {'4', '5', '6', 'B'},
	    {'7', '8', '9', 'C'},
	    {'*', '0', '#', 'D'}
	};

void delayms(uint32_t dly)
{
  uint32_t i,j=0;
  for(i=0;i<dly;i++)
  for(j=0;j<16000;j++);
}

void gpio_keypad_init (void){
	RCC->AHB1ENR |= (1 << 2); // port c

	    // Enable pull-up resistors for PC4–PC7
	   GPIOC->PUPDR &= ~(0xFF << 8);     // Clear
	    GPIOC->PUPDR |=  (0x55 << 8);     // Pull-up (01)
}

char scan_keypad(void) {
    for (int row = 0; row < 4; row++) {
        GPIOC->ODR |= 0x0F;              // Set all rows HIGH
        GPIOC->ODR &= ~(1 << row);       // Pull current row LOW

        for (volatile int d = 0; d < 1000; d++); // short delay

        for (int col = 0; col < 4; col++) {
            if ((GPIOC->IDR & (1 << (col + 4))) == 0) {
                return keymap[row][col]; // Return mapped key
            }
        }
    }
    return 0; // No key pressed
}

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

void Task1(void *pv)
{
	 char passkey[5] = {0};
	    const char password[] = "127C";

	    LcdFxn(0, 0x01); // Clear screen
	    lprint(0x80, "Welcome");

	    while (1) {
	        char key = scan_keypad();
	        if (key == '*') {
	            LcdFxn(0, 0x01);
	            lprint(0x80, "Enter Passkey:");
	            memset(passkey, 0, sizeof(passkey));
	            int i = 0;

	            while (1) {
	                key = scan_keypad();
	                if (key) {
	                    if (key == '#') {
	                        passkey[i] = '\0';
	                        LcdFxn(0, 0x01);

	                        if (strcmp(passkey, password) == 0) {
	                            lprint(0x80, "Please come in");
	                        } else {
	                            lprint(0x80, "Wrong");
	                        }

	                        vTaskDelay(pdMS_TO_TICKS(1000));
	                        LcdFxn(0, 0x01);
	                        lprint(0x80, "Welcome");
	                        break;
	                    }

	                    if (i < 4) {
	                        passkey[i++] = key;
	                        lprint(0xC0 + i - 1, "*");
	                    }
	                    while (scan_keypad()); // Wait for release
	                    vTaskDelay(pdMS_TO_TICKS(100)); // Debounce
	                }

	                vTaskDelay(pdMS_TO_TICKS(10));
	            }
	        }

	        vTaskDelay(pdMS_TO_TICKS(50)); // Polling delay
	    }
}


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  LcdInit();
    gpio_keypad_init();
  /* USER CODE BEGIN 2 */

  xTaskCreate(Task1,"Task1", 128, NULL, 1,NULL);
//  xTaskCreate(Task2,"Task2", 128, NULL, 2,NULL);
//  xTaskCreate(Task3,"Task3", 128, NULL, 3,NULL);
//  xTaskCreate(Task4,"Task4", 128, NULL, 4,NULL);
  vTaskStartScheduler();


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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
