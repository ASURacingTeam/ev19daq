
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#define pi 3.1415
#define radiusOfWheel .5
#define clkFreq 16000000

uint16_t frontRightFlag =0 ;			  // flags to make sure channel enterrupt has been happened 
uint16_t frontLeftFlag =0 ;
uint16_t rearRightFlag =0 ;
uint16_t rearLeftFlag =0 ;

uint16_t frontRightInputCapture =0 ;  // variable to store counter of timer 1 value
uint16_t frontLeftInputCapture =0 ;	 // variable to store counter of timer 2 value
uint16_t rearRightInputCapture =0 ;	 // variable to store counter of timer 3 value
uint16_t rearLeftInputCapture =0 ;	 // variable to store counter of timer 4 value

float frontRightWheelSpeed =0 ;		// speed of front Right Wheel
float frontLeftWheelSpeed =0 ;  // speed of front left Wheel
float rearRightWheelSpeed =0 ;		// speed of rear Right Wheel
float rearLeftWheelSpeed =0 ;		// speed of rear left Wheel


uint16_t carAvarageSpeed =0 ;   		// avarage speed of the car 


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)  
{
 if (htim->Instance==TIM1)
  {
		// i search for the code and i will add it but he will be in the form of this below 
		/*if channel 1 interrupted  
			frontRightFlag =1 ; 
		
		if channel 2 interrupted 
			frontleftFlag = 1 ;
		
		if channel 3 interrupted  
			rearRightFlag =1 ; 

		if channel 4 interrupted  
			rearLeftFlag =1 ; 		
		
		*/
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t frontRightDuration =0 ;		// duration of 1/4 of front Right Wheel
	uint32_t frontLeftDuration =0 ;			// duration of 1/4 of front left Wheel
	uint32_t rearRightDuration =0 ;			// duration of 1/4 of rear Right Wheel
	uint32_t rearleftDuration =0 ;			// duration of 1/4 of rear left Wheel

	uint8_t frontRightWheelCheck =0; 		// variable to check if the steps of calculation of the front right wheel duration have been done or not 
	uint8_t frontLeftWheelCheck =0; 		// variable to check if the steps of calculation of the front left Wheel duration have been done or not 
	uint8_t rearRightWheelCheck =0; 		// variable to check if the steps of calculation of the rear Right Wheel duration have been done or not 
	uint8_t rearLeftWheelCheck =0; 			// variable to check if the steps of calculation of the rear left Wheel duration have been done or not 
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);  
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);  
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);  
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);  

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	if (frontRightFlag == 1){
		frontRightInputCapture = (__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_1)- frontRightInputCapture );    //read TIM1 channel 1 capture value
		frontRightDuration = ( frontRightInputCapture / clkFreq ) ;    // duration in seconds 
		frontRightWheelSpeed = (((60 /(4*frontRightDuration)) *(2*pi/60)) * radiusOfWheel ) ; // 60 to convert to min,*4 to make it for one revolution ,2pi/60 to convert to rad/sec, *raduis to convert to m/sec
		frontRightFlag =0 ;
		frontRightWheelCheck =1 ;  // change 	front Right Wheel Check value to 1 to make sure the calculations have been done 
	}
	if (frontLeftFlag == 1){
		frontLeftInputCapture= (__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_2)- frontLeftInputCapture);    //read TIM1 channel 1 capture value
		frontLeftDuration = (frontLeftInputCapture / clkFreq) ;    // duration in seconds 
		frontLeftWheelSpeed = (((60 /(4*frontLeftDuration)) *(2*pi/60)) * radiusOfWheel ) ; // 60 to convert to min,*4 to make it for one revolution, 2pi/60 to convert to rad/sec, *raduis to convert to m/sec
		frontLeftFlag =0 ;
		frontLeftWheelCheck = 1 ;  // change 	front Left Wheel Check value to 1 to make sure the calculations have been done 
	}
	if (rearRightFlag == 1){
		rearRightInputCapture= (__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_3)- rearRightInputCapture);    //read TIM1 channel 1 capture value
		rearRightDuration = (rearRightInputCapture / clkFreq);    // duration in seconds 
		rearRightWheelSpeed = (((60 /(4*rearRightDuration)) *(2*pi/60)) * radiusOfWheel ) ;// 60 to convert to min,*4 to make it for one revolution, 2pi/60 to convert to rad/sec, *raduis to convert to m/sec
		rearRightFlag =0 ; 
		rearRightWheelCheck =1;  // change 	rear Right Wheel Check value to 1 to make sure the calculations have been done 
	}
	if (rearLeftFlag== 1){
		rearLeftInputCapture= (__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_4)- rearLeftInputCapture);    //read TIM1 channel 1 capture value
		rearleftDuration = (rearLeftInputCapture / clkFreq) ; 		  // duration in seconds 
		rearLeftWheelSpeed = (((60 /(4*rearleftDuration)) *(2*pi/60)) * radiusOfWheel ) ;// 60 to convert to min,*4 to make it for one revolution, 2pi/60 to convert to rad/sec, *raduis to convert to m/sec
		rearLeftFlag = 0;
		rearLeftWheelCheck = 1 ;  // change rear Left Wheel Check value to 1 to make sure the calculations have been done 		
	}
	if ((frontRightWheelCheck & frontLeftWheelCheck & rearRightWheelCheck & rearLeftWheelCheck)== 1){
	carAvarageSpeed = ((frontRightWheelSpeed + frontLeftWheelSpeed + rearRightWheelSpeed + rearLeftWheelSpeed )/4) ; // avarage speed of the car 
  frontRightWheelCheck = 0 ;   // return the value of check to 0 to be aware of changing the value when the calculations happen
	frontLeftWheelCheck = 0 ;		 // return the value of check to 0 to be aware of changing the value when the calculations happen
	rearRightWheelCheck = 0 ;		 // return the value of check to 0 to be aware of changing the value when the calculations happen
	rearLeftWheelCheck = 0 ;		 // return the value of check to 0 to be aware of changing the value when the calculations happen
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
