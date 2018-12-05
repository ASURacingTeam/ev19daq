
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
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

#define __HAL_TIM_GetCompare            __HAL_TIM_GET_COMPARE
#define __HAL_TIM_GET_COMPARE(__HANDLE__, __CHANNEL__) \
  (*(__IO uint32_t *)(&((__HANDLE__)->Instance->CCR1) + ((__CHANNEL__) >> 2U)))

/* USER CODE BEGIN Includes */

/* ---------- Defines of Wheel Speed Sensor ---------*/
#define pi 3.1415
#define radiusOfWheel .1
#define clkFreq 16000000
#define maximumValueOfTimer 65536

/* ---------- Defines of Steering Sensor ---------*/

#define  max_no_of_bits 10 //resolution of sensor
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* -----------Wheel Speed Sensor Variables------------- */

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

uint16_t updateCounter = 0 ;    // variable to know how many times the timer make an update between two pulses

/*------------ Steering Sensor Variables ------------- */

float g_angle=0;//stores position(angle) of shaft
uint32_t g_flag=0;//to know that we have our bits that represent el reading completed shifted
uint32_t g_complete_reading=0;//which will be sent to graytoDecimal function(contains complete gray code after shifting 10 bits)
uint32_t g_mul=1;//to construct gray_code from bits shifted to micro_controller

/*------------ Suspension Sensor Variables ------------- */
uint16_t adc_data[4];
float displacement[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

uint32_t graytoDecimal(uint32_t graycode,uint32_t size_);
void shift_bit(uint16_t reading_bit);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* --------- functions of wheel speed sensor --------*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance==TIM1)
  {
		if((htim->Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U){	 // check if channel 1 has been interrupted
			frontRightFlag =1 ;
		}
    if((htim->Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00U){		// check if channel 2 has been interrupted
			frontLeftFlag = 1 ;
		}
		if((htim->Instance->CCMR2 & TIM_CCMR2_CC3S) != 0x00U){	  // check if channel 3 has been interrupted
		rearRightFlag =1 ;
	}
    if((htim->Instance->CCMR2 & TIM_CCMR2_CC4S) != 0x00U){		// check if channel 4 has been interrupted
			rearLeftFlag =1 ;
		}

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // check the update of timer
	{
		if (htim->Instance==TIM1){     // update of timer 1 for Wheel speed sensor
		updateCounter ++ ;
	}
		if (htim->Instance==TIM3)  {	 // update of timer 3 for suspension sensor

					displacement[0]=((float)adc_data[0]/4096)*50;
					displacement[1]=((float)adc_data[1]/4096)*50;
					displacement[2]=((float)adc_data[2]/4096)*50;
					displacement[3]=((float)adc_data[3]/4096)*50;
		        }
}

/* --------- functions of steering sensor --------*/

uint32_t graytoDecimal(uint32_t graycode,uint32_t size_)
{
{
	uint32_t gray[size_];
	uint32_t binary[size_];
for(uint32_t i=0;i<size_;i++)
{
	gray[size_-1-i]=graycode%10 ;
	graycode/=10;
	if(graycode==0) break;//no need to complete the loop

}

    // MSB of binary code is same as gray code
    binary[0] = gray[0];

    // Compute remaining bits
    for (uint32_t i = 1; i <size_; i++) {
        // If current bit is 0, concatenate
        // previous bit
        if (gray[i] == 0){
            binary[i]= binary[i - 1];
        }
        // Else, concatenate invert of
        // previous bit
        else
        {
            if(binary[i-1]==0)
            binary[i] = 1;
            else
            binary[i]=0;
        }

    }

    uint32_t mul=1;
    uint32_t our_Decimal=0;
    for(uint32_t i=size_-1;i>=0;i--)
    {
        our_Decimal+=mul*binary[i];//converting to decimal
        mul*=2;
    }
    return our_Decimal;
}
}
void shift_bit(uint16_t reading_bit)
{

    g_complete_reading+=reading_bit*g_mul;
    g_flag++;
    g_mul*=10;
    if(g_flag==max_no_of_bits)
    {
        //cout<<endl<<g_complete_reading; this is used for testing on codeBlocks
        g_angle=graytoDecimal(g_complete_reading,max_no_of_bits)/pow(2,max_no_of_bits);
        g_complete_reading=0;
        g_flag=0;
        g_mul=1;
        g_angle*=360;
        //cout<<endl<<g_angle;also for testing
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
	/* ------------ variables of wheel speed sensor ----------*/
	uint32_t frontRightDuration =0 ;		// duration of 1/4 of front Right Wheel
	uint32_t frontLeftDuration =0 ;			// duration of 1/4 of front left Wheel
	uint32_t rearRightDuration =0 ;			// duration of 1/4 of rear Right Wheel
	uint32_t rearleftDuration =0 ;			// duration of 1/4 of rear left Wheel

	uint8_t frontRightWheelCheck =0; 		// variable to check if the steps of calculation of the front right wheel duration have been done or not
	uint8_t frontLeftWheelCheck =0; 		// variable to check if the steps of calculation of the front left Wheel duration have been done or not
	uint8_t rearRightWheelCheck =0; 		// variable to check if the steps of calculation of the rear Right Wheel duration have been done or not
	uint8_t rearLeftWheelCheck =0; 			// variable to check if the steps of calculation of the rear left Wheel duration have been done or not

	/* ------------ variables of steering sensor ----------*/
	uint16_t  reading_bit=0;//for reading bit that comes from shift register

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* ---------- init of suspension sensor -----------*/
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_data,4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	 /* ----------- code of wheel speed sensor-----------*/
	  if (updateCounter <100 )
	  	{
	  	if (frontRightFlag == 1){
	  		frontRightInputCapture= (((updateCounter * maximumValueOfTimer) + __HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_1))- frontRightInputCapture);    //read TIM1 channel 1 capture value
	  		frontRightDuration = ( frontRightInputCapture / clkFreq ) ;    // duration in seconds
	  		frontRightWheelSpeed = ((((60 /(4*frontRightDuration)) *(2*pi/60)) * radiusOfWheel )* (18/5)) ; // 60 to convert to min,*4 to make it for one revolution ,2pi/60 to convert to rad/sec, *raduis to convert to m/sec, *(18/5) to convert to km/h
	  		frontRightFlag =0 ;
	  		frontRightWheelCheck =1 ;  // change 	front Right Wheel Check value to 1 to make sure the calculations have been done
	  	}
	  	if (frontLeftFlag == 1){
	  		frontLeftInputCapture= (((updateCounter * maximumValueOfTimer) + __HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_2))- frontLeftInputCapture);    //read TIM1 channel 2 capture value
	  		frontLeftDuration = (frontLeftInputCapture / clkFreq) ;    // duration in seconds
	  		frontLeftWheelSpeed = ((((60 /(4*frontLeftDuration)) *(2*pi/60)) * radiusOfWheel )*(18/5)) ; // 60 to convert to min,*4 to make it for one revolution, 2pi/60 to convert to rad/sec, *raduis to convert to m/sec, *(18/5) to convert to km/h
	  		frontLeftFlag =0 ;
	  		frontLeftWheelCheck = 1 ;  // change 	front Left Wheel Check value to 1 to make sure the calculations have been done
	  	}
	  	if (rearRightFlag == 1){
	  		rearRightInputCapture= (((updateCounter * maximumValueOfTimer) + __HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_3))- rearRightInputCapture);    //read TIM1 channel 3 capture value
	  		rearRightDuration = (rearRightInputCapture / clkFreq);    // duration in seconds
	  		rearRightWheelSpeed = ((((60 /(4*rearRightDuration)) *(2*pi/60)) * radiusOfWheel )*(18/5)) ;// 60 to convert to min,*4 to make it for one revolution, 2pi/60 to convert to rad/sec, *raduis to convert to m/sec ,*(18/5) to convert to km/h
	  		rearRightFlag =0 ;
	  		rearRightWheelCheck =1;  // change 	rear Right Wheel Check value to 1 to make sure the calculations have been done
	  	}
	  	if (rearLeftFlag== 1){
	  		rearLeftInputCapture= (((updateCounter * maximumValueOfTimer) +__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_4))- rearLeftInputCapture);    //read TIM1 channel 4 capture value
	  		rearleftDuration = (rearLeftInputCapture / clkFreq) ; 		  // duration in seconds
	  		rearLeftWheelSpeed = ((((60 /(4*rearleftDuration)) *(2*pi/60)) * radiusOfWheel )*(18/5)) ;// 60 to convert to min,*4 to make it for one revolution, 2pi/60 to convert to rad/sec, *raduis to convert to m/sec ,*(18/5) to convert to km/h
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

	 /* ----------- code of steering sensor-----------*/
	  reading_bit=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
	  shift_bit(reading_bit);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
