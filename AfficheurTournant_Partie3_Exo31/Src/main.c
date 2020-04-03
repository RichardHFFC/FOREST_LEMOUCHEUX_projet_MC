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
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "LibCaracteres.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// Tableau de caracteres pour toute la zone d'affichage

static unsigned char Motif_Afficheur [180]={
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0x80,0xF6,0xF6,0xF6,0xF9,0xFF,    //P
0x80,0xBE,0xBE,0xBE,0x80,0xFF,    //O
0x80,0xBF,0xBF,0xBF,0xBF,0xFF,    //L
0xFE,0xFD,0x83,0xFD,0xFE,0xFF,    //Y
0xFE,0xFE,0x80,0xFE,0xFE,0xFF,    //T
0x80,0xB6,0xB6,0xBE,0xBE,0xFF,    //E
0x80,0xBE,0xBE,0xBE,0xBE,0xFF,    //C
0x80,0xF7,0xF7,0xF7,0x80,0xFF,    //H
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0XFF
};

/* declarer la variable globale : drapeau interruption externe de la Patte PA15 : fourche */ 
// mettre votre code en dessous
volatile unsigned int tour = 0;
/* declarer la variable globale : drapeau interruption duree pixel  : Timer TIM3 */ 
// mettre votre code en dessous
volatile unsigned int timer3 = 0;
/* declarer la variable globale : drapeau interruption duree tour complet  : Timer TIM17 */ 
// mettre votre code en dessous
volatile unsigned int timer17 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* declarer la variable locale qui compte le nombre de pixel ou colonne */ 
  // mettre votre code en dessous
	unsigned int pixel = 0;
	/* variable local pour choix de la ligne*/ 
	/*unsigned int ligne = 1;*/


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim17);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  if (tour ==1) /* test si le drapeau interruption externe PA15 est a '1' */
	  {
	    /* reset du drapeau interruption externe PA15 pour synchroniser avec la routine d'interruption*/
        // ecrire le code en dessous
			tour = 0;
        /*reset du compteur de colonne*/
        // ecrire le code en dessous
		  pixel = 0;
				/* changement d'état de la variable ligne*/
			/*ligne =~(ligne^tour);*/				/* si ligne = 1 affichage ligne A, si ligne = 0 affichage ligne B*/

		  if ((TIM17->CR1&(1<<0))==0) // test si le bit b0 de TIM17_CR1 est à 0
		  {
			  /*si oui alors demarrer le timer TIM17 : mettre à '1' le bit b0 de TIM17_CR1  */
              // ecrire le code en dessous
			  TIM17->CR1 |= (1<<0);
			  /* Demasquer la demande interruption du timer TIM17 : utiliser la fonction NVIC_EnableIRQ(Numero_IRQn) */
			  // ecrire le code en dessous
				NVIC_EnableIRQ(TIM17_IRQn);
		  }
		  else    // si le TIMER TIM17 n'est pas arreter
		  {
			  /* arreter le timer TIM17 */
              // ecrire le code en dessous
			  TIM17->CR1 &=~(1<<0);
             /* masquer sa demande interruption du TIM17 : utiliser la fonction NVIC_DisableIRQ(Numero_IRQn)*/
             // ecrire le code en dessous
			  NVIC_DisableIRQ(TIM17_IRQn);
			  /* arreter le timer TIM3 */
              // ecrire le code en dessous
				TIM3->CR1 &=~(1<<0);
			  /* masquer la demande interruption du Timer TIM3 : utiliser la fonction NVIC_EnableIRQ(Numero_IRQn)*/   /*?????*/
			  // ecrire le code en dessous
				NVIC_DisableIRQ(TIM3_IRQn);
			  /* copier la valeur du compteur de TIM17 (TIM17_CNT) dans le registre TIM3_ARR du TIMER TIM3 */
			  // ecrire le code en dessous
				TIM3->ARR = (TIM17->CNT);
			  /* mettre a zero le compteur de TIM3 */
			  // ecrire le code en dessous
				TIM3->CNT = 0;
			  /* Demarrer le Timer TIM3 */
			  // ecrire le code en dessous
			  TIM3->CR1 |=(1<<0);
			  /* Demasquer la demande interruption de TIM3 : utiliser la fonction NVIC_DisableIRQ(Numero_IRQn) */  /*?????*/
			  // ecrire le code en dessous
				NVIC_EnableIRQ(TIM3_IRQn);
			  /* mettre a zero le compteur de TIM17 */
			  // ecrire le code ici
				TIM17->CNT = 0;
			  /* Demarrer le Timer TIM17 */
			  // ecrire le code en dessous
				TIM17->CR1 |= (1<<0);
			  /* Demasquer la demande interruption de TIM17 : utiliser la fonction NVIC_DisableIRQ(Numero_IRQn) */  /*?????*/
			  // ecrire le code en dessous
			  NVIC_EnableIRQ(TIM17_IRQn);
		  }
	  }

	  if (timer3 ==1) /* test si le drapeau interruption Timer duree Pixel a '1' : TIM3 */
	  {
		  /* remettre le drapeau interruption Timer dureePixel a '0' : TIM3 */
		  // ecrire le code ici
			timer3 = 0;
		  /* realiser affichage d'une colonne */
		  // ecrire le code en dessous
			/*if (ligne == 1)    si ligne = 1  on affiche la ligne A*/
				GPIOA->ODR &= Motif_Afficheur [pixel];
			/*else	 sinon on affiche la ligne B*/
				GPIOB->ODR &=(Motif_Afficheur [pixel]<<8);
		  /* preparer pour l'affichage de la colonne suivante */
		  // ecrire le code en dessous
			GPIOB->ODR |= (0xFF00);	
			GPIOA->ODR |= (0x00FF);
			pixel++;
	  }

	  if (timer17==1) /* test si le drapeau interruption Timer duree tour complet vaut '1' : TIM17 */
	  {
		 
		  /* remettre le drapeau interruption Timer dureePixel a '1' : TIM3 */    /*??????*/
		  // ecrire le code en dessous
			timer17 = 0;

		  /* arreter le timer tim17 */  
		  // ecrire le code en dessous
			TIM17->CR1 &=~(1<<0);
		  /* masquer la demande interruption du TIM17 */ /*??????*/
		  // ecrire le code en dessous
			NVIC_DisableIRQ(TIM17_IRQn);
		  /* arreter le timer tim17 */         /*??????*/
		  // ecrire le code en dessous
			TIM3->CR1 &=~(1<<0);
		 
		 /* masquer la demande interruption du TIM17 */     /*???????*/
		 // ecrire le code en dessous
			NVIC_DisableIRQ(TIM3_IRQn);
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = PSC_TIM3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 345;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = PSC_TIM17;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 0xFFFF;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |LED5_Pin|LED6_Pin|LED7_Pin|LED8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED11_Pin|LED12_Pin|LED13_Pin|LED14_Pin 
                          |LED15_Pin|LED16_Pin|LED9_Pin|LED10_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin 
                           LED5_Pin LED6_Pin LED7_Pin LED8_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |LED5_Pin|LED6_Pin|LED7_Pin|LED8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED11_Pin LED12_Pin LED13_Pin LED14_Pin 
                           LED15_Pin LED16_Pin LED9_Pin LED10_Pin */
  GPIO_InitStruct.Pin = LED11_Pin|LED12_Pin|LED13_Pin|LED14_Pin 
                          |LED15_Pin|LED16_Pin|LED9_Pin|LED10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Fourche_Pin */
  GPIO_InitStruct.Pin = Fourche_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Fourche_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/*===============================================================================
  FUNCTION:     HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  DESCRIPTION:  Fonction CallBack : appeler a chaque changement état sur les pattes 
  PARAMETERS:   rien
  RETURNS:      rien   
  REQUIREMENTS: la patte doit etre configurer en interruption externe 
===============================================================================*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==Fourche_Pin)  // Test si c'est la patte PA15 
	{
		/* Mettre a '1' le drapeau interruption externe de la Patte PA15 : fourche */
		// ecrire le code en dessous
		tour =1;
	}
}

/*===============================================================================
  FUNCTION:     HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  DESCRIPTION:  Fonction CallBack : appeler a chaque changement état sur la patte PA15
  PARAMETERS:   rien
  RETURNS:      rien   
  REQUIREMENTS: rien
===============================================================================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		/* Mettre a '1' le drapeau interruption duree pixel ou colonne : TIM3 */
		// ecrire le code en dessous
		timer3 = 1;
	}

	if (htim->Instance == TIM17)
	{
		/* Mettre a '1' le drapeau interruption duree tour complet : TIM17 */
		// ecrire le code en dessous
		timer17 = 1;
	}
}


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
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
