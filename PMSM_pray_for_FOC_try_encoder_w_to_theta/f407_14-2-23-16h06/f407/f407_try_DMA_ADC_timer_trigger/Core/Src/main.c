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
#include "pmsm_control.h"
#include "stm32f4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t adcVal[3] ;
float	U_alpha = 0, U_beta = 0, U_d = 0, U_q  = 0, ab_theta ,  elecTheta = gocBanDau , tempElecTheta = 0 ;
float Ia  = 0 , Ib  = 0, Ic  = 0;
float I_alpha  = 0, I_beta = 0 ;
float I_d = 0, I_q  = 0, I_q_lpss = 0 , I_d_lpss = 0 , I_q_ref = 0 ;
#define	T_halfsample  0.00005
uint8_t direct = 0 ;
float	U_max = 24;
uint16_t	*switchtime[3];
float mech_w = 0 , mech_rpm = 0 ;
float elec_w = 0 , prev_elec_w = 0 , ave_elec_w = 0 ;  //314 <=> 50hz <=> 750rpm 
float temp_w ;
float sum_elec_w = 0 ;
uint8_t w_count = 1 ;
uint8_t calib_flag = 0 ;
uint16_t calib_count = 0 ;
uint16_t ADC_OFFSET_1 = 1567 ;
uint16_t ADC_OFFSET_2 = 1564 ;
uint16_t ADC_OFFSET_3 = 1564 ;
// not important variable
uint8_t mode_dac = 0 ;
uint8_t motor_mode =  0 ;
uint32_t mode_count = 0 ;
extern float spd_err ;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float lowpass(uint16_t signal , float f2pi , uint16_t *temp ){
		float out_LP;
    out_LP = (*temp+f2pi*signal)/(f2pi+1);
    *temp = out_LP;
    return out_LP;
}

float f_lowpass(float signal , float f2pi , float *temp ){
		float out_LP;
    out_LP = (*temp+f2pi*signal)/(f2pi+1);
    *temp = out_LP;
    return out_LP;
}
void ADC_CALIB_OFFSET() {
				ADC_OFFSET_1 = lowpass( adcVal[PHA1], 0.0314159 , &ADC_OFFSET_1 );
			  ADC_OFFSET_2 = lowpass( adcVal[PHA2], 0.0314159 , &ADC_OFFSET_2 );
			  ADC_OFFSET_3 = lowpass( adcVal[PHA3], 0.0314159 , &ADC_OFFSET_3 );
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 1 );	
/* Program START ========================================================================================================================================================================= */
	/* Start angle frequency w and angle theta */
if (motor_mode == AUTO) {
				elec_w = 314 ;
				elecTheta += elec_w*0.0002 ;
				if (elecTheta >= pi2 ) {elecTheta -= pi2 ; }
//				mode_count ++ ;
//				if (mode_count >= 10000 ) { motor_mode = CONTROL ;}
			}     
if (motor_mode == CONTROL ) {				
						/* mechanical speed to electrical speed */
					mech_w = 6283/period ;   // mechanical speed or rotor speed 
				  if (direct == BACK )	{elec_w = 4*mech_w ; }      // electrical speed 
					else if (direct == FORWARD  ) { elec_w = (-4)*mech_w ;}	  
					 /*electrical speed to electrical angle  */
					if (prev_elec_w == 0 ) { prev_elec_w = elec_w ;}    				
					elecTheta -= (prev_elec_w + elec_w)*0.0001 ;   // 0.0001 = Ts/2 = 0.0002/2 = (1/5000)/2
				  if (elecTheta >= pi2 ) {elecTheta -= pi2 ; }   //upper limit of Theta is 2pi
				  if (elecTheta <0 ) {elecTheta += pi2   ; }          // lower limit of Theta is 0 
					prev_elec_w = elec_w ;     
						/* average speed calcutaion */
					if (w_count >= SUM_ELEC_W_NUM ) 													// if full
						{	ave_elec_w = sum_elec_w / SUM_ELEC_W_NUM; 
							w_count = 1 ;
						  sum_elec_w = ave_elec_w  ;}
					else if (w_count < SUM_ELEC_W_NUM ) 											//if not full yet
					  {	w_count ++ ;
							sum_elec_w += elec_w ;
						}   
					} 
					mech_rpm = ave_elec_w *2.387324146 ;
/* End angle frequency w and angle theta */
		/* Current Measure START*/	
					/* Current cabib START*/
						if (calib_flag == 1 ) {
								ADC_CALIB_OFFSET () ;
								calib_count ++ ;
								if (calib_count >= 1000 ) {
											calib_flag = 0 ;
											calib_count = 0 ;
											}
						}
					/* Current calib END */						
				Ia = current_calc (adcVal[PHA1], ADC_OFFSET_1);  // current calculation
				Ib = current_calc (adcVal[PHA2], ADC_OFFSET_2);
				Ic = current_calc (adcVal[PHA3], ADC_OFFSET_3);						
				abc_to_alp_bet (Ia , Ib, Ic , &I_alpha , &I_beta );    //clark transformation	a b c  >>> alpha beta
				
        float temp_sin = sinlkrd(elecTheta ) ;
        float temp_cos = coslkrd(elecTheta ) ;
						
				I_d = I_alpha*temp_cos  + I_beta * temp_sin;	// Park transformation alpha beta >>> d q
				I_q = -I_alpha*temp_sin + I_beta * temp_cos;	
				//I_q_lpss = f_lowpass( I_q , 0.628 , &I_q );  //50Hz
HAL_DAC_SetValue (&hdac , DAC_CHANNEL_1 ,DAC_ALIGN_12B_R, I_d*620 + 2048 );		//I_d *248 +			
HAL_DAC_SetValue (&hdac , DAC_CHANNEL_2 ,DAC_ALIGN_12B_R, I_q*620 + 2048 );				
  /* Current Measure END*/
	if (motor_mode == AUTO ) {	
			U_d = 0 ;
			U_q = 3 ;
	}
	if (motor_mode == CONTROL ) {					
	I_q_ref = PI_speed( ave_elec_w );	
	//I_q_ref = 0.5;
	U_q = PI_Q(I_q , I_q_ref);
	//U_q =  3 ;
	U_d = PI_D(I_d);
				}
	
	U_alpha = U_d*temp_cos - U_q*temp_sin ;  // invert Park transformation
	U_beta 	= U_d*temp_sin + U_q*temp_cos ;
	ab_theta = fmod((atan2(U_beta, U_alpha ) + pi2), pi2) ; // alpha beta >>> arctan >>> Theta   
	svpwm();	
	TIM8 -> CCR1 = *switchtime[PHA1] ;
	TIM8 -> CCR2 = *switchtime[PHA2] ;
	TIM8 -> CCR3 = *switchtime[PHA3] ;

	/*program	END =============================================================================================================================================================================	*/
	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 0);
}// END ADC INTERRUPT    

void svpwm(void)	{
	 uint8_t sector = ab_theta /PIdiv3;
	//calculate theta using agular speed
	float U_ref = sqrt(U_alpha*U_alpha + U_beta*U_beta);
	if (U_ref > U_max) {
			U_ref = U_max;
	} 
			float	angle =  ab_theta - (sector*PIdiv3);  
			float	U_ref_percent = (SQRT3)*(U_ref/U_DC); // previous: (2/_SQRT3)
	float t_1 = U_ref_percent*sin(PIdiv3-angle)*T_halfsample;
	float t_2 = U_ref_percent*sin(angle)*T_halfsample;
	float t_0 = T_halfsample - t_1 - t_2;
	float t_0_half = t_0/2;
	// Switching counter values for Timer Interrupts
	// Upper switches 
	uint16_t ontime_t_0_half = (t_0_half) * counterfrequency;
	uint16_t ontime_value_1 = (t_0_half + t_1) * counterfrequency;
	uint16_t ontime_value_2 = (t_0_half + t_2) * counterfrequency;
	uint16_t ontime_value_3 = (t_0_half + t_1 + t_2) * counterfrequency;

	switch (sector)	{
  //Upper switches			
  // Sector 1
		case 0:		switchtime[PHA1] = &ontime_t_0_half;
					switchtime[PHA2] = &ontime_value_1;
					switchtime[PHA3] = &ontime_value_3;
				break;
		// Sector 2 
		case 1:		switchtime[PHA1] = &ontime_value_2;
					switchtime[PHA2] = &ontime_t_0_half;
					switchtime[PHA3] = &ontime_value_3;
				break;
		// Sector 3 
		case 2:		switchtime[PHA1] = &ontime_value_3;
					switchtime[PHA2] = &ontime_t_0_half;
					switchtime[PHA3] = &ontime_value_1;
				break;
		//Sector 4 
		case 3:		switchtime[PHA1] = &ontime_value_3;
					switchtime[PHA2] = &ontime_value_2;
					switchtime[PHA3] = &ontime_t_0_half;
				break;
		// Sector 5 
		case 4:		switchtime[PHA1] = &ontime_value_1;
					switchtime[PHA2] = &ontime_value_3;
					switchtime[PHA3] = &ontime_t_0_half;
				break;
		// Sector 6 
		case 5:		switchtime[PHA1] = &ontime_t_0_half;
					switchtime[PHA2] = &ontime_value_3;
					switchtime[PHA3] = &ontime_value_2;
				break;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)  {
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
  MX_TIM8_Init();
  MX_ADC3_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim8 , TIM_CHANNEL_1 );	//inverter pin
HAL_TIMEx_PWMN_Start(&htim8 , TIM_CHANNEL_1 );

HAL_TIM_PWM_Start(&htim8 , TIM_CHANNEL_2 );	//inverter pin
HAL_TIMEx_PWMN_Start(&htim8 , TIM_CHANNEL_2 );

HAL_TIM_PWM_Start(&htim8 , TIM_CHANNEL_3 );	//inverter pin
HAL_TIMEx_PWMN_Start(&htim8 , TIM_CHANNEL_3 );
//__HAL_TIM_SET_COMPARE( &htim8 , TIM_CHANNEL_1 ,400);/////===================================////////////////////////////////
__HAL_TIM_SET_COMPARE( &htim8 , TIM_CHANNEL_4 ,400);// set synchronous channel
HAL_TIM_Base_Start_IT (&htim8);

HAL_TIM_PWM_Start(&htim2 , TIM_CHANNEL_4 );	
__HAL_TIM_SET_COMPARE( &htim2 , TIM_CHANNEL_4 ,400); // set adc trigger instant
HAL_TIM_Base_Start_IT (&htim2);	
HAL_TIM_Base_Start(&htim5);
HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);
	// (&hadc1 , buffer , 3);
	
	if (HAL_ADC_Start_DMA(&hadc3, adcVal, 3) != HAL_OK)
	{
		Error_Handler();
	}
	
HAL_DAC_Start (&hdac , DAC1_CHANNEL_1 );// dac for debug
HAL_DAC_Start (&hdac , DAC1_CHANNEL_2 );// dac for debug

  // on off shutdown pin
motor_mode = CONTROL   ;
//motor_mode = AUTO  ;
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//-----if (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_4 ) == GPIO_PIN_SET ) { calib_flag = 1 ;}  //key 5  to calib the offset
		
		//------if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_0 ) == GPIO_PIN_SET ) { motor_mode = AUTO  ; HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET); NNN = 1 ; }  //key 1  on 
		//-------if (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_2 ) == GPIO_PIN_SET ) {	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET); NNN = 0 ; elec_w  = 0 ;}   //key 2 off
		
	//	HAL_Delay( 10 );
		
		//if ((elec_w  < 700)&&(motor_mode == AUTO )) {elec_w += 1 ;}
		//if ((refSpeed < 500)&&(motor_mode == CONTROL )) {refSpeed  += 5;}
		
//		for (int f = 0 ; f< 100 ; f++) {int g = 1;}
//		__HAL_TIM_SET_COMPARE( &htim8 , TIM_CHANNEL_1 , 300);
//		__HAL_TIM_SET_COMPARE( &htim8 , TIM_CHANNEL_2 , 300);
//		__HAL_TIM_SET_COMPARE( &htim8 , TIM_CHANNEL_3 , 300);
//	  HAL_Delay(1000);
//	  if (sin_step < 10 ) { sin_step +=1 ;}
		
	  //else { sin_step = 10 ;}
		/*START - xuat DAC dong dien theo nut nhan */
//		if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_0 ) == GPIO_PIN_SET ) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET); NNN = 1 ; }  //key 1  on
//		if (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_2 ) == GPIO_PIN_SET ) {	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET); NNN = 0 ; w_act = 0 ;}  //key 2 off
//	 --------------------------	if (HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13 ) == GPIO_PIN_SET ) {    //key 3
//	 --------------------------			mode_dac = 1 ;
//	 --------------------------	}
//	 --------------------------	if (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_3 ) == GPIO_PIN_SET ) {    //key 4
//	 --------------------------		 mode_dac = 2;			
//	 --------------------------	}
// --------------------------	if (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_4 ) == GPIO_PIN_SET ) {
// --------------------------			mode_dac = 3 ;

		/*END - xuat DAC dong dien theo nut nhan */
		/* START - nut nhan tang giam toc do */
//		if (HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13 ) == GPIO_PIN_SET ) {  //key 3
//			HAL_Delay(10) ;
//			 if (HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13 ) == GPIO_PIN_RESET )   {w_act += 20 ; } 
//				if (w_act >=300) {
//						w_act = 300 ; 
//				}
//			}
	
//			if (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_3 ) == GPIO_PIN_SET ) {  //key 4 ???
//			HAL_Delay(10) ;
//			 if (HAL_GPIO_ReadPin (GPIOG, GPIO_PIN_3 ) == GPIO_PIN_RESET )   {w_act -= 20 ; } 
//				if (w_act <= 0) {
//						w_act = 0 ; 
//				}
//			}
    	/*END - nut nhan tang giam toc do */
							
								 

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
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC4;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = 800;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 79;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 9;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = 800;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 100;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE6 PE9 PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PG3 PG2 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
