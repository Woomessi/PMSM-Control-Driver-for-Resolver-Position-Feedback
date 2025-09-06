/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "fdcan.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sys.h"
#include "delay.h"
#include "Current_IF2.h"

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

/* USER CODE BEGIN PV */

/* 常量宏定义 */
#define PI					3.14159265358979f

/* LibFOC库函数变量定义 */
float angle_0 = 0;        // 用于 getAngle()
float angle_prev = -1.0f; // 
float shaft_angle=0;
float zero_electric_angle = 0;
float full_rotations=0; // full rotation tracking;

/* 时间戳变量定义 */
float tick_0 = 0;
float tick_1 = 0;
float tick_delta = 0;

/* 串口3 DMA模式输出电机状态至上位机 */
int steps_vofa = 30;
int count_vofa = 30 - 1; //控制串口向上位机发送数据的间隔，保证FOC执行速度

/*******************/
/*** IF 参数调节 ***/
/*******************/

float OPSpd = 200;   // 参考速度(rpm) 200-500 范围内可用
float iq_ref = 5;    // 稳定时的电流
float OPTime = 0;    // 加速时间（Current_IF2 无此状态）
float iq_ref0 = 0.5; // 零位角对齐电流

/* PID */
float P_current = 0.179;
float I_current = 193;

/* 串口3 DMA模式输出电机状态至上位机 */
char test[256];                //调整HAL_UART_Transmit所打印数据的格式
extern DMA_HandleTypeDef hdma_usart3_tx;
float load_data[9];
static uint8_t tempData[40] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x80,0x7F};

/* CAN总线接收解算板的角度信号 */
float position = 0; 
float velocity = 0;
float	position_e = 0;
float position_full = 0;
float velocity_cal = 0;
	
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8]={NULL};
uint8_t TxData[8] = {NULL};
float CANtemp[1];

/* DAC级联封波 */
uint16_t DAC_temp = 0;

/* 定义电机控制重要变量 */
float Vbus,Ia,Ib,Ic;
uint8_t Motor_state = 0;
uint16_t IA_Offset,IB_Offset,IC_Offset;
uint16_t adc1_in1, adc1_in2, adc1_in3, Vpoten, adc_vbus;
uint8_t ADC_offset = 0;

/* Simulink生成函数的输入输出 */
extern ExtU rtU;
extern ExtY rtY;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void FDCAN_Config(void);

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_COMP1_Init();
  MX_DAC3_Init();
  MX_DAC1_Init();
  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
	MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	delay_init(160);	
	
	/* can */
	FDCAN_Config();
	
  /* 内部运放的使能 */
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);
	
	/* ADC自校验，消除寄生电容带来的误差值*/
	HAL_ADCEx_Calibration_Start( &hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start( &hadc2, ADC_SINGLE_ENDED);
	
	__HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_JEOC);
	__HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_EOC);
	__HAL_ADC_CLEAR_FLAG( &hadc2, ADC_FLAG_JEOC);
	
	/* ADC转换中断使能 */
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);//ADC2不需要中断
		
	/* comp */
	TIM1->ARR = 8000 - 1; //TIM1计数最大值
	TIM1->CCR4 = 8000 - 2; //TIM1通道4比较值，ADC采样与转换存在时间延迟，故预留出1次计数周期
	
	HAL_TIM_Base_Start( &htim1);//开启TIM1时钟计数
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_4);//TIM1通道4使能
	
	/* TIM1各通道使能 */
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3);

  /* DAC比较封波 */
	HAL_DAC_Start(&hdac3,DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac3,DAC_CHANNEL_1,DAC_ALIGN_12B_R,2500); //2500, 2140 根据式（5-3）中的value_ADC设置
	HAL_COMP_Start (&hcomp1);	
	
	rtU.Motor_OnOff = 1;
	rtU.P_current = P_current;
	rtU.I_current = I_current;
	
	rtU.OPSpd = OPSpd;
	rtU.iq_ref = iq_ref;
	rtU.OPTime = OPTime;
	rtU.iq_ref0 = iq_ref0;
	
//	tick_0 = HAL_GetTick();
//	tick_0 = tick_0*1E-3;				
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/*** Vbus 电压采样 ***/
		/* ADC软件触发*/
		HAL_ADC_Start(&hadc1);
		HAL_ADC_Start(&hadc2);
		adc_vbus = HAL_ADC_GetValue(&hadc2);
		rtU.v_bus = adc_vbus*3.3f/4096*26;
    load_data[0] = rtU.v_bus;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*** ADC中断回调函数 ***/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	static uint8_t cnt;
	
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
	if(hadc == &hadc1)
	{
		/* 电流采样偏置值计算 */
		if(ADC_offset == 0)
		{
			cnt++;
			adc1_in1 = hadc1.Instance->JDR1;//下桥臂断开时，U相低端采样电压经偏置与放大后的ADC值
			adc1_in2 = hadc2.Instance->JDR1;//下桥臂断开时，V相低端采样电压经偏置与放大后的ADC值
			adc1_in3 = hadc1.Instance->JDR2;//下桥臂断开时，W相低端采样电压经偏置与放大后的ADC值
			IA_Offset += adc1_in1;
			IB_Offset += adc1_in2;
			IC_Offset += adc1_in3;
			if(cnt >= 10)
			{
				ADC_offset = 1;
				IA_Offset = IA_Offset/10;
				IB_Offset = IB_Offset/10;
				IC_Offset = IC_Offset/10;
			}
		}
		else
		{			
			/* 低端电流采样 */
			adc1_in1 = hadc1.Instance->JDR1;
			adc1_in3 = hadc1.Instance->JDR2;
			adc1_in2 = hadc2.Instance->JDR1;
			
			rtU.ia = (adc1_in1 - IA_Offset)*0.02197265625f;// 0.02197265625f
			rtU.ib = (adc1_in2 - IB_Offset)*0.02197265625f;
			rtU.ic = (adc1_in3 - IC_Offset)*0.02197265625f;
						
			/* 执行Simulink控制程序 */
			Current_IF2_step();
			
			/* 更新高级定时器CCR寄存器值，调整PWM占空比 */
			TIM1->CCR1 = rtY.tABC[0];
			TIM1->CCR2 = rtY.tABC[1];
			TIM1->CCR3 = rtY.tABC[2];
			
//			tick_1 = micros();
//			tick_delta = tick_1 - tick_0;
//			tick_0 = tick_1;
//			float speed_cal = cal_angular_vel(position,tick_delta*1E-6);
        
			/* 控制串口发送间隔 */
			count_vofa++;
			if(count_vofa >= steps_vofa)
			{
				memcpy(&position, &RxData[0], sizeof(float));
        memcpy(&velocity, &RxData[4], sizeof(float));			

				/* 向上位机发送信息 */
			  load_data[1] = TIM1->CCR1;
			  load_data[2] = TIM1->CCR2;
			  load_data[3] = TIM1->CCR3;
			  load_data[4] = rtY.id;
		    load_data[5] = rtY.iq;
			  load_data[6] = 0;
			  load_data[7] = position;
		    load_data[8] = velocity;		
				
			  memcpy(tempData, (uint8_t *)&load_data, sizeof(load_data));				
				
			  HAL_UART_Transmit_DMA(&huart3,(uint8_t *)tempData,40);
				
				count_vofa = 0;
			}
		}
	}
  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADCEx_InjectedConvCpltCallback must be implemented in the user file.
  */
}

void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;  
  sFilterConfig.FilterIndex = 0;             
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;  
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x01ffffff;       
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
  
  TxHeader.Identifier = 0x1B;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x52;

  HAL_FDCAN_Start(&hfdcan1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
