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
#include "svpwm.h"

#include "lowpass_filter.h"
#include "pid.h"
#include "control_loop.h"
#include "arm_math.h"

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
#define PI 3.14159265358979f
#define _1_3 0.333333333333f
#define _2_3 0.666666666667f
#define _SQRT3_3 0.57735026919f

/* LibFOC库函数变量定义 */
float DIR = 1;                 // 传感器数据方向
float PP = 8;                  // 电机极对数

float zero_electric_angle = 0; // 零位角

float ia = 0;
float ib = 0;
float ic = 0;

float id = 0;
float iq = 0;

float id_filtered = 0;
float iq_filtered = 0;

float id_error = 0;
float iq_error = 0;

float ud = 0;
float uq = 0;

float ct = 0;
float st = 0;

int period = 7999; // period for the PWM

float voltage_limit = 48;
float voltage_power_supply = 48;

/***************************/
/********* 参考值 **********/
/***************************/
float iq_ref = 3;          // 参考电流 (A)
float id_ref = 0;

/********************************/
/********** 电流环参数 **********/
/********************************/
/* PID 参数 */
//struct PIDController pid_current = {.P = 0.179f, .I = 193, .D = 0, .output_ramp = 0, .limit = 24.942f, .error_prev = 0, .output_prev = 0, .integral_prev = 0, .Ts = 1E-4f};
struct PIDController pid_current = {.P = 5.6f, .I = 6000, .D = 0, .output_ramp = 0, .limit = 24.942f, .error_prev = 0, .output_prev = 0, .integral_prev = 0, .Ts = 1E-4f};

//SVPWM输出相电压的基波幅值为母线电压二分之根号三

/* 低通滤波 */
struct LowPassFilter filter_current = {.alpha = 0.5f, .y_prev = 0.0f};
	
/************************************************************************/
/******************* 串口3 DMA模式输出电机状态至上位机 *******************/
/************************************************************************/
int steps_vofa = 30;
int count_vofa = 30 - 1; // 控制串口向上位机发送数据的间隔，保证FOC执行速度

/************************************************************************/
char test[256]; // 调整HAL_UART_Transmit所打印数据的格式
extern DMA_HandleTypeDef hdma_usart3_tx;
float load_data[9];
static uint8_t tempData[40] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x80, 0x7F};

/* CAN总线接收解算板的角度信号 */
float position = 0;   // rad
float velocity = 0;   // rad/s
float position_e = 0; // rad

FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8] = {NULL};
uint8_t TxData[8] = {NULL};
float CANtemp[1];

/* DAC级联封波 */
uint16_t DAC_temp = 0;

/* 定义电机控制重要变量 */
float Vbus, Ia, Ib, Ic;
uint8_t Motor_state = 0;
uint16_t IA_Offset, IB_Offset, IC_Offset;
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
//    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */
    delay_init(160);

    /* can */
    FDCAN_Config();

    /* 待电压稳定后执行后续程序 */
		delay_ms(1000);                              // 待电压稳定后执行后续程序

    /* 内部运放的使能 */
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    /* ADC自校验，消除寄生电容带来的误差值*/
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC);

    /* ADC转换中断使能 */
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2); // ADC2不需要中断

    /* comp */
    TIM1->ARR = 8000 - 1;  // TIM1计数最大值
    TIM1->CCR4 = 8000 - 2; // TIM1通道4比较值，ADC采样与转换存在时间延迟，故预留出1次计数周期

    HAL_TIM_Base_Start(&htim1);               // 开启TIM1时钟计数
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // TIM1通道4使能

    /* TIM1各通道使能 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		
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
        rtU.v_bus = adc_vbus * 3.3f / 4096 * 26;
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_FDCAN;
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
    if (hadc == &hadc1)
    {
        /* 电流采样偏置值（零漂）计算 */
        if (ADC_offset == 0)
        {
            cnt++;
            adc1_in1 = hadc1.Instance->JDR1; // 下桥臂导通时，U相低端采样电压经偏置与放大后的ADC值
            adc1_in2 = hadc2.Instance->JDR1; // 下桥臂导通时，V相低端采样电压经偏置与放大后的ADC值
            adc1_in3 = hadc1.Instance->JDR2; // 下桥臂导通时，W相低端采样电压经偏置与放大后的ADC值
            IA_Offset += adc1_in1;
            IB_Offset += adc1_in2;
            IC_Offset += adc1_in3;
            if (cnt >= 10)
            {
                ADC_offset = 1; // 只计算一次
                IA_Offset = IA_Offset / 10;
                IB_Offset = IB_Offset / 10;
                IC_Offset = IC_Offset / 10;
            }
        }
        else
        {
	          /****************/				
            /*** 闭环电流 ***/
					  /****************/
					
   					/* 低端电流采样 */
            adc1_in1 = hadc1.Instance->JDR1;
            adc1_in2 = hadc2.Instance->JDR1;
            adc1_in3 = hadc1.Instance->JDR2;

            /* 相电流反馈 */
					  ia = (adc1_in1 - IA_Offset) * 0.02197265625f; // 欧拉电子: 0.02197265625f
					  ib = (adc1_in2 - IB_Offset) * 0.02197265625f; // 控制板v2.1: 0.10986328125f
            ic = (adc1_in3 - IC_Offset) * 0.02197265625f;

						/* 读取角度 [0, 2PI] */
            memcpy(&position, &RxData[0], sizeof(float)); // (rad)
						memcpy(&velocity, &RxData[4], sizeof(float));	// (rad/s)
						
						/* 关节电角度反馈（rad）*/
            position_e = _electricalAngle(position);						
            
						/* Clark与Park变换 */				
						iq = cal_Iq(ia, ib, position_e);
						id = cal_Id(ia, ib, position_e);
												
						/* 使用 PID_operator 输出电压参考值 */					
						iq_error = iq_ref - iq;
						id_error = id_ref - id;

						/* 电流滤波 */
						iq_error = LowPassFilter_operator(iq_error, &filter_current);
						id_error = LowPassFilter_operator(id_error, &filter_current);						

						uq = PID_operator(iq_error, &pid_current);
						ud = PID_operator(id_error, &pid_current);					

            /*************/
            /*** SVPWM ***/
						/*************/
						
						rtU.position_e = position_e;
						rtU.uq = uq;
						rtU.ud = ud;
						
						svpwm_step();
						
						/* 更新高级定时器CCR寄存器值，调整PWM占空比 */
            TIM1->CCR1 = rtY.tABC[0];
            TIM1->CCR2 = rtY.tABC[1];
            TIM1->CCR3 = rtY.tABC[2];

						/***********************/
            /*** 控制串口发送间隔 ***/
						/***********************/
						
            count_vofa++;
            if (count_vofa >= steps_vofa)
            {
                /* 向上位机发送信息 */
                load_data[1] = ia;
                load_data[2] = ib;
                load_data[3] = ic;
                load_data[4] = id;
                load_data[5] = iq;
                load_data[6] = 0;
                load_data[7] = position;
                load_data[8] = velocity;
							
                memcpy(tempData, (uint8_t *)&load_data, sizeof(load_data));
                HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tempData, 40);
							
                count_vofa = 0;
            }
						/************************/
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

int fputc(int ch, FILE *f)
{
 while ((USART3->ISR & 0X40) == 0); 
 USART3->TDR = (uint8_t)ch; 
 return ch;
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

#ifdef USE_FULL_ASSERT
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
