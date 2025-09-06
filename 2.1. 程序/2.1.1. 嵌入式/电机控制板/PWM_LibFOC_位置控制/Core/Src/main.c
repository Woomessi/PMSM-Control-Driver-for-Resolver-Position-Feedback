 /***************************************
 * @file           : main.c
 * @author         : 吴宏瑞
 * @date           : 2025/03/01
 ****************************************
 * @attention
 * 仅位置控制，未嵌入电流环，但有电流检测
 ****************************************/


/******************************/
/******* STM32外设头文件 *******/
/******************************/

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
#include "cordic.h"

/*********************************/
/******* 系统常用函数头文件 *******/
/*********************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sys.h"
#include "delay.h"
#include "arm_math.h"

/*********************************/
/******* 电机控制函数头文件 *******/
/*********************************/

#include "pid.h"
#include "control_loop.h"
#include "lowpass_filter.h"

/*****************************************/
/********** LibFOC库函数变量定义 **********/
/*****************************************/

/* control_loop.c 文件中的变量定义 */
float DIR = 1;                   // 对旋转变压器所测得的转子位置的正负进行统一定义
float PP = 8;                    // 电机极对数
float zero_electric_angle = 0;   // 标定的旋变零位
int ARR = 7999;                  // 高级定时器ARR寄存器的值
float voltage_power_supply = 48; // 母线电压大小（V）
float voltage_limit = 48;        // 在setPWM函数中，限制三相电压的最大值（V）

/* 电机运动物理量定义 */
float position_total = 0; // 带圈数的电机位置（rad）
float position_error = 0; // 电机位置误差（rad）

float velocity_error = 0; // 电机速度误差（rad）

/* 电机电气物理量定义 */
float ia = 0;
float ib = 0;
float ic = 0; // 三相电流（A）

float id = 0;
float iq = 0; // dq轴电流（A）

float id_filtered = 0;
float iq_filtered = 0; // 低通滤波后的dq轴电流（A）

float id_error = 0;
float iq_error = 0; // dq轴电流误差（A）

float ud = 0;
float uq = 0; // dq轴电压（V）

/***************************/
/********* 参考值 **********/
/***************************/

float position_ref = PI/2; // 位置 (rad)
float uq_ref = 0;          // 参考q轴电压 (V)

/**********************************************************************/
/*************************** 位置环参数 *******************************/
/**********************************************************************/

/* PID 参数 */
struct PIDController pid_position = {.P = 0.133, .I = 0, .D = 0, .output_ramp = 0, .limit = 24, .error_prev = 0, .output_prev = 0, .integral_prev = 0, .Ts = 1E-4};

/* 更新频率 */
int steps_position = 1;     // 位置环运行间隔 (当ADC低端电流采样中断周期为1e-4时，速度环运行周期为 steps_velocity * 1e-4)
int count_position = 1 - 1; // 保证第一次计数即可执行位置环

/************************************************************************/
/******************* 串口3 DMA模式输出电机状态至上位机 *******************/
/************************************************************************/

int steps_vofa = 100;     // 串口运行间隔 (当ADC低端电流采样中断周期为1e-4时，串口运行周期为 steps_vofa * 1e-4)
int count_vofa = 100 - 1; // 控制串口向上位机发送数据的间隔，保证FOC执行速度

/* 参照 vofa JustFloat 协议 */
char test[256]; // 调整HAL_UART_Transmit所打印数据的格式
extern DMA_HandleTypeDef hdma_usart3_tx;
float load_data[9];
static uint8_t tempData[40] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x80, 0x7F};

/**************************************************************************/
/******************* CAN总线接收旋变解码板传输的角度信号 *******************/
/**************************************************************************/
	
float position = 0;   // rad
float velocity = 0;   // rad/s
float position_e = 0; // rad

FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8] = {NULL};
uint8_t TxData[8] = {NULL};
float CANtemp[1];

/***************************************************/
/******************* DAC级联封波 *******************/
/***************************************************/

uint16_t DAC_temp = 0;

/***************************************************/
/******************* ADC采样变量 *******************/
/***************************************************/
uint16_t IA_Offset, IB_Offset, IC_Offset;
uint16_t adc1_in1, adc1_in2, adc1_in3, Vpoten, adc_vbus;
uint8_t ADC_offset = 0;

/* 函数声明 -----------------------------------------------*/
void SystemClock_Config(void);
void FDCAN_Config(void);

int main(void)
{
    /* 单片机配置 --------------------------------------------------------*/

    /* 重设外设、内存、时钟 */
    HAL_Init();

    /* 系统时钟配置 */
    SystemClock_Config();

    /* 初始化外设 */
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
		MX_CORDIC_Init();      // STM32G474系列产品内置CORDIC运算单元，可以加速三角函数计算
		
    /* 延时函数初始化 */
    delay_init(160);

    /* FDCAN总线初始化 */
    FDCAN_Config();

    /* 如通过插座开关物理按键控制电机启停，需待母线电压稳定后再执行后续程序 */
    delay_ms(1000);

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
    HAL_ADCEx_InjectedStart_IT(&hadc1); // 既有注入组（电流采样），又有规则组（电压采样），
    HAL_ADCEx_InjectedStart(&hadc2);    // 只有注入组

    /* 高级计数器TIM1寄存器值设定 */
    TIM1->ARR = 8000 - 1;  // TIM1计数最大值
    TIM1->CCR4 = 8000 - 2; // TIM1通道4比较值，ADC采样与转换存在时间延迟，故预留出1次计数周期

    HAL_TIM_Base_Start(&htim1);               // 开启TIM1时钟计数
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // TIM1通道4使能

    /* 高级计数器TIM1各通道使能 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* DAC比较封波 */
//    HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
//    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2550); // 2500, 2140 根据式（5-3）中的value_ADC设置
//    HAL_COMP_Start(&hcomp1);
		
		/* CORDIC三角函数计算初始化 */
		Sin_CORDIC_INT();
		Cos_CORDIC_INT();
		
    while (1)
    {
        /*** 母线电压采样 ***/
        /* ADC软件触发*/
        HAL_ADC_Start(&hadc1);
        HAL_ADC_Start(&hadc2);
        adc_vbus = HAL_ADC_GetValue(&hadc2);
        load_data[0] = adc_vbus * 3.3f / 4096 * 26;
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

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
            /**************/
            /*** 位置环 ***/
            /**************/
            count_position++;
            if (count_position >= steps_position)
            {
						    /* 低端电流采样 */
                adc1_in1 = hadc1.Instance->JDR1;
                adc1_in2 = hadc2.Instance->JDR1;
                adc1_in3 = hadc1.Instance->JDR2;

                /* 相电流反馈 */
					      ia = (adc1_in1 - IA_Offset) * 0.02197265625f; // 欧拉电子: 0.02197265625f
					      ib = (adc1_in2 - IB_Offset) * 0.02197265625f; // 控制板v2.1: 0.10986328125f
                ic = (adc1_in3 - IC_Offset) * 0.02197265625f;					
            						
                /* 读取角度 [0, 2PI] */  
					      memcpy(&position, &RxData[0], sizeof(float)); // rad
							
							  /* 电角度计算 */
                position_e = _electricalAngle(position);      // rad
						 
					      /* 读取旋转变压器测得的速度 (rad/s) */
                memcpy(&velocity, &RxData[4], sizeof(float)); // rad/s

						    /* Clark与Park变换 */				
					    	iq = cal_Iq(ia, ib, position_e);
					    	id = cal_Id(ia, ib, position_e);
						
					
					      /* 误差计算 */
								
							  // 使用不带圈数的角度
							  position_error = position_ref - DIR * position;
                position_error = _normalizeAngle(position_error); // 归一化误差至[0, 2PI]区间
                if (position_error > PI)
                {
                    position_error -= 2 * PI;
                }								
								position_error = position_error / PI * 180;        //将误差的单位由(rad)转换为(deg)
							  
								/* 使用 PID_operator 输出电流参考值 */
                uq_ref = PID_operator(position_error, &pid_position);

                /* 参考电压赋值 */
                setPhaseVoltage(uq_ref, 0, position_e, TIM1);
						
						    count_position = 0;
			      }   
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
    while ((USART3->ISR & 0X40) == 0)
        ;
    USART3->TDR = (uint8_t)ch;
    return ch;
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
