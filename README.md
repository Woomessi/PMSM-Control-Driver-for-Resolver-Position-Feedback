[中文](README_CN.md).
# Motor Control Board
## Speed Control
The source code for this part is located in `2.1. 程序\2.1.1. 嵌入式\电机控制板\PWM_LibFOC_速度控制`. The LibFOC motor control library used is based on [DengFOC](https://dengfoc.com/#/dengfoc/%E7%81%AF%E5%93%A5%E6%89%8B%E6%8A%8A%E6%89%8B%E6%95%99%E4%BD%A0%E5%86%99FOC%E7%AE%97%E6%B3%95/%E5%BA%8F%E4%B8%BA%E4%BB%80%E4%B9%88%E8%A6%81%E5%87%BA%E8%BF%99%E7%B3%BB%E5%88%97%E8%AF%BE%E7%A8%8B.md) and [DengFOC_on_STM32](https://github.com/haotianh9/DengFOC_on_STM32). The program framework is mainly built using CubeMX (HAL library), and the project file is `STM32G4_GPIO.uvprojx` in `2.1. 程序\2.1.1. 嵌入式\电机控制板\PWM_LibFOC_速度控制\MDK-ARM`.

Before updating the project using CubeMX, ensure that the code written by the user is placed between the /* USER CODE BEGIN xx */ and /* USER CODE END xx */ structures; otherwise, it will be deleted.

The control architecture of this part of the program is FOC, the current control strategy is "direct-axis current zero", the control algorithm is PID, and the PWM control technology is "sinusoidal pulse width modulation". The code related to position control is similar to speed control and can directly refer to the content of this section.

<img src="https://cdn.nlark.com/yuque/0/2025/webp/33745167/1742302430811-43780d8d-265c-4535-8e01-2173e5e24d32.webp" alt="Classification of common concepts in the field of motor control">

### Microcontroller Configuration
Starting from the main function `main()` at line 143 of the `main.c` file, the microcontroller is first configured. This part can be learned with reference to [欧拉电子 STM32G4 Simulink FOC开发实战](https://space.bilibili.com/458115745/lists/1688763?type=season). The relevant source code is as follows.

```cpp
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
HAL_ADCEx_InjectedStart_IT(&hadc1);
HAL_ADCEx_InjectedStart(&hadc2); // ADC2不需要中断

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
HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2550); // 2500, 2140 根据式（5-3）中的value_ADC设置
HAL_COMP_Start(&hcomp1);
```

#### ADC and DAC
The ADC-related configuration files can be found in lines 156 to 157 of the `main.c` file, `MX_ADC1_Init()` and `MX_ADC2_Init()`. In this program, ADC1 regular group channel 11 is used to collect the bus voltage, with a single-ended sampling range of [0, 3.3 V], a sampling period of 2.5 clock cycles, a clock division of 4, a sampling resolution of 12 bits, right-aligned data, and sampling triggered by software. In the designed control board, the bus voltage of the frameless torque motor is divided by resistors with values of 75 kΩ and 3 kΩ, and the voltage from the 3 kΩ resistor is input to the ADC interface of the microcontroller. Therefore, the conversion relationship between the actual bus voltage and the ADC sampling value is:

<img src="https://cdn.nlark.com/yuque/0/2025/png/33745167/1742289033349-8646c99e-4007-47fb-a5ad-1e380877cab4.png" alt="Conversion formula of bus voltage and ADC sampling value">

ADC1 injected group channels 3 and 12, and ADC2 injected group channel 3 are used to collect three-phase currents. Since the maximum current allowed by the motor selected for this joint is 61 A, the control board uses a sampling resistor with a nominal resistance of 1 mΩ and a maximum operating current of 77 A to sample the phase voltage of the motor winding at the low end. The sampled phase voltage is biased and amplified by the internal operational amplifier (OPAMP) of the microcontroller, and finally input to the ADC interface. The gain of the microcontroller's OPAMP is configured through external resistors, and the gain value of this circuit is configured to 22/3. The ADC interface of the microcontroller is cascaded with the output end of the OPAMP, and the sampled signal amplified by the OPAMP is converted by the injected group of the ADC. The priority of the injected group is higher than that of the regular group, with a single-ended sampling range of [0, 3.3 V], a sampling period of 2.5 clock cycles, and the sampling trigger signal comes from the advanced timer TIM1. To ensure that the input voltage of the ADC is within the single-ended sampling voltage range, the control board uses a voltage divider resistor outside the OPAMP interface of the microcontroller to positively bias the ADC input voltage by 1.65 V. After biasing, considering the 22/3 OPAMP gain and the voltage division of the 1 mΩ low-end sampling resistor, the relationship between the ADC port input voltage, the ADC sampling value, and the phase current can be obtained as:

<img src="https://cdn.nlark.com/yuque/0/2025/png/33745167/1742289223431-c9d8b2ac-5ed2-4201-8a40-5c32713ef959.png" alt="Conversion formula of phase current, ADC input voltage and ADC sampling value">

In lines 210 to 213 of the `main.c` file, we set the DAC comparison and wave blocking to prevent phase current overcurrent. If the ADC current sampling value exceeds the input `Data` of the `HAL_DAC_SetValue` function, the advanced timer PWM wave transmission will stop.

#### Advanced Timer
Among the peripherals of the microcontroller, the advanced counter TIM1 functions to generate PWM waves and control the execution cycle of the program. For the configuration of TIM1, its prescaler `PSC` is 1 division, and the value of the repetition counter register `RCR` is 1, that is, only one update event is triggered in one cycle.

Channels 1 to 3 of TIM1 work in complementary PWM output mode, PWM mode 1, and counting mode is [center-aligned mode](https://www.zhihu.com/question/419152940). The relevant configuration program of TIM1 is located in the function `MX_TIM1_Init()` at line 161 of the `main.c` file.

To prevent simultaneous conduction of the upper and lower bridge arms of the full-bridge module, the dead time of TIM1 is set to 1.5 microseconds, and the corresponding value of the `DTG` register is 0x78.

$ \begin{align}
\text{DT}&=\text{DTG}[7:0]*t_{dtg}\\
&=120*\frac{1}{160\text{M}}*2(\text{Note: CKD frequency division coefficient）}\\
&=1500ns\\
\end{align} $

It can be known from line 195 of the `main.c` file that the value of the auto-reload register `ARR` is 7999. Thus, the frequency $ f_{\text{PWM}} $ of the PWM wave generated by TIM1 is:

$ f_{\text{PWM}} =160\text{MHz}/(7999+1)/2=10\text{kHz} $. Among them, the frequency is divided by 2 due to the center-aligned mode.

TIM1 channel 4, configured in no-output mode, is mainly used to generate TRGO signals to trigger three-phase current sampling of the ADC peripheral, and its PWM mode is 2. To ensure the stability of the sampling value, the lower bridge arms of the three-phase full-bridge inverter circuit must all be turned on at the sampling trigger moment. For this reason, in the center-aligned mode, the value of the register `CCR4` should be close to the overflow value 7999. Due to the delay in ADC sampling and conversion, its value is set to 7998 in line 196 of the `main.c` file.

### while Loop
The functions in the while loop between lines 215 and 223 of the `main.c` file are relatively simple, and are only used for ADC software triggering and [bus voltage sampling](https://zhuanlan.zhihu.com/p/495852697). The ADC conversion formula refers to the `ADC and DAC` part of this document.

```cpp
while (1)
{
    /*** 母线电压采样 ***/
    /* ADC软件触发*/
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    adc_vbus = HAL_ADC_GetValue(&hadc2);
    load_data[0] = adc_vbus * 3.3f / 4096 * 26;
}
```

### ADC Interrupt Callback Function
```cpp
/*** ADC中断回调函数 ***/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED(hadc);
    if (hadc == &hadc1)
    {
        /**************/
        /*** 速度环 ***/
        /**************/
        count_velocity++;
        if (count_velocity >= steps_velocity)
        {
            /* 读取角度 [0, 2PI] */  
            memcpy(&position, &RxData[0], sizeof(float)); // rad

            /* 读取旋转变压器测得的速度 (rad/s) */
            memcpy(&velocity, &RxData[4], sizeof(float)); // rad/s

            /* 低通滤波 */
            velocity = LowPassFilter_operator(velocity, &filter_velocity); // 低通滤波

            /* 误差计算 */
            velocity_error = velocity_ref - DIR * velocity;

            /* 使用 PID_operator 输出电流参考值 */
            velocity_error = velocity_error / M_PI * 180;
            uq_ref = PID_operator(velocity_error, &pid_velocity);

            /* 参考电流赋值 */
            position_e = _electricalAngle(position);
            setPhaseVoltage(uq_ref, 0, position_e, TIM1);

            count_velocity = 0;
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
            load_data[4] = id_filtered;
            load_data[5] = iq_filtered;
            load_data[6] = 0;
            load_data[7] = position;
            load_data[8] = velocity;

            memcpy(tempData, (uint8_t *)&load_data, sizeof(load_data));
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tempData, 40);

            count_vofa = 0;
        }
    }
}
```

When the `CNT` register of the TIM1 advanced counter increments to the `CCR4` comparison value, the ADC interrupt callback function will be triggered. At this time, the ADC injected group completes current sampling and executes programs directly related to motor control.

#### Program Execution Cycle Control
This part of the program commonly has the following structure:

```cpp
count++;
if (count >= steps)
{
    /* 电机控制程序 */
    /*   ......    */
    count = 0;
}
```

This is to control the execution cycle of related programs. For example, if `steps = 100`, the program in the if structure will run once every 100 executions of the ADC interrupt callback function. This structure is mostly used to adjust the operating frequency of different loops in multi-level control (such as position loop + current loop) ([Link 1](https://www.zhihu.com/question/383770864/answer/3454138454), [Link 2](https://www.cnblogs.com/zhongzhe/archive/2012/11/14/2770781.html)), or to limit the frequency of sending data to the upper computer through the serial port (this process is time-consuming and affects the execution efficiency of the algorithm). For the VOFA upper computer serial port protocol tutorial, please refer to [Link](https://www.bilibili.com/video/BV1nH4y1X72k?spm_id_from=333.788.videopod.sections&vd_source=ad856f51618186902c24682a8c8279ff).

#### Low-Pass Filtering
In motor control, the speed data measured by sensors often has large fluctuations (caused by differentiating position data to solve speed). To ensure the stable operation of the motor, it is often necessary to perform [low-pass filtering](https://www.zhihu.com/question/518024588/answer/3364056229) on the speed data. Low-pass filtering in discrete space can be regarded as weighted summation of the data collected at the current moment and the data collected at the previous moment to smooth the data.

<img src="https://cdn.nlark.com/yuque/0/2025/png/33745167/1742301893237-142a47e9-c4ad-4f06-9b17-b14deebce00f.png" alt="Speed data before low-pass filtering">
<img src="https://cdn.nlark.com/yuque/0/2025/png/33745167/1742301893193-813a6be1-2f27-480c-8acc-be5374d5afd4.png" alt="Speed data after low-pass filtering">

#### Inverse Transformation
The `setPhaseVoltage` function converts the q-axis voltage into three-phase voltages, and its program is:

```cpp
void setPhaseVoltage(float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE) {

    //  angle_el = _normalizeAngle(angle_el + zero_electric_angle);

    Uq = _constrain(Uq,-(voltage_power_supply)/2,(voltage_power_supply)/2);

    /* 帕克逆变换(不考虑Ud）*/
    float Ualpha =  -Uq*sin(angle_el);
    float Ubeta  =   Uq*cos(angle_el);

    /* 帕克逆变换(考虑Ud) */
    //	float Ualpha = Ud*cos(angle_el) - Uq*sin(angle_el);
    //  float Ubeta  = Ud*sin(angle_el) + Uq*cos(angle_el);

    /* 克拉克逆变换 */
    float Ua =                    Ualpha + voltage_power_supply/2;
    float Ub = (_SQRT3*Ubeta - Ualpha)/2 + voltage_power_supply/2;
    float Uc = (-Ualpha -_SQRT3*Ubeta)/2 + voltage_power_supply/2;

    setPWM(Ua,Ub,Uc,TIM_BASE);
}
```

In this part of the program, we limit the size of the Uq output by the controller to the range of `±bus voltage/2`, and shift the phase voltage upward by `bus voltage/2` overall in the inverse Clarke transformation, which is determined by the characteristics of PWM modulation. For a detailed explanation, please refer to [Link](https://zhuanlan.zhihu.com/p/99976227).


## Current Control
The source code for this part is located in `2.1. 程序\2.1.1. 嵌入式\电机控制板\SVPWM_MATLAB_电流环`. The MATLAB motor control library used is based on the svpwm subsystem in `交接材料_吴宏瑞\2. 工程文件\2.1. 程序\2.1.2. Simulink仿真及程序生成\IF`, and is generated using Embedded Coder (relevant tutorials can refer to Chapter 13 of the *STM32G4 Simulink FOC Development Kit User Manual* and the corresponding [video](https://www.bilibili.com/video/BV12C4y1f78j/?spm_id_from=333.1387.collection.video_card.click&vd_source=ad856f51618186902c24682a8c8279ff)). The program framework is mainly built using CubeMX (HAL library), and the project file is `STM32G4_GPIO.uvprojx` in `交接材料_吴宏瑞\2. 工程文件\2.1. 程序\2.1.1. 嵌入式\电机控制板\SVPWM_MATLAB_电流环\MDK-ARM`.

<img src="https://cdn.nlark.com/yuque/0/2025/png/33745167/1742361984092-4a7872d5-4eb9-4c5f-b7d4-07fe5801d450.png" alt="svpwm subsystem">

The control architecture of this part of the program is FOC, the current control strategy is "direct-axis current zero", the control algorithm is PID, and the PWM control technology is "Space Vector Pulse Width Modulation (SVPWM)".

### ADC Interrupt Callback Function
The main difference between the current control program and the speed control program using LibFOC is that in the ADC interrupt callback function, the three-phase current of the motor is collected.

#### Current Sampling Offset Calculation
First, to compensate for the zero drift of the ADC, the current sampling offset is calculated before the main control program is executed.

```cpp
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
```

The calculation of the offset is performed only once. Next, based on the ADC sampling value and the offset, the phase current is calculated and used as the feedback signal in the current control process, input to the control algorithm.

#### Current Control Notes
```cpp
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

```

<img src="https://cdn.nlark.com/yuque/0/2025/png/33745167/1742364165223-214e3aa5-4096-4c45-a1d8-e24986f16801.png" alt="Global macro definition">

After converting the Simulink subsystem into executable embedded programs using Embedded Coder, the inputs of the subsystem are defined in the `rtU` structure, the outputs are defined in the `rtY` structure, and the main program is executed through the `xxx_step()` function.

:::danger
It should be noted that if using the self-developed motor control board v2.1 instead of the motor control board provided by Euler Electronics, in addition to configuring GPIO and peripherals, it is necessary to replace `STM32G431xx` with `STM32G474xx` in the global macro definition of Keil, and at the same time, modify the conversion gain from the difference between the ADC sampling value and the offset to the phase current to 0.10986328125f.

In embedded programs, adding `f` after floating-point numbers [can improve calculation efficiency](http://www.openedv.com/thread-40175-1-1.html).

:::

# Resolver Decoding Board
The source code for this part is located in `2.1. 程序\2.1.1. 嵌入式\旋变解码板`. The program framework is mainly built based on the engineering template of 正点原子 (Zhengdian Atom) (HAL library), and the project file is `atk_g474.uvprojx` in `交接材料_吴宏瑞\2. 工程文件\2.1. 程序\2.1.1. 嵌入式\旋变解码板\Board_2.2_CAN_tune\Projects\MDK-ARM`.

The resolver decoding board is mainly based on the [AD2S1210](https://www.analog.com/cn/products/ad2s1210.html) chip, which converts the analog voltage signal output by the resolver into digital position and speed signals. These signals are read through a software-simulated SPI bus (directly simulating the high and low level timing of the SPI protocol through GPIO ports) and then sent to the motor control board via the CAN bus.

## AD2S1210 Driver Program
This part of the program mainly realizes reading and writing between STM32 and AD2S1210 based on software SPI, and can refer to the Software Resources -> AD2S1210 Reference Code section in [Link](https://www.analog.com/cn/products/ad2s1210.html#software-resources).

### Common Functions
:::danger
Special attention should be paid to the actual delay time of the delay function `delay` in the driver program. On the one hand, it will affect the timing of the driver; on the other hand, it will affect the execution efficiency of the program.

:::

#### Initialization Function
When powering on, the AD2S1210 has certain timing requirements for each pin:

<img src="https://cdn.nlark.com/yuque/0/2023/png/33745167/1689497865455-b4570e85-0b02-4c26-8e35-173553fa57be.png" alt="AD2S1210 power-on timing requirements">

Among them, $ \text{t}_\text{RST} $ is at least $ 10\mu $ seconds, and the length of $ \text{t}_\text{TRACK} $ is related to the resolution:

<img src="https://cdn.nlark.com/yuque/0/2023/png/33745167/1689498333563-6c62b475-b1f6-4ad0-b4a4-6045971973aa.png" alt="Relationship between t_TRACK and resolution">

Based on this, we wrote the initialization function of AD2S1210:

```cpp
void AD2S1210Initiate()
{
//RESET->0 initially  
	CLR_RESET();  
	SET_SPL();
	delay_us(20);
	SET_RESET(); 
  delay_ms(20);//delay_ms(10);
	CLR_SPL();
	delay_ms(1);
	SET_SPL();
}
```

#### Working Mode Selection Function
Pulling down or pulling up the two pins `A0` and `A1` can select the working mode of AD2S1210. We wrote the corresponding function to realize this function:

```cpp
void AD2S1210SelectMode(unsigned char mode)
{
	if (mode==POSITION)
	{
		CLR_A0();
		CLR_A1();
		delay(1);//delay_ms(1);		//Normal Mode position output
	}
	else if (mode==VELOCITY)
	{
		CLR_A0();
		SET_A1();
		delay(1);//delay_ms(1);		//Normal Mode velocity output
	}
	else if (mode==CONFIG)
	{
		SET_A0();
		SET_A1();
		delay(1);//delay_ms(1);		//Configuration Mode
	}
}
```

#### Write Function
Writing data to the internal registers of AD2S1210 is done in configuration mode. First, it is necessary to send an 8-bit register address to the SPI interface, and then send 8-bit data. This project directly uses the software SPI write function. After sending data once, it is necessary to pull up the `WR#` pin and use the rising edge to latch the data.

```cpp
void WriteToAD2S1210(unsigned char address, unsigned char data)
{
	unsigned	char	buf;

	//write control register address
	buf = address;

	SET_SCLK();
	delay(1);//delay_ms(4);
	SET_CS();
	delay(1);//delay_ms(4);
	CLR_CS();
	delay(1);//delay_ms(4);
	
	SET_WR();
	delay(1);//delay_ms(4);
	CLR_WR();
	delay(1);//delay_ms(4);

	SPIWrite(1,&buf);	  	 
	
	SET_WR();
	delay(1);//delay_ms(4);	
	SET_CS();	
	//write control register address

	//write control register data
	buf = data;

	SET_SCLK();
	delay(1);//delay_ms(4);
	SET_CS();
	delay(1);//delay_ms(4);
	CLR_CS();
	delay(1);//delay_ms(4);
	
	SET_WR();
	delay(1);//delay_ms(4);
	CLR_WR();
	delay(1);//delay_ms(4);

	SPIWrite(1,&buf);	  	 

	SET_WR();
	delay(1);//delay_ms(4);	
	SET_CS();
	//write control register data
}
```

The corresponding signal timing is as follows:

<img src="https://cdn.nlark.com/yuque/0/2023/png/33745167/1689513847865-b88101a4-3073-42c1-abbe-9eb9b28bea7f.png" alt="Write function signal timing">

#### Read Function
Both position registers and speed registers of AD2S1210 can be read in normal mode or configuration mode, while reading other registers can only be done in configuration mode. To read data in configuration mode, we need to send the address of the target register to the SPI interface. Before reading data in the position register or speed register, it is necessary to first pull up and then pull down the `SAMPLE#` pin to update the data.

```cpp
void ReadFromAD2S1210(unsigned char mode, unsigned char address, unsigned char * buf)
{
	if (mode==CONFIG)
	{
		
		//write control register address
		buf[0] = address;

		SET_SCLK();
		delay(1);//delay_ms(4);
		SET_CS();
		delay(1);//delay_ms(4);
		CLR_CS();
		delay(1);//delay_ms(4);
		
		SET_WR();
		delay(1);//delay_ms(4);
		CLR_WR();
		delay(1);//delay_ms(4);

		SPIWrite(1,buf);

		SET_WR();
		delay(1);//delay_ms(4);	
		SET_CS();
		//write control register address


		//read 1-byte register
		SET_SCLK();
			
		SET_CS();
		SET_WR();
		delay(1);//delay_ms(4);
	
		CLR_CS();
		delay(1);//delay_ms(4);
	
		CLR_SCLK();	
		delay(1);//delay_ms(1);
		
		CLR_WR();
		delay(1);//delay_ms(4);

		SPIRead(1,buf);	

		SET_WR();
		delay(1);//delay_ms(4);

		SET_CS();
		//read 1-byte register
	}
	else if (mode==POSITION||mode==VELOCITY)
	{
		SET_SPL();
		delay(1);//delay_ms(1);
		CLR_SPL();
		delay(5);//delay_ms(5);

		//read 3-byte register 
		SET_SCLK();
				
		SET_CS();
		SET_WR();
		delay(1);//delay_ms(4);
		
		CLR_CS();
		delay(1);//delay_ms(4);
	
		CLR_SCLK();	
		delay(1);//delay_ms(4);
			
		CLR_WR();
		delay(1);//delay_ms(4);
	
		SPIRead(3,buf);		//read data register
	
		SET_WR();
		delay(1);//delay_ms(4);
	
		SET_CS();
		//read 3-byte register


	}
}
```

Signal timing for reading data in configuration mode:

<img src="https://cdn.nlark.com/yuque/0/2023/png/33745167/1689513904592-947b64f0-4943-41fe-b162-1eb0af381ca2.png" alt="Signal timing for reading data in configuration mode">

Signal timing for reading data in normal mode:

<img src="https://cdn.nlark.com/yuque/0/2023/png/33745167/1689513935030-c74473d3-f812-48ee-abff-5139ec75a307.png" alt="Signal timing for reading data in normal mode">

### Speed Two's Complement Conversion
The speed value solved by the resolver decoder AD2S1210 is stored in speed registers 0x82 and 0x83 in the form of a 16-bit binary two's complement. According to the agreement for reading position and speed in AD2S1210's normal mode, the `unsigned char buf[4]` returned by the function `ReadFromAD2S1210(VELOCITY, POS_VEL, buf)` contains a 24-bit wide shift register value. Among them, `buf[2]` to `buf[1]` contain the 16-bit binary two's complement representing speed in MSB-first order (MSB is in `buf[2]`), and `buf[0]` contains data from the fault register. The meaning of each bit of the fault register is shown in the following table:

<img src="https://cdn.nlark.com/yuque/0/2024/png/33745167/1713863886694-7a409be6-73cd-4f21-87e2-2079eaa05b05.png" alt="Fault register bit meanings">

In this case, the resolution of AD2S1210 is set to 12 bits. At this time, in the 16-bit speed data composed of `buf[2]` and `buf[1]`, only bits 15 to 4 provide valid data, while bits 3 to 0 should be ignored. Therefore, to represent the speed using a 12-bit number, it is necessary to first shift `buf[2]` left by 8 bits, add it to `buf[1]`, and then shift the entire result right by 4 bits:

```cpp
velocity0 = ((buf[2] << 8) | buf[1])>>4;
```

Next, we need to convert the binary two's complement to the original code. The binary two's complement should be understood according to the following case: assuming the resolution of AD2S1210 is 12 bits, the modulus is $ 2^{12}=4096\text{D} $, and the range of signed original codes that can be represented by the two's complement is $ [-2048\text{D},2047\text{D}] $. When we need to calculate the two's complement corresponding to the original code -170D, we can calculate it according to the following formula:

$ \text{Negative two's complement}=\text{Modulus - Absolute value of negative original code}=4096\text{D}-170\text{D}=3926\text{D}=\text{0xF}56 $. At this time, the sign bit should be included in the conversion between binary and decimal.

Conversely, $ \text{Absolute value of negative original code}=\text{Modulus - Negative two's complement} $:

```cpp
velocity = 4096 - velocity0;
```

First, we judge the sign of the two's complement. If the two's complement is negative, execute the following program:

```cpp
if ((velocity0 & 0x800)>>11)
{
 velocity = 4096 - velocity0;
 velocity = velocity*1000/2048;
 printf("rps: -%f\n",velocity);
 printf("error: %X\n",buf[0]);
}
```

Among them, `velocity = velocity*1000/2048` is to calculate the absolute value of the actual speed represented in decimal. The full-scale range refers to the following table:

<img src="https://cdn.nlark.com/yuque/0/2024/png/33745167/1713856888221-6b64efc9-1228-4d7e-991c-06a709070db1.png" alt="AD2S1210 full-scale range">

Correspondingly, if the two's complement is positive, the original code is the two's complement itself, and the following program is executed:

```cpp
else
{
 velocity = velocity0;
 velocity = velocity*1000/2048;
 printf("rps: %f\n",velocity);
 printf("error: %X\n",buf[0]);
}
```

In addition, binary numbers in computers are stored in two's complement form. For example, when inverting 0x7FF bit by bit, first supplement it to a 12-bit binary number 011111111111, then invert each bit to get 100000000000. For this value, the computer output is -2048. This is because the computer treats it as a two's complement, with the highest bit as the sign bit. The decimal value of the original code corresponding to the negative two's complement 100000000000 is -2048. Note that the highest bit 0 should not be discarded during bit operations. It is recommended to treat ~x as -(x+1).

## CAN Bus Configuration
This project uses CAN-FD, which supports transmitting 64 bytes of data in one frame. However, due to the limitation that the CAN transceiver chip TJA1050 used does not support this function, it can still only transmit 8 bytes of data.

<img src="https://cdn.nlark.com/yuque/0/2024/png/33745167/1714484801478-24be0570-1941-41c2-8f93-ae863cfd617f.png" alt="CAN bus data transmission">

Here, we use the CAN bus to send the motor's position and speed information to the motor control board. Both data types are 32-bit floating-point numbers, which exactly occupy 8 bytes.

### Baud Rate Matching
In the `main` function of the resolver decoding board, the CAN bus configuration program is as follows:

```cpp
/*** 初始化 ***/
HAL_Init();                           /* 初始化HAL库 */
sys_stm32_clock_init(85, 2, 2, 4, 8); /* 设置时钟,170Mhz */
delay_init(170);                      /* 延时初始化 */

usart_init(921600);                   /* 串口初始化波特率 */	

fdcan_init(17, 8, 2, 7, FDCAN_MODE_NORMAL);     /* FDCAN初始化*/
```

Among them:

```cpp
uint8_t fdcan_init(uint16_t presc, uint8_t ntsjw, uint16_t ntsg1, uint16_t ntsg2, uint32_t mode)
```

In the motor control board, the CAN bus configuration program is:

```cpp
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 16;
  hfdcan1.Init.DataSyncJumpWidth = 8;
  hfdcan1.Init.DataTimeSeg1 = 7;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */
}

```

The formula for calculating the baud rate is:

$ \text{Baud rate}=\frac{\text{Clock frequency}/\text{presc}}{\text{ntsjw}+\text{ntsg1}+\text{ntsg2}} $

After calculation, the two match:

$ \frac{\text{170M}/\text{17}}{\text{8}+\text{2}+\text{7}}=\frac{\text{160M}/\text{16}}{\text{8}+\text{7}+\text{2}} $

### Resolver Decoding Board Transmission Program
```cpp
void CAN_SendTwoFloats(float value1, float value2) {

    // 将第一个浮点数的4字节二进制表示放入缓冲区
    memcpy(&buffer[0], &value1, sizeof(float));

    // 将第二个浮点数的4字节二进制表示放入缓冲区
    memcpy(&buffer[4], &value2, sizeof(float));
	
    fdcan1_send_msg(buffer, FDCAN_DLC_BYTES_8); /* 发送8个字节 */
}
```

### Motor Control Board Receiving Program
```cpp
/* 读取角度 [0, 2PI] */  
memcpy(&position, &RxData[0], sizeof(float)); // rad

/* 读取旋转变压器测得的速度 (rad/s) */
memcpy(&velocity, &RxData[4], sizeof(float)); // rad/s
```

## Data Direction Matching
For the motor control algorithm, when viewed from the resolver installation position at the tail of the motor, counterclockwise rotation is the positive direction. As for the electrical characteristics of the resolver itself, clockwise is positive. Therefore, before transmission, it is necessary to convert the data direction:

```cpp
position1 = 360 - position; //算法逆时针为正，旋变顺时针为正
velocity1 = -1 * velocity;  //算法逆时针为正，旋变顺时针为正
```

# Existing Problems
## Zero Angle Problem
The 0 angle of the resolver (resolver electrical 0 position) is inconsistent with the 0 angle of the motor rotor (the angle when the rotor coincides with the stator a-axis), resulting in a deviation in the actual electrical angle of the motor in motor control.

For details: [Link 1](https://zhuanlan.zhihu.com/p/139287600) (Chinese)

## Computational Efficiency of Clarke and Park Transformations
Due to the trigonometric floating-point calculations in Clarke and Park transformations, the computational efficiency of the microcontroller will be significantly reduced, making it impossible for projects using LibFOC to normally realize motor current control (it may also be that the PWM program has problems).

It is recommended to use the DMA-based CORDIC module to accelerate the operation.

For details: [Link 1](https://blog.csdn.net/zhvngchvng/article/details/131540042) (Chinese), [Link 2](https://www.st.com/resource/en/application_note/dm00614795-getting-started-with-the-cordic-accelerator-using-stm32cubeg4-mcu-package-stmicroelectronics.pdf), [Link 3](https://shequ.stmicroelectronics.cn/thread-635016-1-1.html) (Chinese)

To manually configure the CORDIC module, in addition to adding `cordic.c`, `stm32g4xx_hal_cordic.c`, and `stm32g4xx_hal_II_cordic.c`, it is also necessary to uncomment the #define HAL_CORDIC_MODULE_ENABLED statement in the `stm32g4xx_hal_conf.h` file.

## Update Frequency Problem of Multi-Loop Control
When adopting multi-loop control (such as outer position loop and inner current loop), the update frequency of the outer loop is generally lower than that of the inner loop.

For details: [Link 1](https://www.zhihu.com/question/383770864/answer/3454138454) (Chinese), [Link 2](https://www.zhihu.com/question/404520965) (Chinese)

## Output End Position Control Problem
Since the joint position is output through a harmonic reducer, and the feedback position information is the input of the harmonic reducer (motor position), there is a conversion relationship between motor position control and joint position control. Generally, when the output end of the harmonic reducer rotates one circle, the motor end rotates "reduction ratio + 1" circles.
