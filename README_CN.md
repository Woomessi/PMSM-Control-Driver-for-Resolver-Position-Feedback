常用参考资料：[DengFOC](https://dengfoc.com/#/dengfoc/%E7%81%AF%E5%93%A5%E6%89%8B%E6%8A%8A%E6%89%8B%E6%95%99%E4%BD%A0%E5%86%99FOC%E7%AE%97%E6%B3%95/%E5%BA%8F%E4%B8%BA%E4%BB%80%E4%B9%88%E8%A6%81%E5%87%BA%E8%BF%99%E7%B3%BB%E5%88%97%E8%AF%BE%E7%A8%8B.md)、[欧拉电子 STM32G4 Simulink FOC开发实战](https://space.bilibili.com/458115745/lists/1688763?type=season)

# 电机控制板
## 速度控制
该部分程序源码位于`2.1. 程序\2.1.1. 嵌入式\电机控制板\PWM_LibFOC_速度控制`，其使用的 LibFOC 电机控制库是基于 [DengFOC](https://dengfoc.com/#/dengfoc/%E7%81%AF%E5%93%A5%E6%89%8B%E6%8A%8A%E6%89%8B%E6%95%99%E4%BD%A0%E5%86%99FOC%E7%AE%97%E6%B3%95/%E5%BA%8F%E4%B8%BA%E4%BB%80%E4%B9%88%E8%A6%81%E5%87%BA%E8%BF%99%E7%B3%BB%E5%88%97%E8%AF%BE%E7%A8%8B.md) 以及 [DengFOC_on_STM32](https://github.com/haotianh9/DengFOC_on_STM32) 编写的，程序框架主要基于 CubeMX 进行搭建（HAL 库），工程文件为`2.1. 程序\2.1.1. 嵌入式\电机控制板\PWM_LibFOC_速度控制\MDK-ARM`的`STM32G4_GPIO.uvprojx`文件。

使用 CubeMX 更新工程前，必须保证用户自己编写的代码放置在/* USER CODE BEGIN xx */与/* USER CODE END xx */结构之间，否则会被删除。

该部分程序控制架构为 FOC，电流控制策略为“直轴电流为零”，控制算法为 PID，PWM 控制技术为“正弦脉宽调制”。位置控制相关代码与速度控制类似，可直接参照本节内容。

![电机控制领域常见概念的分类](https://cdn.nlark.com/yuque/0/2025/webp/33745167/1742302430811-43780d8d-265c-4535-8e01-2173e5e24d32.webp)

### 单片机配置
从 `main.c`文件第 143 行主函数 `main()`开始，首先对单片机进行了配置，该部分可参考 [欧拉电子 STM32G4 Simulink FOC开发实战](https://space.bilibili.com/458115745/lists/1688763?type=season) 进行学习，相关源码如下。

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

#### ADC 与 DAC
ADC 相关配置文件见`main.c`文件第 156 行到 157 行，`MX_ADC1_Init()`和 `MX_ADC2_Init()`。本程序中，ADC1 规则组通道 11 用于采集母线电压，其单边采样范围为[0, 3.3 V]，采样周期为 2.5 个时钟周期，时钟分频为 4 分频，采样分辨率为 12 位，数据右对齐，通过软件触发采样。在所设计的控制板卡中，无框力矩电机的母线电压经阻值为 75 kΩ 与 3 kΩ 的电阻分压后，由 3 kΩ 的电阻输入至微控制器的 ADC 接口中，故实际母线电压与 ADC 采样值之间的转换关系为：

![](https://cdn.nlark.com/yuque/0/2025/png/33745167/1742289033349-8646c99e-4007-47fb-a5ad-1e380877cab4.png)

ADC1 注入组通道 3、通道 12，ADC2 注入组通道 3 用于采集三相电流。由于本关节所选用的电机允许通过的最大电流为 61 A，控制板卡使用了标称阻值为 1 mΩ，最大工作电流为 77 A 的采样电阻对电机绕组的相电压进行低端采样。所采得的相电压通过微控制器的内部运算放大器 OPAMP 进行偏置与放大，最后输入到 ADC 接口中。其中，微控制器 OPAMP 的增益通过外部电阻进行配置，本电路将增益值配置为 22/3。微控制器的 ADC 接口与 OPAMP 的输出端进行级联，经 OPAMP 放大后的采样信号通过 ADC 的注入组进行模数转换，注入组的优先级高于规则组，单边采样范围为[0, 3.3 V]，采样周期为 2.5 个时钟周期，采样触发信号来自高级定时器 TIM1。为保证 ADC 的输入电压在单边采样电压范围之内，本控制板卡在微控制器的 OPAMP 接口外使用分压电阻对 ADC 输入电压正向偏置 1.65 V。偏置之后，考虑到 22/3 的 OPAMP 运放增益和 1 mΩ 低端采样电阻的分压，可以得到 ADC 端口输入电压与 ADC 采样值以及相电流之间的关系为：

![](https://cdn.nlark.com/yuque/0/2025/png/33745167/1742289223431-c9d8b2ac-5ed2-4201-8a40-5c32713ef959.png)

在`main.c`文件第 210 行到 213 行，我们设置了 DAC 比较封波，防止相电流过流。若 ADC 电流采样值超过了 `HAL_DAC_SetValue`函数的输入`Data`，则高级定时器 PWM 发波将停止。

#### 高级定时器
在单片机的外设中，高级计数器 TIM1 起到 PWM 发波以及控制程序执行周期的作用。就 TIM1 的配置来说，其预分频器 `PSC` 为 1 分频，重复计数寄存器 `RCR` 的值为 1，即在一个周期内仅触发一次更新事件。

TIM1 通道 1 到通道 3 的工作模式为互补 PWM 输出模式， PWM 模式为 1，计数模式为[中心对齐模式](https://www.zhihu.com/question/419152940)。TIM1 的相关配置程序位于 `main.c`文件第 161 行函数 `MX_TIM1_Init()`中。

为防止全桥模块上下桥臂同时导通， TIM1 的死区时间被设置为 1.5 微秒，对应的`DTG`寄存器的值为 0x78。

$ \begin{align}
\text{DT}&=\text{DTG}[7:0]*t_{dtg}\\
&=120*\frac{1}{160\text{M}}*2(备注：\text{CKD}分频系数）\\
&=1500ns\\
\end{align} $

由 `main.c`文件第 195 行可知，自动装载寄存器 `ARR` 的值为 7999。由此可得 TIM1所发出的 PWM 波的频率$ f_{\text{PWM}} $为：

$ f_{\text{PWM}} =160\text{MHz}/(7999+1)/2=10\text{kHz} $。其中，频率除以 2 是中心对齐模式决定的。

配置为无输出模式的 TIM1 通道 4 主要用于产生 TRGO 信号以触发 ADC 外设的三相电流采样，其 PWM 模式为 2。为了保证采样值的稳定，采样触发时刻三相全桥逆变电路的下桥臂需全部打开。为此，在中心对齐模式中，寄存器 `CCR4` 的值应接近上溢值 7999。由于ADC 存在采样与转换延迟，在`main.c`文件第 196 行中，其大小设为 7998。

### while 循环
从 `main.c`文件第 215 行到第 223 行之间的 while 循环中的函数比较简单，仅用于 ADC 软件触发以及[母线电压采样](https://zhuanlan.zhihu.com/p/495852697)。ADC 转换公式参照本文档`ADC 与 DAC`部分。

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

### ADC 中断回调函数
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

当 TIM1 高级计数器的`CNT`寄存器递增至 `CCR4`比较值时，将触发 ADC 中断回调函数。此时，ADC 注入组完成电流采样，并执行与电机控制直接相关的程序。

#### 程序执行周期控制
该部分程序常见以下结构：

```cpp
count++;
if (count >= steps)
{
    /* 电机控制程序 */
    /*   ......    */
    count = 0;
}
```

这是为了控制相关程序的执行周期。比如，设 `steps = 100`，那么，ADC 中断回调函数每执行 100 次，才会运行一次该 if 结构中的程序。该结构多用于调整多级控制（如位置环+电流环）不同回路的运行频率（[链接 1](https://www.zhihu.com/question/383770864/answer/3454138454)，[链接 2](https://www.cnblogs.com/zhongzhe/archive/2012/11/14/2770781.html)），或限制通过串口向上位机发送数据的频率（这一过程耗时较长，影响算法执行效率）。VOFA 上位机串口协议教程参考 [链接](https://www.bilibili.com/video/BV1nH4y1X72k?spm_id_from=333.788.videopod.sections&vd_source=ad856f51618186902c24682a8c8279ff)。

#### 低通滤波
在电机控制中，传感器测量到的速度数据往往存在较大的波动（对位置数据微分进而求解速度导致）。为保证电机平稳运转，往往需要对速度数据进行[低通滤波](https://www.zhihu.com/question/518024588/answer/3364056229)。离散空间中的低通滤波可视为将当前时刻采集的数据与上一时刻采集的数据进行加权求和，使数据平滑。

![低通滤波前的速度数据](https://cdn.nlark.com/yuque/0/2025/png/33745167/1742301893237-142a47e9-c4ad-4f06-9b17-b14deebce00f.png)![低通滤波后的速度数据](https://cdn.nlark.com/yuque/0/2025/png/33745167/1742301893193-813a6be1-2f27-480c-8acc-be5374d5afd4.png)

#### 逆变换
`setPhaseVoltage`函数 将 q 轴电压转换为三相电压，其程序为：

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

在该部分程序中，我们将控制器输出的 Uq 大小限制在`±母线电压/2`范围内，并在克拉克逆变换中将相电压整体上移 `母线电压/2`，这是由 PWM 调制的特性决定的。详细讲解请参照 [链接](https://zhuanlan.zhihu.com/p/99976227)。



## 电流控制
该部分程序源码位于`2.1. 程序\2.1.1. 嵌入式\电机控制板\SVPWM_MATLAB_电流环`，其使用的 MATLAB 电机控制库是基于 `交接材料_吴宏瑞\2. 工程文件\2.1. 程序\2.1.2. Simulink仿真及程序生成\IF` 中的 svpwm 子系统，利用 Embedded Coder 生成的（相关教程可参照《STM32G4 Simulink FOC开发套件用户手册》第十三章，以及相对应的[视频](https://www.bilibili.com/video/BV12C4y1f78j/?spm_id_from=333.1387.collection.video_card.click&vd_source=ad856f51618186902c24682a8c8279ff)），程序框架主要基于 CubeMX 进行搭建（HAL 库），工程文件为`交接材料_吴宏瑞\2. 工程文件\2.1. 程序\2.1.1. 嵌入式\电机控制板\SVPWM_MATLAB_电流环\MDK-ARM`的`STM32G4_GPIO.uvprojx`文件。

![svpwm 子系统](https://cdn.nlark.com/yuque/0/2025/png/33745167/1742361984092-4a7872d5-4eb9-4c5f-b7d4-07fe5801d450.png)

该部分程序控制架构为 FOC，电流控制策略为“直轴电流为零”，控制算法为 PID，PWM 控制技术为“空间矢量脉宽调制 SVPWM”。

### ADC 中断回调函数
电流控制程序与使用 LibFOC 的速度控制程序的主要区别，是在ADC 中断回调函数，对电机的三相电流进行了采集。

#### 电流采样偏置值计算
首先，为补偿 ADC 的零漂，在主要控制程序执行前，对电流采样偏置值进行了计算。

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

以上对偏置值的计算仅执行一次。接下来，根据 ADC 采样值与偏置值，对相电流进行了计算，并作为电流控制过程中的反馈信号，输入至控制算法中。

#### 电流控制注意事项
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

![全局宏定义](https://cdn.nlark.com/yuque/0/2025/png/33745167/1742364165223-214e3aa5-4096-4c45-a1d8-e24986f16801.png)

使用 Embedded Coder 将 Simulink 子系统转化为可以直接运行的嵌入式程序后，子系统的输入在 `rtU`结构体中进行定义，输出在 `rtY`结构体中进行定义，主程序通过`xxx_step()`函数执行。

:::danger
需要注意的是，若使用自主研发的电机控制板 v2.1，而非欧拉电子提供的电机控制板，除配置 GPIO 以及外设外，需要在 Keil 的全局宏定义中将`STM32G431xx`替换为 `STM32G474xx`，同时需要将ADC 采样值与偏置值之差到相电流的换算增益修改为0.10986328125f。

嵌入式程序中，在[浮点数后加 f](http://www.openedv.com/thread-40175-1-1.html) 可提升计算效率。

:::

# 旋变解码板
该部分程序源码位于`2.1. 程序\2.1.1. 嵌入式\旋变解码板`，程序框架主要基于正点原子的工程模板进行搭建（HAL 库），工程文件为`交接材料_吴宏瑞\2. 工程文件\2.1. 程序\2.1.1. 嵌入式\旋变解码板\Board_2.2_CAN_tune\Projects\MDK-ARM`的`atk_g474.uvprojx`文件。

旋变解码板主要基于 [AD2S1210](https://www.analog.com/cn/products/ad2s1210.html) 芯片，将旋转变压器输出的模拟电压信号转换为数字位置与速度信号，由软件模拟 SPI 总线（通过 GPIO 口直接模拟 SPI 协议的高低电平时序）进行读取，再通过 CAN 总线发送至电机控制板。

## AD2S1210 驱动程序
该部分程序主要基于软件 SPI 实现 STM32 与 AD2S1210 之间的读写，可参考 [链接](https://www.analog.com/cn/products/ad2s1210.html#software-resources) 中的软件资源->AD2S1210 参考代码部分。

### 常用函数
:::danger
要格外注意驱动程序中延时函数 `delay` 的实际延时时间。一方面，它会影响驱动的时序；另一方面，它会对程序的执行效率带来影响。

:::

#### 初始化函数
上电时，AD2S1210对各引脚的时序有一定的要求：

![](https://cdn.nlark.com/yuque/0/2023/png/33745167/1689497865455-b4570e85-0b02-4c26-8e35-173553fa57be.png)

其中，$ \text{t}_\text{RST} $至少为$ 10\mu $秒，$ \text{t}_\text{TRACK} $的长短与分辨率有关：

![](https://cdn.nlark.com/yuque/0/2023/png/33745167/1689498333563-6c62b475-b1f6-4ad0-b4a4-6045971973aa.png)

据此，我们编写了AD2S1210的初始化函数：

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

#### 工作模式选择函数
拉低或拉高`A0`和`A1`两个引脚可以选择AD2S1210的工作模式，我们编写了相应的函数来实现该功能：

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

#### 写函数
向AD2S1210的片内寄存器写入数据，是在配置模式下完成的。首先需要向SPI接口发送8位寄存器地址，再发送8位数据。本项目直接使用软件 SPI 写函数。每发送一次数据后，需要拉高`WR#`引脚，利用上升沿锁存数据。

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

相应信号时序如下：

![](https://cdn.nlark.com/yuque/0/2023/png/33745167/1689513847865-b88101a4-3073-42c1-abbe-9eb9b28bea7f.png)

#### 读函数
无论是普通模式还是配置模式，都可以读取 AD2S1210 的位置寄存器或速度寄存器，而要读取其他寄存器，则只能在配置模式下进行。在配置模式下读取数据，我们需要向 SPI 接口发送目标寄存器的地址。在读取位置寄存器或速度寄存器中的数据前，需要先拉高，再拉低`SAMPLE#`引脚，以更新数据。

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

配置模式下读取数据的信号时序：

![](https://cdn.nlark.com/yuque/0/2023/png/33745167/1689513904592-947b64f0-4943-41fe-b162-1eb0af381ca2.png)

普通模式下读取数据的信号时序：

![](https://cdn.nlark.com/yuque/0/2023/png/33745167/1689513935030-c74473d3-f812-48ee-abff-5139ec75a307.png)

### 速度补码转换
旋变解码器 AD2S1210 所解得的速度值以 16 位二进制补码的形式储存于速度寄存器 0x82 与 0x83。按照 AD2S1210 在普通模式下进行位置与速度读取的约定，使用函数 `ReadFromAD2S1210(VELOCITY, POS_VEL, buf)`返回的 `unsigned char buf[4]` 中存有 24 位宽的移位寄存器值。其中，`buf[2]`到 `buf[1]`以 MSB 优先的方式（MSB 位于`buf[2]`）存有表示速度的16 位二进制补码，`buf[0]`存有故障寄存器的数据。故障寄存器各位含义如下表所示：

![](https://cdn.nlark.com/yuque/0/2024/png/33745167/1713863886694-7a409be6-73cd-4f21-87e2-2079eaa05b05.png)

本案例中，AD2S1210 的分辨率被设为 12 位。这时，在`buf[2]`和`buf[1]`组成的 16 位速度数据中，仅位 15 到位 4 提供有效数据，位 3 到位 0 则应忽略。因此，要使用 12 进制数表示速度，应先将`buf[2]`左移 8 位后与`buf[1]`相加，再整体右移 4 位：

```cpp
velocity0 = ((buf[2] << 8) | buf[1])>>4;
```

接下来，我们需要将二进制补码转换为原码。二进制补码应按以下案例进行理解：假设 AD2S1210 的分辨率为 12 位，那么模为$ 2^{12}=4096\text{D} $，补码所能表示的带符号原码的区间为$ [-2048\text{D},2047\text{D}] $。当我们需要计算原码-170D 所对应的补码时，可按以下公式计算

$ \text{负数补码}=\text{模-负数原码的绝对值}=4096\text{D}-170\text{D}=3926\text{D}=\text{0xF}56 $。此时，符号位应被包括到二进制与十进制的转换中。

反过来，$ 负数原码的绝对值=模-负数的补码 $：

```cpp
velocity = 4096 - velocity0;
```

首先，我们判断补码的符号，若补码为负，则执行以下程序：

```cpp
if ((velocity0 & 0x800)>>11)
{
 velocity = 4096 - velocity0;
 velocity = velocity*1000/2048;
 printf("rps: -%f\n",velocity);
 printf("error: %X\n",buf[0]);
}
```

其中，`velocity = velocity*1000/2048`是计算用十进制表示的实际速度的绝对值。满量程范围参考下表：

![](https://cdn.nlark.com/yuque/0/2024/png/33745167/1713856888221-6b64efc9-1228-4d7e-991c-06a709070db1.png)

相应地，若补码为正，则原码即为补码本身，执行以下程序：

```cpp
else
{
 velocity = velocity0;
 velocity = velocity*1000/2048;
 printf("rps: %f\n",velocity);
 printf("error: %X\n",buf[0]);
}
```

另外，计算机中的二进制数以补码的形式存储。比如，对 0x7FF 按位取反，首先将其补充为12位二进制数 011111111111，再按位取反，得到 100000000000。对于该值，计算机的输出为-2048。这是由于计算机将其视为补码，最高位视为符号位。负数补码 100000000000 所对应的原码的十进制为-2048。注意位与时不要舍去最高位的 0。建议将~x 视为-(x+1)。

## CAN 总线配置
本项目采用 CAN-FD ，支持一帧传输 64 字节的数据。然而，限于所使用的 CAN 收发芯片 TJA1050 不支持该功能，故仍然仅能传输 8 字节的数据。

![](https://cdn.nlark.com/yuque/0/2024/png/33745167/1714484801478-24be0570-1941-41c2-8f93-ae863cfd617f.png)

在这里，我们使用 CAN 总线向电机控制板发送电机的位置与速度信息，它们的数据类型都是 32 位浮点数，刚好占满 8 字节。

### 波特率匹配
在旋变解码板的`main`函数中，CAN 总线配置程序如下：

```cpp
/*** 初始化 ***/
HAL_Init();                           /* 初始化HAL库 */
sys_stm32_clock_init(85, 2, 2, 4, 8); /* 设置时钟,170Mhz */
delay_init(170);                      /* 延时初始化 */

usart_init(921600);                   /* 串口初始化波特率 */	

fdcan_init(17, 8, 2, 7, FDCAN_MODE_NORMAL);     /* FDCAN初始化*/
```

其中：

```cpp
uint8_t fdcan_init(uint16_t presc, uint8_t ntsjw, uint16_t ntsg1, uint16_t ntsg2, uint32_t mode)
```

在电机控制板的中，CAN 总线配置程序为：

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

波特率的计算公式为：

$ 波特率=\frac{\text{时钟频率}/\text{presc}}{\text{ntsjw}+\text{ntsg1}+\text{ntsg2}} $

经计算，两者匹配：

$ \frac{\text{170M}/\text{17}}{\text{8}+\text{2}+\text{7}}=\frac{\text{160M}/\text{16}}{\text{8}+\text{7}+\text{2}} $

### 旋变解码板发送程序
```cpp
void CAN_SendTwoFloats(float value1, float value2) {

    // 将第一个浮点数的4字节二进制表示放入缓冲区
    memcpy(&buffer[0], &value1, sizeof(float));

    // 将第二个浮点数的4字节二进制表示放入缓冲区
    memcpy(&buffer[4], &value2, sizeof(float));
	
    fdcan1_send_msg(buffer, FDCAN_DLC_BYTES_8); /* 发送8个字节 */
}
```

### 电机控制板接收程序
```cpp
/* 读取角度 [0, 2PI] */  
memcpy(&position, &RxData[0], sizeof(float)); // rad

/* 读取旋转变压器测得的速度 (rad/s) */
memcpy(&velocity, &RxData[4], sizeof(float)); // rad/s
```

## 数据方向匹配
对电机控制算法而言，从电机尾部旋变安装位置看去，逆时针旋转为正方向。而就旋转变压器本身的电气特性而言，顺时针为正。故在传递之前，需对数据方向进行转换：

```cpp
position1 = 360 - position; //算法逆时针为正，旋变顺时针为正
velocity1 = -1 * velocity;  //算法逆时针为正，旋变顺时针为正
```

# 现有问题
## 零位角问题
旋转变压器的 0 角度（旋变电气 0 位置）与电机转子的 0 角度（转子与定子 a 轴重合时的角度）不一致，导致电机控制中电机实际的电角度有偏差。

详见：[链接1](https://zhuanlan.zhihu.com/p/139287600)

## 克拉克变换与帕克变换计算效率问题
由于克拉克变换与帕克变换中包含三角函数浮点计算问题，单片机的计算效率将大幅降低，导致使用 LibFOC 的工程无法正常实现电机的电流控制（也有可能是 PWM 程序有问题）。

建议使用基于 DMA 的 CORDIC 模块，以加速运算。

详见：[链接1](https://blog.csdn.net/zhvngchvng/article/details/131540042)、[链接2](https://www.st.com/resource/en/application_note/dm00614795-getting-started-with-the-cordic-accelerator-using-stm32cubeg4-mcu-package-stmicroelectronics.pdf)、[链接3](https://shequ.stmicroelectronics.cn/thread-635016-1-1.html)

手动配置 CORDIC 模块，除需添加 `cordic.c`、`stm32g4xx_hal_cordic.c`、`stm32g4xx_hal_II_cordic.c`之外，还需在 `stm32g4xx_hal_conf.h`文件中，取消 #define HAL_CORDIC_MODULE_ENABLED 语句的注释。

## 多环控制更新频率问题
当采用多环控制时（如外环位置环，内环电流环），外环的更新频率一般低于内环的更新频率。

详见：[链接1](https://www.zhihu.com/question/383770864/answer/3454138454)、[链接2](https://www.zhihu.com/question/404520965)

## 输出端位置控制问题
由于关节位置通过谐波减速器输出，而反馈回的位置信息是谐波减速器的输入（电机位置），电机位置控制与关节位置控制之间存在一个转换关系。一般地，当谐波减速器输出端转过一圈时，电机端转过“减速比 + 1”圈。
