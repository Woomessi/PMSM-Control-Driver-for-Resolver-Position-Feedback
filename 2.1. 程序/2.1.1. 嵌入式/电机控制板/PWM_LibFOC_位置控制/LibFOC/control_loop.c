#include "control_loop.h"
#include "arm_math.h"

extern float DIR;
extern float PP;
extern int ARR;
extern float zero_electric_angle;
extern float voltage_limit;
extern float voltage_power_supply;

float position_previous = 0;     // 在getAngle函数中，表示上一周期测得的角度
float cnt_turns = 0;             // 在getAngle函数中，表示电机当前转过的圈数

float output = 0;
float output_prev1 = 0;
float output_prev2 = 0;
float output_prev3 = 0;

float input_prev1 = 0;
float input_prev2 = 0;
float input_prev3 = 0;

/* 
 *******************************************************************************************
 * 名称        : 【setPhaseVoltage】 *       
 *************************************
 * 功能        : 根据dq轴电压，计算三相电压
 * 作者        : https://dengfoc.com/#/dengfoc/DengFOC%E5%BA%93%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3/1%E6%A6%82%E8%BF%B0
 *******************************************************************************************
 * 备注 *
 ********
 * 三相电压计算可参照基本的帕克变换与克拉克变换。
 * 由于使用 PWM 进行调制，三相电压的范围为[-(voltage_power_supply)/2,(voltage_power_supply)/2]
 * 实际电路无法生成负电压，故须在克拉克逆变换后将三相电压上移 voltage_power_supply/2
 *******************************************************************************************
 * 参考链接 *
 ************
 * 链接1: https://zhuanlan.zhihu.com/p/99976227
 * 链接2: https://dengfoc.com/#/dengfoc/%E7%81%AF%E5%93%A5%E6%89%8B%E6%8A%8A%E6%89%8B%E6%95%99%E4%BD%A0%E5%86%99FOC%E7%AE%97%E6%B3%95/4.2FOC%E5%BC%80%E7%8E%AF%E9%80%9F%E5%BA%A6%E4%BB%A3%E7%A0%81%E7%9A%84%E5%8E%9F%E7%90%86%E6%95%99%E5%AD%A6
 *******************************************************************************************
*/

void setPhaseVoltage(float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE) {
	
	Uq = _constrain(Uq,-(voltage_power_supply)/2,(voltage_power_supply)/2);
	
  /* 帕克逆变换(不考虑Ud）*/ 
	float Ualpha =  -Uq*sin_f((angle_el / PI * 180));
  float Ubeta  =   Uq*cos_f((angle_el / PI * 180));
		
  /* 克拉克逆变换 */
  float Ua =                    Ualpha + voltage_power_supply/2;
  float Ub = (_SQRT3*Ubeta - Ualpha)/2 + voltage_power_supply/2;
  float Uc = (-Ualpha -_SQRT3*Ubeta)/2 + voltage_power_supply/2;
	
  setPWM(Ua,Ub,Uc,TIM_BASE);
}


/* 
 *****************************************************************************************
 * 名称        : 【setPWM】 *
 ****************************
 * 功能        : 根据所需的三相电压，利用PWM算法调整高级定时器的CCR寄存器值，驱动电机
 * 作者        : https://dengfoc.com/#/dengfoc/DengFOC%E5%BA%93%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3/1%E6%A6%82%E8%BF%B0
 *****************************************************************************************
*/
void setPWM(float Ua, float Ub, float Uc, TIM_TypeDef * TIM_BASE) {

  /* 限制三相电压的大小 */
	Ua = _constrain(Ua, 0.0f, voltage_limit);
	Ub = _constrain(Ub, 0.0f, voltage_limit);
	Uc = _constrain(Uc, 0.0f, voltage_limit);
	
	/* 占空比计算 */
	float dutyCycle_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
	float dutyCycle_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
	float dutyCycle_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

	/* 高级定时器CCR寄存器值计算 */
	TIM_BASE->CCR1 = (float) roundf(dutyCycle_a*ARR);
	TIM_BASE->CCR2 = (float) roundf(dutyCycle_b*ARR);
	TIM_BASE->CCR3 = (float) roundf(dutyCycle_c*ARR);
}

/* 
 *****************************************************************************************
 * 名称        : 【getFullAngle】
 *******************************
 * 功能        : 根据不带圈数的角度数据 position 计算带圈数的角度（可大于2*PI）
 * 作者        : https://dengfoc.com/#/dengfoc/DengFOC%E5%BA%93%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3/1%E6%A6%82%E8%BF%B0
 *****************************************************************************************
 * 备注 *
 ********
 * 当转子转过一整圈时，角度传感器所测得的不带圈数的角度将发生突变（例如：从2*PI突变至0）。   
 * 通过分析角度变化量 position_delta 的大小与正负，可对电机转子转过的圈数 cnt_turns 进行测算。    
 * 如 position_delta 的绝对值超过一定阈值（本程序为2*PI的80%），则认为转子转过了一圈。 
 * 再通过判断 position_delta 的符号，确定旋转的方向:
 * 正转时，所测得的角度由接近2*PI突变至接近0，position_delta 符号为负；反之，符号为正。
 *****************************************************************************************
*/
float getFullAngle(float position) // position为不带圈数的角度（rad），范围为[0,2*PI]
{
    float position_delta = position - position_previous; // 求当前测得的角度与上一周期测得的角度之间的变化量
    if(fabs(position_delta) > (0.8f*6.28318530718f) ) cnt_turns += ( position_delta > 0 ) ? -1 : 1; 
    position_previous = position;
    return (float)cnt_turns * 6.28318530718f + position_previous;
}


/* 
 *****************************************************************************************
 * 名称        : 【cal_Iq】
 *************************
 * 功能        : 对三相电流进行Clark变换与Park变换，求q轴电流
 * 作者        : https://dengfoc.com/#/dengfoc/DengFOC%E5%BA%93%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3/1%E6%A6%82%E8%BF%B0
 *****************************************************************************************
 * 备注 *
 ********
 * 运用基尔霍夫电流定律，只使用了两相电流。   
 * 使用CORDIC模块加速三角函数计算。建议后续探索使用DMA配置CORDIC，进一步提速。
 *****************************************************************************************
*/
float cal_Iq(float current_a,float current_b,float position_e)
{
  float I_alpha = current_a;
  float I_beta  = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

	float ct = cos_f((position_e / PI * 180));
	float st = sin_f((position_e / PI * 180));
		  
	float I_q = I_beta * ct - I_alpha * st;
  return I_q;
}

/* 
 *****************************************************************************************
 * 名称        : 【cal_Id】
 *************************
 * 功能        : 对三相电流进行Clark变换与Park变换，求d轴电流
 * 作者        : https://dengfoc.com/#/dengfoc/DengFOC%E5%BA%93%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3/1%E6%A6%82%E8%BF%B0
 *****************************************************************************************
 * 备注 *
 ********
 * 运用基尔霍夫电流定律，只使用了两相电流。   
 * 使用CORDIC模块加速三角函数计算。建议后续探索使用DMA配置CORDIC，进一步提速。
 *****************************************************************************************
*/
float cal_Id(float current_a,float current_b,float position_e)
{
  float I_alpha = current_a;
  float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

	float ct = cos_f((position_e / PI * 180));
	float st = sin_f((position_e / PI * 180));
	
	float I_d = I_alpha * ct + I_beta * st;
  return I_d;
}

/* 
 *****************************************************************************************
 * 名称        : 【_electricalAngle】
 ***********************************
 * 功能        : 求电角度
 * 作者        : https://dengfoc.com/#/dengfoc/DengFOC%E5%BA%93%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3/1%E6%A6%82%E8%BF%B0
 *****************************************************************************************
 * 备注 *
 ********
 * 包含了旋变零位角校准。
 *****************************************************************************************
*/
float _electricalAngle(float position){
  return  _normalizeAngle(((float)(DIR * PP) * position) - zero_electric_angle);
}

/* 
 **********************************************************************************************************
 * 名称        : 【_normalizeAngle】
 **********************************
 * 功能        : 将角度归一化至[0,2*PI]范围内
 * 作者        : https://dengfoc.com/#/dengfoc/DengFOC%E5%BA%93%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3/1%E6%A6%82%E8%BF%B0
 **********************************************************************************************************
 * 备注 *
 ********
 * 三目运算符。格式：condition ? expr1 : expr2
 * 其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。
 * 可以将三目运算符视为 if-else 语句的简化形式。
 **********************************************************************************************************
 * fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2M_PI 的符号相反。
 * 也就是说，如果 angle 的值小于 0 且 _2M_PI 的值为正数，则 fmod(angle, _2M_PI) 的余数将为负数。
 * 例如，当 angle 的值为 -PI/2，_2M_PI 的值为 2M_PI 时，fmod(angle, _2M_PI) 将返回一个负数。
 * 在这种情况下，可以通过将负数的余数加上 _2M_PI 来将角度归一化到 [0, 2M_PI] 的范围内，以确保角度的值始终为正数。
 **********************************************************************************************************
*/
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算用于归一化
  return a >= 0 ? a : (a + 2*PI);
}

/* 
 **********************************************************************************************************
 * 名称        : 【Sin_CORDIC_INT】
 *********************************
 * 功能        : CORDIC正弦函数初始化
 * 作者        : https://blog.csdn.net/weixin_42550185/article/details/141894401
 **********************************************************************************************************
*/
void Sin_CORDIC_INT    (void)
{
    CORDIC_HandleTypeDef hcordic;                             //三角函数描述结构体
    CORDIC_ConfigTypeDef sCordicConfig;                       //参数配置结构体
  __HAL_RCC_CORDIC_CLK_ENABLE();                              //开启时钟
    
    hcordic.Instance = CORDIC;                                //选择三角函数计算单元
  HAL_CORDIC_Init(&hcordic);                                  //初始化
    
    sCordicConfig.Function         = CORDIC_FUNCTION_SINE;    //选择计算正弦
  sCordicConfig.Precision        = CORDIC_PRECISION_6CYCLES;  //选择计算精度等级
  sCordicConfig.Scale            = CORDIC_SCALE_0;            //选择计算系数
  sCordicConfig.NbWrite          = CORDIC_NBWRITE_1;          //选择计算结果个数
  sCordicConfig.NbRead           = CORDIC_NBREAD_1;           //选择输出正弦
  sCordicConfig.InSize           = CORDIC_INSIZE_32BITS;
  //选择输入数据格式Q1.31，在Q1.31格式的数字范围：-1 (0x80000000) to 1 至 2^(-31) (0x7FFFFFFF).
  sCordicConfig.OutSize          = CORDIC_OUTSIZE_32BITS;    //选择数据输出格式Q1.31
  HAL_CORDIC_Configure(&hcordic, &sCordicConfig);            //初始化
}


/* 
 **********************************************************************************************************
 * 名称        : 【Cos_CORDIC_INT】
 *********************************
 * 功能        : CORDIC余弦函数初始化
 * 作者        : https://blog.csdn.net/weixin_42550185/article/details/141894401
 **********************************************************************************************************
*/
void Cos_CORDIC_INT    (void)
{
    CORDIC_HandleTypeDef hcordic;                            //三角函数描述结构体
    CORDIC_ConfigTypeDef sCordicConfig;                      //参数配置结构体
  __HAL_RCC_CORDIC_CLK_ENABLE();                             //开启时钟
    
    hcordic.Instance = CORDIC;                               //选择三角函数计算单元
  HAL_CORDIC_Init(&hcordic);                                 //初始化
    
    sCordicConfig.Function         = CORDIC_FUNCTION_COSINE; //选择计算余弦
  sCordicConfig.Precision        = CORDIC_PRECISION_6CYCLES; //选择计算精度等级
  sCordicConfig.Scale            = CORDIC_SCALE_0;           //选择计算系数
  sCordicConfig.NbWrite          = CORDIC_NBWRITE_1;         //选择计算结果个数
  sCordicConfig.NbRead           = CORDIC_NBREAD_1;          //选择输出余弦
  sCordicConfig.InSize           = CORDIC_INSIZE_32BITS;     //选择输入数据格式Q1.31
  sCordicConfig.OutSize          = CORDIC_OUTSIZE_32BITS;    //选择数据输出格式Q1.31
  HAL_CORDIC_Configure(&hcordic, &sCordicConfig);            //初始化
}

/* 
 *****************************************************************************************
 * 名称        : 【sin_f】
 ************************
 * 功能        : CORDIC正弦函数
 * 作者        : https://blog.csdn.net/weixin_42550185/article/details/141894401
 *****************************************************************************************
 * 备注 *
 ********
 * 0<=angles<360,返回值在-1和1之间
 * 主频170MHz时，本函数执行时间330ns
 *****************************************************************************************
*/
float sin_f(float angles)
{
    MODIFY_REG(CORDIC->CSR,CORDIC_CSR_FUNC|CORDIC_CSR_SCALE,CORDIC_FUNCTION_SINE|CORDIC_SCALE_0);
    //选择计算类型:CORDIC_FUNCTION_SINE
    WRITE_REG(CORDIC->WDATA, (int32_t)((180.0f-angles)*11930464.7f));
    //小于180度为正数，大于180度为负数，乘以11930464.7就转换成“q1.31格式”的数据
    //写入CORDIC_WDATA寄存器后，就可以读取“CORDIC_RDATA寄存器的数据”
    //由于“模为0x80000000”，0x80000000/180=2147483648/180=11930464.7

    return (int32_t)READ_REG(CORDIC->RDATA)/2147483648.0f;
    //读取CORDIC_RDATA寄存器的数据是“q1.31格式”的数据，经过转换后，就是正弦值
    //由于“模为0x80000000”，也就是2147483648，除以“模”后就得到正弦值，范围为[-1,1]
}

/* 
 *****************************************************************************************
 * 名称        : 【cos_f】
 ************************
 * 功能        : CORDIC余弦函数
 * 作者        : https://blog.csdn.net/weixin_42550185/article/details/141894401
 *****************************************************************************************
 * 备注 *
 ********
 * 0<=angles<360,返回值在-1和1之间
 * 主频170MHz时，本函数执行时间330ns
 *****************************************************************************************
*/
float cos_f(float angles)
{
    MODIFY_REG(CORDIC->CSR,CORDIC_CSR_FUNC|CORDIC_CSR_SCALE,CORDIC_FUNCTION_COSINE|CORDIC_SCALE_0);
    //选择计算类型:CORDIC_FUNCTION_COSINE

    WRITE_REG(CORDIC->WDATA, (int32_t)((180.0f-angles)*11930464.7f));
    //小于180度为正数，大于180度为负数，乘以11930464.7就转换成“q1.31格式”的数据
    //写入CORDIC_WDATA寄存器后，就可以读取“CORDIC_RDATA寄存器的数据”
    //由于“模为0x80000000”，0x80000000/180=2147483648/180=11930464.7

    return -(int32_t)READ_REG(CORDIC->RDATA)/2147483648.0f;
    //读取CORDIC_RDATA寄存器的数据是“q1.31格式”的数据，经过转换后，就是余弦值
    //由于“模为0x80000000”，也就是2147483648，除以“模”后就得到正弦值，范围为[-1,1]
}

/* 
 *****************************************************************************************
 * 名称        : 【thirdOrderController】
 ***************************************
 * 功能        : 离散化后的三阶控制器
 * 作者        : 吴宏瑞
 *****************************************************************************************
 * 备注 *
 ********
 * 离散化方法详见：https://zhuanlan.zhihu.com/p/20927674
 *****************************************************************************************
*/
float thirdOrderController (float input)
{
	input_prev1 = input;
	input_prev2 = input_prev1;
	input_prev3 = input_prev2;
	
	output = coefficient_b1*input_prev1 + coefficient_b2*input_prev2 + coefficient_b3*input_prev3 - coefficient_a1*output_prev1 - coefficient_a2*output_prev2 - coefficient_a3*output_prev3;
	output = _constrain(output, -24, 24);
		
	output_prev1 = output;
	output_prev2 = output_prev1;
	output_prev3 = output_prev2;
	
	return output;
}
