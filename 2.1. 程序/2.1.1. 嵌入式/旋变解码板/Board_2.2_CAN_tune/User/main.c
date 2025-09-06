#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/AD2S1210/AD2S1210.h"
#include "./BSP/DMA/dma.h"
#include "./BSP/FDCAN/fdcan.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define PI 3.14159265358979f

char test[256]; 
float position = 0;
float velocity = 0;
float position1 = 0;
float velocity1 = 0;
float position2 = 0;
float velocity2 = 0;
unsigned char buf[4] = {0, 0, 0, 0};   //用于存储AD2S1210位置或速度寄存器值
unsigned short velocity_com = 0;       //关节转速补码。unsigned short
uint8_t buffer[8]; // 创建8字节的缓冲区

void CAN_SendTwoFloats(float value1, float value2) {

    // 将第一个浮点数的4字节二进制表示放入缓冲区
    memcpy(&buffer[0], &value1, sizeof(float));

    // 将第二个浮点数的4字节二进制表示放入缓冲区
    memcpy(&buffer[4], &value2, sizeof(float));
	
    fdcan1_send_msg(buffer, FDCAN_DLC_BYTES_8);                               /* 发送8个字节 */
}

int main(void)
{
	 /*** 初始化 ***/
    HAL_Init();                           /* 初始化HAL库 */
    sys_stm32_clock_init(85, 2, 2, 4, 8); /* 设置时钟,170Mhz */
    delay_init(170);                      /* 延时初始化 */
	
    usart_init(921600);                   /* 串口初始化波特率 */	
		      
    fdcan_init(17, 8, 2, 7, FDCAN_MODE_NORMAL);     /* FDCAN初始化*/
	
	 /*** AD2S1210 ***/
    AD2S1210GPIOInitiate();
    AD2S1210Initiate();                    //上电时序控制和复位

    CLR_SOE();
    SET_RD();                              //串口通信需将RD拉高

    AD2S1210SelectMode(CONFIG);            //进入配置模式，对寄存器进行编程，以设置AD2S1210的激励频率、分辨率和故障检测阈值

    CLR_RES0();
    SET_RES1();                            //控制寄存器中的分辨率位应与RES0, RES1输入引脚的电平保持一致，默认为12位
			
    while (1)
    {
				/*** AD2S1210 ***/
			  /* 位置读取模式 */ 
        AD2S1210SelectMode(POSITION);
        ReadFromAD2S1210(POSITION, POS_VEL, buf);      //读取数据寄存器
			
			  position = (((buf[2] << 8) | buf[1]) >> 4) * 360 / 4095;
			
			  /* 速度读取模式 */
        AD2S1210SelectMode(VELOCITY);             
        ReadFromAD2S1210(VELOCITY, POS_VEL, buf);  
        velocity_com = ((buf[2] << 8) | buf[1]) >> 4; // 关节转速12位补码
        if ((velocity_com & 0x800) >> 11){            // 当补码符号位大于0时（为1），补码表示非正数
            if (velocity_com == 0xFFF)                // 补码表示-0时
            {
							  velocity = 0;                         // 原码
            }
            else                                         //补码表示负数时
            {
                velocity = -1 * (4096 - velocity_com) * 1000 / 2048; //速度输出（转/秒）。对AD2S1210来说，当跟踪速率为12位（数据位11位）、时钟频率为8.192MHz时，可跟踪的速度最大值为1000（rps）

            }
				}
        else                                   //当补码符号位为0时，补码表示正数
        {
            velocity = velocity_com * 1000 / 2048; //速度输出（转/秒）。

        }
								
				position1 = 360 - position; //算法逆时针为正，旋变顺时针为正
		    velocity1 = -1 * velocity;  //算法逆时针为正，旋变顺时针为正
		
		    position2 = position1 * PI / 180;  //关节角度反馈（rad）
        velocity2 = velocity1 * 2 * PI; //关节转速反馈（rad/s）
								
				CAN_SendTwoFloats(position2, velocity2);
    }
}
