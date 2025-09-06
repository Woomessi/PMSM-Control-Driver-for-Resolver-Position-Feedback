/**
 ****************************************************************************************************
 * @file        fdcan.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief       FDCAN 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台：正点原子 STM32G474开发板
 * 在线视频：www.yuanzige.com
 * 技术论坛：http://www.openedv.com/forum.php
 * 公司网址：www.alientek.com
 * 购买地址：zhengdianyuanzi.tmall.com
 *
 * 修改说明
 * V1.0 20230801
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "./BSP/FDCAN/fdcan.h"
#include "./BSP/LED/led.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"


FDCAN_HandleTypeDef   g_fdcanx_handler = {0};     /* FDCANx句柄 */
FDCAN_TxHeaderTypeDef g_fdcanx_txheader = {0};    /* 发送参数句柄 */
FDCAN_RxHeaderTypeDef g_fdcanx_rxheader = {0};    /* 接收参数句柄 */

/**
 * @brief       FDCAN初始化
 * @param       presc   : 分频值，取值范围1~512;
 * @param       ntsjw   : 重新同步跳跃时间单元.范围:1~128;
 * @param       ntsg1   : 时间段1的时间单元.取值范围2~256;
 * @param       ntsg2   : 时间段2的时间单元.取值范围2~128;
 * @param       mode    : FDCAN_MODE_NORMAL，普通模式; 
                          FDCAN_MODE_INTERNAL_LOOPBACK，内部回环模式;
                          FDCAN_MODE_EXTERNAL_LOOPBACK，外部回环模式;
                          FDCAN_MODE_RESTRICTED_OPERATION，限制操作模式
                          FDCAN_MODE_BUS_MONITORING，总线监控模式
 *   @note      以上5个参数, 除了模式选择其余的参数在函数内部会减1, 所以, 任何一个参数都不能等于0
 *              FDCAN其输入时钟频率为 Fpclk1 = PCLK1 = 170Mhz
 *              波特率 = Fpclk1 / ((ntsg1 + ntsg2 + 1) * presc);
 *              我们设置 can_init(17, 8, 11, 8, 1), 则CAN波特率为:
 *              170M / ((11 + 8 + 1) * 17) = 500Kbps
 * @retval      0,  初始化成功; 其他, 初始化失败;
 */
uint8_t fdcan_init(uint16_t presc, uint8_t ntsjw, uint16_t ntsg1, uint16_t ntsg2, uint32_t mode)
{
    FDCAN_FilterTypeDef canx_rxfilter = {0};
    HAL_FDCAN_DeInit(&g_fdcanx_handler);                                /* 先清除以前的设置 */
    
    g_fdcanx_handler.Instance = FDCAN1;
    g_fdcanx_handler.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    g_fdcanx_handler.Init.FrameFormat = FDCAN_FRAME_CLASSIC;            /* 传统模式 */
    g_fdcanx_handler.Init.Mode = mode;                                  /* 回环测试 */
    g_fdcanx_handler.Init.AutoRetransmission = DISABLE;                 /* 关闭自动重传！传统模式下一定要关闭！！！ */
    g_fdcanx_handler.Init.TransmitPause = DISABLE;                      /* 关闭传输暂停 */
    g_fdcanx_handler.Init.ProtocolException = DISABLE;                  /* 关闭协议异常处理 */
    /* FDCAN中仲裁段位速率最高1Mbit/s, 数据段位速率最高8Mbit/s */
    /* 数据段通信速率（仅FDCAN模式需配置） = 42.5M / (1 + dseg1 + dseg2) = 42.5M / (6 + 1 + 1) = 5.3 Mbit/s */
    g_fdcanx_handler.Init.DataPrescaler = 4;                            /* 数据段分频系数范围:1~32  */
    g_fdcanx_handler.Init.DataSyncJumpWidth = 16;                       /* 数据段重新同步跳跃宽度1~16 */
    g_fdcanx_handler.Init.DataTimeSeg1 = 3;                             /* 数据段dsg1范围:1~32  5 */
    g_fdcanx_handler.Init.DataTimeSeg2 = 1;                             /* 数据段dsg2范围:1~16  1 */
    
    /* 仲裁段通信速率（FDCAN与传统CAN均需配置） = 10M / (1 + ntsg1 + ntsg2) = 10M / (11 + 8 + 1) = 500Kbit/s */
    g_fdcanx_handler.Init.NominalPrescaler = presc;                     /* 分频系数 */
    g_fdcanx_handler.Init.NominalSyncJumpWidth = ntsjw;                 /* 重新同步跳跃宽度 */
    g_fdcanx_handler.Init.NominalTimeSeg1 = ntsg1;                      /* ntsg1范围:2~256 */
    g_fdcanx_handler.Init.NominalTimeSeg2 = ntsg2;                      /* ntsg2范围:2~128 */
    g_fdcanx_handler.Init.StdFiltersNbr = 28;                           /* 标准信息ID滤波器编号0 ~ 28  */
    g_fdcanx_handler.Init.ExtFiltersNbr = 8;                            /* 扩展信息ID滤波器编号0 ~ 8 */
    g_fdcanx_handler.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;    /* 发送FIFO序列模式 */
    
    if (HAL_FDCAN_Init(&g_fdcanx_handler) != HAL_OK)
    {
        return 1;
    }
    
    /* 配置CAN过滤器 */
    canx_rxfilter.IdType = FDCAN_STANDARD_ID;                           /* 标准ID */
    canx_rxfilter.FilterIndex = 0;                                      /* 滤波器索引 */                   
    canx_rxfilter.FilterType = FDCAN_FILTER_MASK;                       /* 滤波器类型 */
    canx_rxfilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;               /* 过滤器0关联到FIFO0 */  
    canx_rxfilter.FilterID1 = 0x0000;                                   /* 32位ID */
    canx_rxfilter.FilterID2 = 0x0000;                                   /* 如果FDCAN配置为传统模式的话，这里是32位掩码 */
    /* 过滤器配置 */
    if (HAL_FDCAN_ConfigFilter(&g_fdcanx_handler, &canx_rxfilter) != HAL_OK)
    {
        return 2;
    }
    /* 配置全局过滤器,拒收所有不匹配的标准帧或扩展帧 */
    if (HAL_FDCAN_ConfigGlobalFilter(&g_fdcanx_handler, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        return 3;
    }
    /* 启动CAN外围设备 */
    if (HAL_FDCAN_Start(&g_fdcanx_handler) != HAL_OK)
    {
        return 4;
    }
    HAL_FDCAN_ActivateNotification(&g_fdcanx_handler, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    return 0;
}

/**
 * @brief       FDCAN底层驱动，引脚配置，时钟配置，中断配置
                此函数会被HAL_FDCAN_Init()调用
 * @param       hcan:FDCAN句柄
 * @retval      无
 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    
    if (hfdcan->Instance == FDCAN1)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

        CAN_RX_GPIO_CLK_ENABLE();                           /* CAN_RX脚时钟使能 */
        CAN_TX_GPIO_CLK_ENABLE();                           /* CAN_TX脚时钟使能 */
        __HAL_RCC_FDCAN_CLK_ENABLE();                       /* 使能FDCAN时钟 */

        gpio_init_struct.Pin = CAN_TX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(CAN_TX_GPIO_PORT, &gpio_init_struct); /* CAN_TX脚 模式设置 */

        gpio_init_struct.Pin = CAN_RX_GPIO_PIN;
        HAL_GPIO_Init(CAN_RX_GPIO_PORT, &gpio_init_struct); /* CAN_RX脚 必须设置成输入模式 */

#if FDCAN1_RX0_INT_ENABLE     
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn,1,2);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
#endif
    }
}


/**
 * @brief       CAN 发送一组数据
 * @note        发送格式固定为: 标准ID, 数据帧
 * @param       len     :数据长度,取值范围：FDCAN_DLC_BYTES_0 ~ FDCAN_DLC_BYTES_64
 * @param       msg     :数据指针,最大为8个字节
 * @retval      发送状态 0, 成功; 1, 失败;
 */
uint8_t fdcan1_send_msg(uint8_t *msg, uint32_t len)
{
    g_fdcanx_txheader.Identifier = 0x12;                              /* 32位ID */
    g_fdcanx_txheader.IdType = FDCAN_STANDARD_ID;                     /* 标准ID */
    g_fdcanx_txheader.TxFrameType = FDCAN_DATA_FRAME;                 /* 数据帧 */
    g_fdcanx_txheader.DataLength = len;                               /* 数据长度 */
    g_fdcanx_txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;            
    g_fdcanx_txheader.BitRateSwitch = FDCAN_BRS_OFF;                  /* 关闭速率切换 */
    g_fdcanx_txheader.FDFormat = FDCAN_CLASSIC_CAN;                   /* 传统的CAN模式 */
    g_fdcanx_txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;        /* 无发送事件 */
    g_fdcanx_txheader.MessageMarker = 0;                           
     
    if(HAL_FDCAN_AddMessageToTxFifoQ(&g_fdcanx_handler, &g_fdcanx_txheader, msg) != HAL_OK) 
    {
        return 1;
    }
    return 0;
}

/**
 * @brief       CAN 接收数据查询
 * @note        接收数据格式固定为: 标准ID, 数据帧
 * @param       buf     : 数据缓存区
 * @retval      接收结果
 * @arg         0   , 无数据被接收到;
 * @arg         其他, 接收的数据长度
 */
uint8_t fdcan1_receive_msg(uint8_t *buf)
{
    if(HAL_FDCAN_GetRxMessage(&g_fdcanx_handler, FDCAN_RX_FIFO0, &g_fdcanx_rxheader, buf) != HAL_OK)    /* 读取数据 */
    {
        return 0;
    }
    return g_fdcanx_rxheader.DataLength >> 16;
}


#if FDCAN1_RX0_INT_ENABLE /* 使能RX0中断 */

/**
 * @brief       FDCAN中断服务函数
 * @param       无
 * @retval      无
 */
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&g_fdcanx_handler);
}

/**
 * @brief       FIFO0回调函数
 * @note        处理CAN FIFO0的接收中断
 * @param       无
 * @retval      无
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint8_t rxdata[8];
    uint8_t i = 0;
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)   /* FIFO1新数据中断 */
    {
        /* 提取FIFO0中接收到的数据 */
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &g_fdcanx_rxheader, rxdata);
        printf("id:%#x\r\n", g_fdcanx_rxheader.Identifier);
        printf("len:%d\r\n", g_fdcanx_rxheader.DataLength >> 16);
        for(i = 0; i < 8; i++)
        {
            printf("rxdata[%d]:%d\r\n", i, rxdata[i]);
        }
        
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}

#endif
