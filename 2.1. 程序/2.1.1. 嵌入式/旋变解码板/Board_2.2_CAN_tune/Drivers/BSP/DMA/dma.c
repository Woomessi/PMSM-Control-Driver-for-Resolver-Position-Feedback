/**
 ****************************************************************************************************
 * @file        dma.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief       DMA 驱动代码
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

#include "./BSP/DMA/dma.h"
#include "./SYSTEM/delay/delay.h"


DMA_HandleTypeDef  g_dma_handle;                  /* DMA句柄 */

extern UART_HandleTypeDef g_uart1_handle;         /* UART句柄 */

/**
 * @brief       串口TX DMA初始化函数
 * @note        这里的传输形式是固定的, 这点要根据不同的情况来修改
 *              从存储器 -> 外设模式/8位数据宽度/存储器增量模式
 *
 * @param       dma_channel_handle : DMA通道：DMA1_Channel1~DMA1_Channel8
 * @retval      无
 */
void dma_init(DMA_Channel_TypeDef *dma_channel_handle)
{ 
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Tx DMA配置 */
    g_dma_handle.Instance = dma_channel_handle;                     /* DMA通道选择 */
    g_dma_handle.Init.Request = DMA_REQUEST_USART1_TX;              /* DMA请求选择 */
    g_dma_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;             /* 存储器到外设 */
    g_dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;                 /* 外设非增量模式 */
    g_dma_handle.Init.MemInc = DMA_MINC_ENABLE;                     /* 存储器增量模式 */
    g_dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;    /* 外设数据长度:8位 */
    g_dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;       /* 存储器数据长度:8位 */
    g_dma_handle.Init.Mode = DMA_NORMAL;                            /* 外设流控模式 */
    g_dma_handle.Init.Priority = DMA_PRIORITY_VERY_HIGH;               /* 中等优先级 */
    HAL_DMA_DeInit(&g_dma_handle);   
    HAL_DMA_Init(&g_dma_handle);
	
	    __HAL_LINKDMA(&g_uart1_handle, hdmatx, g_dma_handle);           /* 将DMA与USART1联系起来(发送DMA) */
	
	  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);   // 设置优先级
	  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);           // 启用DMA通道中断


}























