#ifndef AD2S1210_H
#define AD2S1210_H

// add the header file here

#define DIR_GPIO_PORT             GPIOC
#define DIR_GPIO_PIN              GPIO_PIN_13
#define DIR_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define _RESET_GPIO_PORT             GPIOE
#define _RESET_GPIO_PIN              GPIO_PIN_3
#define _RESET_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)

#define LOT_GPIO_PORT             GPIOE
#define LOT_GPIO_PIN              GPIO_PIN_1
#define LOT_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)

#define DOS_GPIO_PORT             GPIOE
#define DOS_GPIO_PIN              GPIO_PIN_0
#define DOS_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)

#define A1_GPIO_PORT             GPIOB
#define A1_GPIO_PIN              GPIO_PIN_5
#define A1_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define A0_GPIO_PORT             GPIOB
#define A0_GPIO_PIN              GPIO_PIN_4
#define A0_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define RES0_GPIO_PORT             GPIOA
#define RES0_GPIO_PIN              GPIO_PIN_1
#define RES0_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define RES1_GPIO_PORT             GPIOA
#define RES1_GPIO_PIN              GPIO_PIN_0
#define RES1_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define _RD_GPIO_PORT             GPIOC
#define _RD_GPIO_PIN              GPIO_PIN_3
#define _RD_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define _WR_GPIO_PORT             GPIOC
#define _WR_GPIO_PIN              GPIO_PIN_2
#define _WR_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define _SOE_GPIO_PORT             GPIOF
#define _SOE_GPIO_PIN              GPIO_PIN_10
#define _SOE_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

#define _SAMPLE_GPIO_PORT             GPIOF
#define _SAMPLE_GPIO_PIN              GPIO_PIN_9
#define _SAMPLE_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

#define _CS_GPIO_PORT             GPIOF
#define _CS_GPIO_PIN              GPIO_PIN_2
#define _CS_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)   

#define SCLK_GPIO_PORT             GPIOE
#define SCLK_GPIO_PIN              GPIO_PIN_2
#define SCLK_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   

#define SDI_GPIO_PORT             GPIOE
#define SDI_GPIO_PIN              GPIO_PIN_6
#define SDI_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)  

#define SDO_GPIO_PORT             GPIOE
#define SDO_GPIO_PIN              GPIO_PIN_5
#define SDO_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   

//Mode Select
#define POSITION	0
#define	VELOCITY	1
#define CONFIG		2  


//Register Map 
#define	POSITIONMSB		0x80
#define	POSITIONLSB		0x81
#define	VELOCITYMSB		0x82
#define	VELOCITYLSB		0x83
#define	LOSTHRES		0x88
#define	DOSORTHRES		0x89
#define	DOSMISTHRES		0x8A
#define	DOSRSTMXTHRES	0x8B
#define	DOSRSTMITHRES	0x8C
#define	LOTHITHRES		0x8D
#define	LOTLOTHRES		0x8E
#define	EXFREQUENCY		0x91
#define	CONTROL			0x92
#define	SOFTRESET		0xF0
#define	FAULT			0xFF
#define POS_VEL			0x00  //void register for normal read address

#define SET_RD() do{ HAL_GPIO_WritePin(_RD_GPIO_PORT, _RD_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_RD() do{ HAL_GPIO_WritePin(_RD_GPIO_PORT, _RD_GPIO_PIN, GPIO_PIN_RESET); }while(0)

#define SET_RESET() do{ HAL_GPIO_WritePin(_RESET_GPIO_PORT, _RESET_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_RESET() do{ HAL_GPIO_WritePin(_RESET_GPIO_PORT, _RESET_GPIO_PIN, GPIO_PIN_RESET); }while(0)

#define SET_SPL() do{ HAL_GPIO_WritePin(_SAMPLE_GPIO_PORT, _SAMPLE_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_SPL() do{ HAL_GPIO_WritePin(_SAMPLE_GPIO_PORT, _SAMPLE_GPIO_PIN, GPIO_PIN_RESET); }while(0)

#define SET_A0() do{ HAL_GPIO_WritePin(A0_GPIO_PORT, A0_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_A0() do{ HAL_GPIO_WritePin(A0_GPIO_PORT, A0_GPIO_PIN, GPIO_PIN_RESET); }while(0)

#define SET_A1() do{ HAL_GPIO_WritePin(A1_GPIO_PORT, A1_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_A1() do{ HAL_GPIO_WritePin(A1_GPIO_PORT, A1_GPIO_PIN, GPIO_PIN_RESET); }while(0)

#define SET_SDI() do{ HAL_GPIO_WritePin(SDI_GPIO_PORT, SDI_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_SDI() do{ HAL_GPIO_WritePin(SDI_GPIO_PORT, SDI_GPIO_PIN, GPIO_PIN_RESET); }while(0)

#define SET_SCLK() do{ HAL_GPIO_WritePin(SCLK_GPIO_PORT, SCLK_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_SCLK() do{ HAL_GPIO_WritePin(SCLK_GPIO_PORT, SCLK_GPIO_PIN, GPIO_PIN_RESET); }while(0)

#define SET_CS() do{ HAL_GPIO_WritePin(_CS_GPIO_PORT, _CS_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_CS() do{ HAL_GPIO_WritePin(_CS_GPIO_PORT, _CS_GPIO_PIN, GPIO_PIN_RESET); }while(0)

#define SET_WR() do{ HAL_GPIO_WritePin(_WR_GPIO_PORT, _WR_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_WR() do{ HAL_GPIO_WritePin(_WR_GPIO_PORT, _WR_GPIO_PIN, GPIO_PIN_RESET); }while(0)

#define SET_SOE() do{ HAL_GPIO_WritePin(_SOE_GPIO_PORT, _SOE_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_SOE() do{ HAL_GPIO_WritePin(_SOE_GPIO_PORT, _SOE_GPIO_PIN, GPIO_PIN_RESET); }while(0)

#define SET_RES0() do{ HAL_GPIO_WritePin(RES0_GPIO_PORT, RES0_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_RES0() do{ HAL_GPIO_WritePin(RES0_GPIO_PORT, RES0_GPIO_PIN, GPIO_PIN_RESET); }while(0)

#define SET_RES1() do{ HAL_GPIO_WritePin(RES1_GPIO_PORT, RES1_GPIO_PIN, GPIO_PIN_SET); }while(0)
#define CLR_RES1() do{ HAL_GPIO_WritePin(RES1_GPIO_PORT, RES1_GPIO_PIN, GPIO_PIN_RESET); }while(0)

void AD2S1210GPIOInitiate(void);
void AD2S1210Initiate(void);
void AD2S1210SelectMode(unsigned char mode);
void AD2S1210SoftReset(void);
void WriteToAD2S1210(unsigned char address, unsigned char data);
void ReadFromAD2S1210(unsigned char mode, unsigned char address, unsigned char * buf);
void SPIWrite(unsigned char count, unsigned char *buf);
void SPIRead(unsigned char count, unsigned char *buf);

#endif

