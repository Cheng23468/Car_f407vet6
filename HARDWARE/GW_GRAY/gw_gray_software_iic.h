#ifndef __GW_GRAY_SOFTWARE_IIC_H
#define __GW_GRAY_SOFTWARE_IIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "gw_grayscale_sensor.h"
#include "delay.h"

#define SDA_PIN     GPIO_Pin_1  // PC1 -> SDA
#define SDA_PORT    GPIOC
#define SCL_PIN     GPIO_Pin_0  // PC0 -> SCL
#define SCL_PORT    GPIOC

/* 基本I2C操作宏 */
#define SDA_HIGH() GPIO_SetBits(SDA_PORT, SDA_PIN)
#define SDA_LOW()  GPIO_ResetBits(SDA_PORT, SDA_PIN)
#define SCL_HIGH() GPIO_SetBits(SCL_PORT, SCL_PIN)
#define SCL_LOW()  GPIO_ResetBits(SCL_PORT, SCL_PIN)
#define GW_READ_SDA() GPIO_ReadInputDataBit(SDA_PORT, SDA_PIN)

void gw_gray_init(void);
unsigned char Ping(void);
unsigned char IIC_Get_Digtal(void);
unsigned char IIC_Get_Anolog(unsigned char * Result,unsigned char len);
unsigned char IIC_Get_Single_Anolog(unsigned char Channel);
unsigned char IIC_Get_Normalize(unsigned char * Result,unsigned char len);
unsigned short IIC_Get_Offset(void );
uint8_t IIC_Turn_On_Normalize();
uint8_t IIC_Turn_Off_Normalize();

#ifdef __cplusplus
}
#endif

#endif /* __GW_GRAY_SOFTWARE_IIC_H */