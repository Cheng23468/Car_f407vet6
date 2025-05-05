// 感为多路循迹模块，软件iic读取模拟量
#include "gw_gray_software_iic.h"


/**
 * @brief  初始化"感为循迹模块"的 GPIO 引脚。
 * 
 * 此函数将 GPIO 引脚 PC0 和 PC1 配置为开漏输出，带上拉电阻，
 * 并设置速度为 100 MHz。同时，它还启用了 GPIOC 外设的时钟。
 * 
 * @note  此初始化特定于 感为循迹模块 模块，并假设目标 STM32 微控制器
 *        上可用 GPIOC 外设。
 */
void gw_gray_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}


/* 基本时序操作 */
static void IIC_Delay(void)
{
    delay_us(15);  // 根据实际I2C速度调整延时
}
/* 软件I2C基础函数 */
void IIC_Start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    IIC_Delay();
    SDA_LOW();
    IIC_Delay();
    SCL_LOW();
}

void IIC_Stop(void)
{
    SDA_LOW();
    IIC_Delay();
    SCL_HIGH();
    IIC_Delay();
    SDA_HIGH();
    IIC_Delay();
}

unsigned char IIC_WaitAck(void)
{
    unsigned char ack;
    SDA_HIGH();
    SCL_HIGH();
    IIC_Delay();
    ack = GW_READ_SDA();
    SCL_LOW();
    IIC_Delay();
    return ack;
}

void IIC_SendAck(void)
{
    SDA_LOW();
    SCL_HIGH();
    IIC_Delay();
    SCL_LOW();
    SDA_HIGH();
}

void IIC_SendNAck(void)
{
    SDA_HIGH();
    SCL_HIGH();
    IIC_Delay();
    SCL_LOW();
}

unsigned char IIC_SendByte(unsigned char dat)
{
    for(unsigned char i = 0; i < 8; i++) {
        (dat & 0x80) ? SDA_HIGH() : SDA_LOW();
        dat <<= 1;
        SCL_HIGH();
        IIC_Delay();
        SCL_LOW();
        IIC_Delay();
    }
    return IIC_WaitAck();
}

unsigned char IIC_RecvByte(void)
{
    unsigned char dat = 0;
    SDA_HIGH();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.GPIO_Pin = SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;        // 关键修改！
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;            // 保持上拉
    GPIO_Init(SDA_PORT, &GPIO_InitStruct);
    for(unsigned char i = 0; i < 8; i++) {
        dat <<= 1;
        SCL_HIGH();
        IIC_Delay();
        if(GW_READ_SDA()) dat |= 0x01;
        SCL_LOW();
        IIC_Delay();
    }
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(SDA_PORT, &GPIO_InitStruct);
    return dat;
}

/* 应用层函数改写 */
unsigned char IIC_ReadByte(unsigned char Salve_Address)
{
    unsigned char dat;
    
    IIC_Start();
    IIC_SendByte(Salve_Address | 0x01);  // 读模式
    dat = IIC_RecvByte();
    IIC_SendNAck();
    IIC_Stop();
    
    return dat;
}

unsigned char IIC_ReadBytes(unsigned char Salve_Address, unsigned char Reg_Address, 
                          unsigned char *Result, unsigned char len)
{
    IIC_Start();
    if(IIC_SendByte(Salve_Address & 0xFE)) {  // 写模式
        IIC_Stop();
        return 0;
    }
    if(IIC_SendByte(Reg_Address)) {
        IIC_Stop();
        return 0;
    }
    IIC_Start();
    if(IIC_SendByte(Salve_Address | 0x01)) {  // 读模式
        IIC_Stop();
        return 0;
    }
    
    for(unsigned char i = 0; i < len; i++) {
        Result[i] = IIC_RecvByte();
        (i == len-1) ? IIC_SendNAck() : IIC_SendAck();
    }
    IIC_Stop();
    return 1;
}

unsigned char IIC_WriteByte(unsigned char Salve_Address, unsigned char Reg_Address, 
                          unsigned char data)
{
    IIC_Start();
    if(IIC_SendByte(Salve_Address & 0xFE)) {  // 写模式
        IIC_Stop();
        return 0;
    }
    if(IIC_SendByte(Reg_Address)) {
        IIC_Stop();
        return 0;
    }
    if(IIC_SendByte(data)) {
        IIC_Stop();
        return 0;
    }
    IIC_Stop();
    return 1;
}

unsigned char IIC_WriteBytes(unsigned char Salve_Address, unsigned char Reg_Address,
                           unsigned char *data, unsigned char len)
{
    IIC_Start();
    if(IIC_SendByte(Salve_Address & 0xFE)) {
        IIC_Stop();
        return 0;
    }
    if(IIC_SendByte(Reg_Address)) {
        IIC_Stop();
        return 0;
    }
    
    for(unsigned char i = 0; i < len; i++) {
        if(IIC_SendByte(data[i])) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 1;
}
unsigned char Ping(void)
{
	unsigned char dat;
	IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,GW_GRAY_PING,&dat,1);
	if(dat==GW_GRAY_PING_OK)
	{
			return 0;
	}	
	else return 1;
}
unsigned char IIC_Get_Digtal(void)
{
	unsigned char dat;
	IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,GW_GRAY_DIGITAL_MODE,&dat,1);
	return dat;
}
unsigned char IIC_Get_Anolog(unsigned char * Result,unsigned char len)
{
	if(IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,GW_GRAY_ANALOG_BASE_,Result,len))return 1;
	else return 0;
}
unsigned char IIC_Get_Single_Anolog(unsigned char Channel)
{
	unsigned char dat;
	IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,GW_GRAY_ANALOG(Channel),&dat,1);
	return dat;
}
unsigned char IIC_Get_Normalize(unsigned char * Result,unsigned char len)
{
	if(IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,GW_GRAY_ANALOG_NORMALIZE,Result,len))return 1;
	else return 0;
}
unsigned short IIC_Get_Offset(void )
{
	unsigned char dat;
	IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,Offset,&dat,1);
	return dat;
}

uint8_t IIC_Turn_On_Normalize()
{
    if(IIC_WriteByte(GW_GRAY_ADDR_DEF<<1, GW_GRAY_ANALOG_NORMALIZE, 0xFF))
        return 1;
    else
        return 0;
}

uint8_t IIC_Turn_Off_Normalize()
{
    if(IIC_WriteByte(GW_GRAY_ADDR_DEF<<1, GW_GRAY_ANALOG_NORMALIZE, 0x00))
        return 1;
    else
        return 0;
}
