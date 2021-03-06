//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4
/*!                    北京邮电大学 K66 学习例程
 *  文件名称：       I2C.h
 *      作者：       maddevil
 *      说明：       仅做内部学习使用，请勿外传
 *  参考资料：       历届学长代码、山外K60库、龙邱K66模板、北邮KEA模板
 *    版本号：       V1.0.0
 *  最后更新：       2018-12-21 13:41
 */
//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4

#ifndef _I2C_H_
#define _I2C_H_

/**********************************  IIC(引脚复用) ***************************************/

#define I2C0_SCL    PTB2        // PTB0、PTB2、PTD8
#define I2C0_SDA    PTB3        // PTB1、PTB3、PTD9

#define I2C1_SCL    PTE1       // PTE1、PTC10
#define I2C1_SDA    PTE0       // PTE0、PTC11

/**********************************  IIC(引脚复用) ***************************************/

//定义模块号
typedef enum I2Cn
{
    I2C_0  = 0,
    I2C_1  = 1
} I2Cn;

//定义读写选项
typedef enum MSmode
{
    write =   0x00,  /* Master write  */
    read =   0x01   /* Master read */
} MSmode;


#define I2C_DisableAck(I2Cn)        I2C_C1_REG(I2Cx[I2Cn]) |= I2C_C1_TXAK_MASK

//
#define I2C_RepeatedStart(I2Cn)     I2C_C1_REG(I2Cx[I2Cn]) |= I2C_C1_RSTA_MASK

//启动信号
#define I2C_Start(I2Cn)             I2C_C1_REG(I2Cx[I2Cn]) |= I2C_C1_TX_MASK+I2C_C1_MST_MASK;\
                                    //I2C_C1_REG(I2Cx[I2Cn]) |= I2C_C1_MST_MASK

//暂停信号
#define I2C_Stop(I2Cn)              I2C_C1_REG(I2Cx[I2Cn]) &= ~(I2C_C1_MST_MASK+I2C_C1_TX_MASK);\
                                    //I2C_C1_REG(I2Cx[I2Cn]) &= ~I2C_C1_TX_MASK

//进入接收模式(应答)
#define I2C_EnterRxMode(I2Cn)       I2C_C1_REG(I2Cx[I2Cn]) &= ~I2C_C1_TX_MASK;\
                                    I2C_C1_REG(I2Cx[I2Cn]) &= ~I2C_C1_TXAK_MASK
//进入接收模式(不应答)
#define I2C_PutinRxMode(I2Cn)       I2C_C1_REG(I2Cx[I2Cn]) &= ~I2C_C1_TX_MASK

//等待 I2C0_S
#define I2C_Wait(I2Cn)              while(( I2C_S_REG(I2Cx[I2Cn]) & I2C_S_IICIF_MASK)==0) {} \
                                    I2C_S_REG(I2Cx[I2Cn]) |= I2C_S_IICIF_MASK;

//写一个字节
#define I2C_write_byte(I2Cn,data)   I2C_D_REG(I2Cx[I2Cn]) = data


void  I2C_Init(I2Cn A);                                         //初始化I2C
void  I2C_WriteAddr(I2Cn, u8 SlaveID, u8 Addr, u8 Data);      //读取地址里的内容
void  I2C_StartTransmission (I2Cn, u8 SlaveID, MSmode);       //启动传输
u8    I2C_ReadAddr(I2Cn, u8 SlaveID, u8 Addr);                //往地址里写入内容



#endif