//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4
/*!                    北京邮电大学 K66 学习例程
 *  文件名称：       include.h
 *      作者：       maddevil
 *      说明：       仅做内部学习使用，请勿外传
 *  参考资料：       历届学长代码、山外K60库、龙邱K66模板、北邮KEA模板
 *    版本号：       V1.0.0
 *  最后更新：       2018-12-21 13:41
 */
//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4
#ifndef INCLUDE_H_
#define INCLUDE_H_

//通用头文件
    #include    <stdio.h>                       //printf
    #include    <string.h>                      //memcpy
    #include    <stdlib.h>                      //malloc

//Cortex-M内核MCU寄存器头文件
    #include "MK66F18.h"   //寄存器映像头文件
    #include "arm_math.h "
    #include "Systick.h"

//摄像头参数
struct Bline
{
  uint8 line;//行
  int coord;  //列坐标
};

typedef struct
{
  int    SET_FLAG;
  uint8 ZHIDAO_SPEED;
  uint8 CD_SPEED;
  uint8 DIS_IMG_FLAG;
  float BILI;
  float DJ_KP;
  int KP;
  int KD;
  uint8 CMP;
}ParValue;
#define endlineROW 240     //OV7620，每场240行
#define change_start 15
#define change_set 30
#define PIC_H 60 //提取行行数
#define  H 120   //采集行数
#define  V 188  //每行采集点数
//#define Cmp 160//摄像头二值化阈值

#define NORMAL_SPEED 90//110 //90
#define CIRCLE_SPEED 85//105 //85
#define CROSS_SPEED 88//108  //88
#define START_SPEED 70 //85
#define STOP_SPEED 70

#define KEY1 0x01
#define KEY2 0x02
#define KEY3 0x03
#define KEY4 0x04
#define KEY5 0x05

#define FAR_FIND_MIDDLE 1//全搜索标志
#define FAR_FIND_DIFF   0//跟踪搜索标志
#define SD_MODE 1

#define ROAD_SHIZI 3		//赛道标志 十字道
////////////////////////////提取用标志///////////////////////////////
#define LEFT_LOST_W 0x01u		//00000001
#define LEFT_LOST_B 0x02u		//00000010
#define RIGHT_LOST_W 0x04u		//00000100
#define RIGHT_LOST_B 0x08u		//00001000
#define ALL_LOST_W 0x10u		//
#define ALL_LOST_B 0x20u		//
#define ALL_LOST 0x40u			//
///////////////////////////判断用标志//////////////////////////////////
#define INVALID_LINE 0x01u			//
#define NONE_LOST 0 				//
#define DEAL_LEFT_LOST 0x2u		    //初始时左边线丢失标志
#define DEAL_RIGHT_LOST 0x4u		//初始时右边线丢失标志

//////////////////////编译宏定义///////////////////////////
#define SAVE_FLASH_DATA 0	//	保存flash参数宏定义
#define CHANG_TEST 0		//  场中断中LED亮宏定义

#define FLASH_ADDR 200
#define SECTOR_SIZE 2048

#define TEST_FLASH_ADDR 202
#define TEST_SECTOR_SIZE 2048
#define TEST_SD_SECTOR_SIZE 512

#define LCD_HEIGHT 64

//摄像头参数
///////////////////////////////////////////////摄像头///////////////////////////////////////////////////////////
////////////////////////////////全局变量定义////////////////////////////////////

extern uint16 Bline_left[H];	 //左边线存放数组
extern uint16 Bline_right[H];	 //右边线存放数组
extern uint16 Pick_table[H];	 //中心线存放数组
extern uint8 Cmp;	//黑线阈值

extern int speedflag; 
extern int signal_number;


extern uint8 Image_Data[H][V];      //图像原始数据存放
///////////////////////////////temp_variable////////////////////////////////
extern float area;
extern float last_area;
///////////////////////////////////////////////摄像头///////////////////////////////////////////////////////////


////////////循环队列结构////////////
typedef int Status;
typedef int QElemType;
typedef struct{
  QElemType *base;
  int front;
  int rear;
}SqQueue;



//MCU内部模块驱动的头文件
    #include "GPIO.h"
    #include "GPIO_Cfg.h"
    #include "PLL.h"
    #include "FTM.h"    
    #include "UART.h"
    #include "ADC.h"
    #include "PLL.h"    
    #include "PIT.h"
    #include "I2C.h"
    #include "motor.h"
    //#include "SPI.h"
    #include "DMA.h"
    #include "lptmr.h"    
    #include "RTC.h"
    #include "exti.h"

//中断向量表及中断函数声明
    #include "vectors.h"
    #include "isr.h"

//数据类型及端口名称重定义
    #include "common.h"

//摄像头头文件
    #include "fun.h"

//外部设备及自定义功能驱动的头文件
#include "func.h"
#include "oled.h"
#include "motor.h"
#include "steer.h"
#include "u_iic.h"
#include "pid.h"
#include "LQMT9V034.h"
#include "variable.h"
#endif