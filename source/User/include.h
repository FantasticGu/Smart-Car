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
#define left_way 0  //舵机方向左
#define right_way 2 //舵机方向右
#define center_way 1 //舵机方向中

#define dj_center 885    //舵机中心值
#define dj_left_max 1020   //舵机左向最大值
#define dj_right_max 775  //舵机右向最大值

#define IMG_DIFF 12  //图像跟踪寻线算法差值
#define BW_DIFF 15  //黑线阈值与白色灰度值差
#define JUDGE_DIFF 10	//判断黑线是否有效斜率判断差值

#define NORMAL_SPEED 700
#define CIRCLE_SPEED 550
#define CROSS_SPEED 650

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

extern unsigned int row;	//摄像头行计数，最大240
extern uint8 video[H][V];	//存放数据数组
extern uint8 video_deal[H][V];	//存放数据数组
extern uint16 Bline_left[H];	 //左边线存放数组
extern uint16 Bline_right[H];	 //右边线存放数组
extern uint16 Pick_table[H];	 //中心线存放数组

extern uint8 cuision_avg;

extern uint16 PrBline_left[H];  //原始的左边线存放数组
extern uint16 PrBline_right[H]; //原始的右边线存放数组
extern uint8 shizi_left;
extern uint8 shizi_right;
extern uint8  Pick_flag[H];//该行是否找到黑线标志数组
extern uint8  Deal_flag[H];//处理数据是否有效标志数组
extern uint16 lost_already;
extern uint8 Pick_line;
//uint8 PickCenter_flag=0;	//提取中线标志
extern uint8 Lost_Line_count;
extern uint8 Lost_left_count;
extern uint8 mid_before;
extern uint8 Lost_right_count;
extern uint8 Near_lost;//近处丢失标志
extern uint8 Shi_zi_line;
extern uint8 Shi_zi_num;
extern uint8 Shi_zi_flag;
extern uint8 positive_num;
extern uint8 decrease_num;
extern uint16 last_position;
extern uint8 stop_num;
extern int Position_diff;
extern float biLi;
extern unsigned int const data_table[H];//需采集数据的行
extern uint8 Cmp;	//黑线阈值
extern uint8 row_F[H];	//该行采集完成标志
extern char startline_F;	//发现起始行
extern char endline_F;	//发现结束行 
extern unsigned int imagerow;	//采集行计数，最大H
extern uint8 Out_flag;		//出界标志，无效图像标志
extern uint8 start_write;   //起跑线白色阈值
extern uint8 liangdu;	//摄像头亮度

extern uint8 last_vline;
extern uint8 valid_line;//最大有效行
extern uint8 judge_vl;//用于判断的有效行
extern uint8 last_lost;	//上一场丢失行
extern uint8 baoguang;	//曝光时间值
extern uint8 Enter_shi_zi;//进入十字标志
extern uint16 Bline_diff;//两黑线距离
extern uint16 maxBline_diff;
extern uint16 LR_diff;
extern uint32 whole_area;
extern uint32 tx_area;

extern int far_diff;	//远端相对偏移量 作方向和直道长度判断
extern uint8 zhidao_speed;	//直道速度
extern uint8 CD_speed;	//全局速度
extern uint8 all_speed_flag;   //全局速度标志

extern uint8 slow_down_num;//减速标志
extern int judge_xielv[H-5];	//斜率判断数组
extern uint8 buzzer_num;	//蜂鸣器次数
extern uint8 buzzer_flag;	//蜂鸣器响应标志
extern uint8 zhidao_count_flag;	//直道判断标志
extern uint8 last_zhidao_flag;
extern uint8 start_end_flag;	//起跑线标志
extern uint8 roadFlag;//赛道标志
extern uint8 last_roadFlag;
extern uint8 temp_shizi;//零时用
extern uint32 run_time;//程序运行时间（次数）
extern uint8 ls_flag;//小S标志
extern uint8 S_road_flag;//S弯标志
extern uint8 pick_way;	//方向变量 左为0 右为2 中间为1
extern uint8 last_pick_way;//
extern uint8 lost_w_count;//白色丢失行变量
extern uint8 lost_b_count;//黑色丢失行变量
extern int near_xielv_left;
extern int near_xielv_right;

extern int even_diff;	//中心线平均偏差
extern int even_diff_near;
extern int even_diff1;   //
extern float dj_Kp;	//舵机Kp值
extern int D_slope_near;		//舵机斜率控制值
extern int D_slope;		//舵机斜率控制值
extern int Curve;		//Curve(曲线，判断)值
extern int Curve_near;		//Curve(曲线，判断)值
extern int Curve_middle;		//Curve(曲线，判断)值
extern int Curve_far;		//Curve(曲线，判断)值

extern int jiaodu_num;	//角度值
extern int last_turn;	//上一次转向值
extern int dj_pid_num;	//舵机角度值

extern uint8 set_flag;//设置标志位
extern uint32 switch_key;	//拨码开关，IO端口值
extern uint8 par_num;	//设置参数编号，4为全局速度
extern uint8 last_par_num;	//上一次设置参数，作清屏用  
extern uint8 ctl_num;//控制标志 0为新控制，1为老控制
extern uint8 xianshi;//显示参数
extern uint8 init_7620_flag;	//OV7620设置标志位
extern uint8 flash_flag;//Flash标志
extern uint8 DisImg_flag;//显示图像标志
extern uint8 Far_find_flag;//
extern uint16 save_data_num;
extern uint8 save_file_num;
extern ParValue myPar_num;
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
    #include "duoji_ctl.h"

//外部设备及自定义功能驱动的头文件
#include "func.h"
#include "oled.h"
#include "motor.h"
#include "steer.h"
#include "u_iic.h"
#include "pid.h"
#include "LQMPU6050.h"
#include "LQMT9V034.h"
#include "variable.h"
#endif