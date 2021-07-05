//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4
/*!                    �����ʵ��ѧ K66 ѧϰ����
 *  �ļ����ƣ�       include.h
 *      ���ߣ�       maddevil
 *      ˵����       �����ڲ�ѧϰʹ�ã������⴫
 *  �ο����ϣ�       ����ѧ�����롢ɽ��K60�⡢����K66ģ�塢����KEAģ��
 *    �汾�ţ�       V1.0.0
 *  �����£�       2018-12-21 13:41
 */
//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4
#ifndef INCLUDE_H_
#define INCLUDE_H_

//ͨ��ͷ�ļ�
    #include    <stdio.h>                       //printf
    #include    <string.h>                      //memcpy
    #include    <stdlib.h>                      //malloc

//Cortex-M�ں�MCU�Ĵ���ͷ�ļ�
    #include "MK66F18.h"   //�Ĵ���ӳ��ͷ�ļ�
    #include "arm_math.h "
    #include "Systick.h"

//����ͷ����
struct Bline
{
  uint8 line;//��
  int coord;  //������
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
#define endlineROW 240     //OV7620��ÿ��240��
#define change_start 15
#define change_set 30
#define PIC_H 60 //��ȡ������
#define  H 120   //�ɼ�����
#define  V 188  //ÿ�вɼ�����
//#define Cmp 160//����ͷ��ֵ����ֵ

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

#define FAR_FIND_MIDDLE 1//ȫ������־
#define FAR_FIND_DIFF   0//����������־
#define SD_MODE 1

#define ROAD_SHIZI 3		//������־ ʮ�ֵ�
////////////////////////////��ȡ�ñ�־///////////////////////////////
#define LEFT_LOST_W 0x01u		//00000001
#define LEFT_LOST_B 0x02u		//00000010
#define RIGHT_LOST_W 0x04u		//00000100
#define RIGHT_LOST_B 0x08u		//00001000
#define ALL_LOST_W 0x10u		//
#define ALL_LOST_B 0x20u		//
#define ALL_LOST 0x40u			//
///////////////////////////�ж��ñ�־//////////////////////////////////
#define INVALID_LINE 0x01u			//
#define NONE_LOST 0 				//
#define DEAL_LEFT_LOST 0x2u		    //��ʼʱ����߶�ʧ��־
#define DEAL_RIGHT_LOST 0x4u		//��ʼʱ�ұ��߶�ʧ��־

//////////////////////����궨��///////////////////////////
#define SAVE_FLASH_DATA 0	//	����flash�����궨��
#define CHANG_TEST 0		//  ���ж���LED���궨��

#define FLASH_ADDR 200
#define SECTOR_SIZE 2048

#define TEST_FLASH_ADDR 202
#define TEST_SECTOR_SIZE 2048
#define TEST_SD_SECTOR_SIZE 512

#define LCD_HEIGHT 64

//����ͷ����
///////////////////////////////////////////////����ͷ///////////////////////////////////////////////////////////
////////////////////////////////ȫ�ֱ�������////////////////////////////////////

extern uint16 Bline_left[H];	 //����ߴ������
extern uint16 Bline_right[H];	 //�ұ��ߴ������
extern uint16 Pick_table[H];	 //�����ߴ������
extern uint8 Cmp;	//������ֵ

extern int speedflag; 
extern int signal_number;


extern uint8 Image_Data[H][V];      //ͼ��ԭʼ���ݴ��
///////////////////////////////temp_variable////////////////////////////////
extern float area;
extern float last_area;
///////////////////////////////////////////////����ͷ///////////////////////////////////////////////////////////


////////////ѭ�����нṹ////////////
typedef int Status;
typedef int QElemType;
typedef struct{
  QElemType *base;
  int front;
  int rear;
}SqQueue;



//MCU�ڲ�ģ��������ͷ�ļ�
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

//�ж��������жϺ�������
    #include "vectors.h"
    #include "isr.h"

//�������ͼ��˿������ض���
    #include "common.h"

//����ͷͷ�ļ�
    #include "fun.h"

//�ⲿ�豸���Զ��幦��������ͷ�ļ�
#include "func.h"
#include "oled.h"
#include "motor.h"
#include "steer.h"
#include "u_iic.h"
#include "pid.h"
#include "LQMT9V034.h"
#include "variable.h"
#endif