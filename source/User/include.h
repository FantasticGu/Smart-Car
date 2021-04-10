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
#define left_way 0  //���������
#define right_way 2 //���������
#define center_way 1 //���������

#define dj_center 885    //�������ֵ
#define dj_left_max 1020   //����������ֵ
#define dj_right_max 775  //����������ֵ

#define IMG_DIFF 12  //ͼ�����Ѱ���㷨��ֵ
#define BW_DIFF 15  //������ֵ���ɫ�Ҷ�ֵ��
#define JUDGE_DIFF 10	//�жϺ����Ƿ���Чб���жϲ�ֵ

#define NORMAL_SPEED 700
#define CIRCLE_SPEED 550
#define CROSS_SPEED 650

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

extern unsigned int row;	//����ͷ�м��������240
extern uint8 video[H][V];	//�����������
extern uint8 video_deal[H][V];	//�����������
extern uint16 Bline_left[H];	 //����ߴ������
extern uint16 Bline_right[H];	 //�ұ��ߴ������
extern uint16 Pick_table[H];	 //�����ߴ������

extern uint8 cuision_avg;

extern uint16 PrBline_left[H];  //ԭʼ������ߴ������
extern uint16 PrBline_right[H]; //ԭʼ���ұ��ߴ������
extern uint8 shizi_left;
extern uint8 shizi_right;
extern uint8  Pick_flag[H];//�����Ƿ��ҵ����߱�־����
extern uint8  Deal_flag[H];//���������Ƿ���Ч��־����
extern uint16 lost_already;
extern uint8 Pick_line;
//uint8 PickCenter_flag=0;	//��ȡ���߱�־
extern uint8 Lost_Line_count;
extern uint8 Lost_left_count;
extern uint8 mid_before;
extern uint8 Lost_right_count;
extern uint8 Near_lost;//������ʧ��־
extern uint8 Shi_zi_line;
extern uint8 Shi_zi_num;
extern uint8 Shi_zi_flag;
extern uint8 positive_num;
extern uint8 decrease_num;
extern uint16 last_position;
extern uint8 stop_num;
extern int Position_diff;
extern float biLi;
extern unsigned int const data_table[H];//��ɼ����ݵ���
extern uint8 Cmp;	//������ֵ
extern uint8 row_F[H];	//���вɼ���ɱ�־
extern char startline_F;	//������ʼ��
extern char endline_F;	//���ֽ����� 
extern unsigned int imagerow;	//�ɼ��м��������H
extern uint8 Out_flag;		//�����־����Чͼ���־
extern uint8 start_write;   //�����߰�ɫ��ֵ
extern uint8 liangdu;	//����ͷ����

extern uint8 last_vline;
extern uint8 valid_line;//�����Ч��
extern uint8 judge_vl;//�����жϵ���Ч��
extern uint8 last_lost;	//��һ����ʧ��
extern uint8 baoguang;	//�ع�ʱ��ֵ
extern uint8 Enter_shi_zi;//����ʮ�ֱ�־
extern uint16 Bline_diff;//�����߾���
extern uint16 maxBline_diff;
extern uint16 LR_diff;
extern uint32 whole_area;
extern uint32 tx_area;

extern int far_diff;	//Զ�����ƫ���� �������ֱ�������ж�
extern uint8 zhidao_speed;	//ֱ���ٶ�
extern uint8 CD_speed;	//ȫ���ٶ�
extern uint8 all_speed_flag;   //ȫ���ٶȱ�־

extern uint8 slow_down_num;//���ٱ�־
extern int judge_xielv[H-5];	//б���ж�����
extern uint8 buzzer_num;	//����������
extern uint8 buzzer_flag;	//��������Ӧ��־
extern uint8 zhidao_count_flag;	//ֱ���жϱ�־
extern uint8 last_zhidao_flag;
extern uint8 start_end_flag;	//�����߱�־
extern uint8 roadFlag;//������־
extern uint8 last_roadFlag;
extern uint8 temp_shizi;//��ʱ��
extern uint32 run_time;//��������ʱ�䣨������
extern uint8 ls_flag;//СS��־
extern uint8 S_road_flag;//S���־
extern uint8 pick_way;	//������� ��Ϊ0 ��Ϊ2 �м�Ϊ1
extern uint8 last_pick_way;//
extern uint8 lost_w_count;//��ɫ��ʧ�б���
extern uint8 lost_b_count;//��ɫ��ʧ�б���
extern int near_xielv_left;
extern int near_xielv_right;

extern int even_diff;	//������ƽ��ƫ��
extern int even_diff_near;
extern int even_diff1;   //
extern float dj_Kp;	//���Kpֵ
extern int D_slope_near;		//���б�ʿ���ֵ
extern int D_slope;		//���б�ʿ���ֵ
extern int Curve;		//Curve(���ߣ��ж�)ֵ
extern int Curve_near;		//Curve(���ߣ��ж�)ֵ
extern int Curve_middle;		//Curve(���ߣ��ж�)ֵ
extern int Curve_far;		//Curve(���ߣ��ж�)ֵ

extern int jiaodu_num;	//�Ƕ�ֵ
extern int last_turn;	//��һ��ת��ֵ
extern int dj_pid_num;	//����Ƕ�ֵ

extern uint8 set_flag;//���ñ�־λ
extern uint32 switch_key;	//���뿪�أ�IO�˿�ֵ
extern uint8 par_num;	//���ò�����ţ�4Ϊȫ���ٶ�
extern uint8 last_par_num;	//��һ�����ò�������������  
extern uint8 ctl_num;//���Ʊ�־ 0Ϊ�¿��ƣ�1Ϊ�Ͽ���
extern uint8 xianshi;//��ʾ����
extern uint8 init_7620_flag;	//OV7620���ñ�־λ
extern uint8 flash_flag;//Flash��־
extern uint8 DisImg_flag;//��ʾͼ���־
extern uint8 Far_find_flag;//
extern uint16 save_data_num;
extern uint8 save_file_num;
extern ParValue myPar_num;
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
    #include "duoji_ctl.h"

//�ⲿ�豸���Զ��幦��������ͷ�ļ�
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