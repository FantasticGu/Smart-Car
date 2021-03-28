#ifndef _VARIABLE_H_
#define _VARIABLE_H_

#include "include.h"

//////////////////////////main.c变量//////////////////
extern unsigned int row;	//摄像头行计数，最大240
extern uint8 video[H][V];	//存放数据数组
extern uint8 video_deal[H][V];	//存放数据数组
extern uint16 Bline_left[H];	//左边线存放数组
extern uint16 Bline_right[H];	//右边线存放数组
extern uint16 Pick_table[H];	//中心线存放数组
extern uint16 PrBline_left[H];  //原始的左边线存放数组
extern uint16 PrBline_right[H]; //原始的右边线存放数组
extern uint8  Pick_flag[H];//该行是否找到黑线标志数组
extern uint8  Deal_flag[H];//处理数据是否有效标志数组
extern uint16 lost_already;
extern uint8 Pick_line;
//uint8 PickCenter_flag=0;	//提取中线标志
extern uint8 Lost_Line_count;
extern uint8 Lost_left_count;
extern uint8 Lost_right_count;
extern uint8 Near_lost;//近处丢失标志
extern uint8 Shi_zi_line;
extern uint8 Shi_zi_num;
extern uint8 Shi_zi_flag;
extern uint16 last_position;
extern uint8 stop_num;
extern int Position_diff;
extern float biLi;

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
extern uint8 start_end_flag;	//起跑线标志
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
extern uint8 start_end_flag;
extern uint8 roadFlag;//赛道标志
extern uint8 last_roadFlag;
extern uint8 temp_shizi;//零时用
extern uint32 run_time;//程序运行时间（次数）
extern uint8 ls_flag;//小S标志
extern uint8 S_road_flag;
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
extern int D_slope_near;
extern int D_slope;		//舵机斜率控制值
extern int Curve;
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
///////////////////////main.c  End/////////////////////////////

#endif