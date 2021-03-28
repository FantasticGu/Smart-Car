#ifndef __FUN_H__
#define __FUN_H__

#include "include.h"
//#include "core_cm4.h"

//#define TEST_KEY(x,y) do{x=(GPIO_PDIR_REG(PORTB)&0x0000FFFF);y=(uint16)x;}while(0)

#define byte uint8
#define word uint16

//IO初始化函数
void IO_Init();

void put_image_center(uint16 *center,uint8 put_line);//提取中心线发送图像功能函数
void put_image_data();//发送图像功能函数
void put_image_1();//关中断发送一幅完整图像
void put_image_2(uint8 put_line);//关中断发送指定行
void put_center_hex(uint16 *center,uint8 put_line);//发送中心线，每5行发送1行，以十六进制发送
void put_center_char(uint16 *center,uint8 put_line);//发送中心线，每5行发送1行，以字符发送
void put_get_hex(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line);
void put_get_hex_whole(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line);
void put_get_char_whole(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line);
uint8 find_coordinate();//找到定位参考线
float get_curvature(uint16 x1,uint16 x2,uint16 x3,uint16 y1,uint16 y2,uint16 y3);//A(x1,y1),B(x2,y2),C(x3,y3)
float get_area_dsp(uint16 x1,uint16 x2,uint16 x3,uint16 y1,uint16 y2,uint16 y3);//A(x1,y1),B(x2,y2),C(x3,y3)
//获得面积 正则左方向，负则右方向（但得注意左右边线下标）
float get_area(uint32 x1,uint32 x2,uint32 x3,uint32 y1,uint32 y2,uint32 y3);//A(x1,y1),B(x2,y2),C(x3,y3)
/*
									*C		
							*
					*
				*B
			*
		*
	 *
  *
  *A
*/
//获得面积 正则左方向，负则右方向（但得注意左右边线下标）
void dis_img_irq();//关闭采集图像相关中断
int regression(uint16 Pick_table[],int startline,int endline);//最小二乘法计算斜率函数
void center_led();//显示中心线
void duoji_control(uint16 jiaodu,uint8 way);//舵机控制函数，注意舵机数组修改后对直道影响
void ti_jiaozheng();//矫正梯形失真
void ti_jiaozheng_new();//矫正梯形失真新方法
void Binaryzation();//二值化数据
void Binary_line(uint8 line);//二值化数据
void send_data_1(short int data);//以字符形式发送数据
void PickCenter_up(uint16 line);
int32 PickCenter_m(int32 start_pixel,uint16 pick_line);
void PickCenter_diff(uint16 line);
int32 PickCenter_near();//寻找近处中心线
void PickCenter_new();
void find_edge();//find_edge
void put_image_3(void);
void char_change(uint8 num,uint8 *table);//将显示字符转换为12864显示字符
void char_change_1(uint8 num,uint8 *table);
void char_change_2(float f_num,uint8 *table);
void buzzer_on();//蜂鸣器触发函数
void buzzer_ctl();//蜂鸣器控制函数
int32 find_shizi(uint8 line);//寻找十字道
uint32 abs_sub(uint32 diff1,uint32 diff2);//两个数相减的绝对值
float mult_f(float mult1,float mult2);//浮点数相乘，结果为浮点数
void send_some();
void show_miss(uint8 state);
void lvbo(uint8 );//滤波函数
void lvbo_cu(uint8 num);//粗滤波函数
void bu_xian();//补线函数
void set_ov7620_ld();
void set_ov7620_bg();
void lcd_int(uint8 position,uint8 *string,int num);//显示整型数
void Init_7620();
void test_center();//找到中心线,矫正摄像头用
void center_buxian();//中心线补线
uint32 getWholeArea();//获得有效行内的整体面积
uint32 getTxArea();//获得梯形的整体面积

void choice_xs();//选择显示参数
void choice_kz();//选择控制参数
void mode();//拨码开关功能函数选择
uint8 Menu_active(char **menuText, uint8 numItems);
uint8 scan_key();
void Testled();
int32 checkShizi(uint8 line);
void setCDSpeed();
void setZDSpeed();//设置直道速度
void BiLi();
void displaySet();
void displayPar();
void displayBinary();
void displayBlineSimple();//显示黑线简版
void Reset();
void set_pid_D();
void set_dj_Kp();
void set_pid_P();
void setThreshold();
void djSet();
void sendFlashData();
void readSDData();

void setArgFloat(char *str,float * Arg,const float step);//设置浮点型参数
void setArgUchar(char *str,unsigned char * Arg,const unsigned char step);//设置无符号char型参数
void setArgInt(char *str,int * Arg,const int step);//设置整型参数
void test_threshold();//测试灰度值函数
void choice_par();//选择设置函数
void set_par();//设置参数函数
void displayBline();//显示黑线
void LCD_P6x8Int(byte x,byte y,byte ch[],int num);//显示整型数

void delay(uint32 );
void delay2();
void display1();
void test_lcd();
void delay1(uint8 t);


void findleftpoint();
void findrightpoint();
void findleftwhite();
void findrightwhite();
int CrossRecognition();//判断是否有十字以及车体面对十字的状态，
void CrossConduct(int flag);//十字处理函数


void specialroad();

void CircleRecognition();

void stopcar();

void stable_control_fix(int valid_line);

void findnewroad();


extern uint16 bigbendflag;

extern uint8 readyleftflag;
extern uint8 readyrightflag;

extern uint8 readyleftnum[10];
extern uint8 readyrightnum[10];

extern uint8 leftcircleflag;
extern uint8 rightcircleflag;

extern uint8 leftflagnum[10];
extern uint8 rightflagnum[10];

extern uint8 intoleftcircleflag;
extern uint8 intorightcircleflag;

extern uint8 outleftcircleflag;
extern uint8 outrightcircleflag;

extern uint8 gostraightflag;


extern uint8 leftcrossflag;
extern uint8 rightcrossflag;
extern uint8 middlecrossflag;

extern uint8 stopcarflag;
extern uint8 stopcarflagnum[10];

extern uint8 whiteblackflag;

extern uint8 camera_flag;


#endif 