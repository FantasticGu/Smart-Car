#ifndef __FUN_H__
#define __FUN_H__

#include "include.h"
//#include "core_cm4.h"

//#define TEST_KEY(x,y) do{x=(GPIO_PDIR_REG(PORTB)&0x0000FFFF);y=(uint16)x;}while(0)

#define byte uint8
#define word uint16

//IO��ʼ������
void IO_Init();

void put_image_center(uint16 *center,uint8 put_line);//��ȡ�����߷���ͼ���ܺ���
void put_image_data();//����ͼ���ܺ���
void put_image_1();//���жϷ���һ������ͼ��
void put_image_2(uint8 put_line);//���жϷ���ָ����
void put_center_hex(uint16 *center,uint8 put_line);//���������ߣ�ÿ5�з���1�У���ʮ�����Ʒ���
void put_center_char(uint16 *center,uint8 put_line);//���������ߣ�ÿ5�з���1�У����ַ�����
void put_get_hex(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line);
void put_get_hex_whole(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line);
void put_get_char_whole(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line);
uint8 find_coordinate();//�ҵ���λ�ο���
float get_curvature(uint16 x1,uint16 x2,uint16 x3,uint16 y1,uint16 y2,uint16 y3);//A(x1,y1),B(x2,y2),C(x3,y3)
float get_area_dsp(uint16 x1,uint16 x2,uint16 x3,uint16 y1,uint16 y2,uint16 y3);//A(x1,y1),B(x2,y2),C(x3,y3)
//������ �������򣬸����ҷ��򣨵���ע�����ұ����±꣩
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
//������ �������򣬸����ҷ��򣨵���ע�����ұ����±꣩
void dis_img_irq();//�رղɼ�ͼ������ж�
int regression(uint16 Pick_table[],int startline,int endline);//��С���˷�����б�ʺ���
void center_led();//��ʾ������
void duoji_control(uint16 jiaodu,uint8 way);//������ƺ�����ע���������޸ĺ��ֱ��Ӱ��
void ti_jiaozheng();//��������ʧ��
void ti_jiaozheng_new();//��������ʧ���·���
void Binaryzation();//��ֵ������
void Binary_line(uint8 line);//��ֵ������
void send_data_1(short int data);//���ַ���ʽ��������
void PickCenter_up(uint16 line);
int32 PickCenter_m(int32 start_pixel,uint16 pick_line);
void PickCenter_diff(uint16 line);
int32 PickCenter_near();//Ѱ�ҽ���������
void PickCenter_new();
void find_edge();//find_edge
void put_image_3(void);
void char_change(uint8 num,uint8 *table);//����ʾ�ַ�ת��Ϊ12864��ʾ�ַ�
void char_change_1(uint8 num,uint8 *table);
void char_change_2(float f_num,uint8 *table);
void buzzer_on();//��������������
void buzzer_ctl();//���������ƺ���
int32 find_shizi(uint8 line);//Ѱ��ʮ�ֵ�
uint32 abs_sub(uint32 diff1,uint32 diff2);//����������ľ���ֵ
float mult_f(float mult1,float mult2);//��������ˣ����Ϊ������
void send_some();
void show_miss(uint8 state);
void lvbo(uint8 );//�˲�����
void lvbo_cu(uint8 num);//���˲�����
void bu_xian();//���ߺ���
void set_ov7620_ld();
void set_ov7620_bg();
void lcd_int(uint8 position,uint8 *string,int num);//��ʾ������
void Init_7620();
void test_center();//�ҵ�������,��������ͷ��
void center_buxian();//�����߲���
uint32 getWholeArea();//�����Ч���ڵ��������
uint32 getTxArea();//������ε��������

void choice_xs();//ѡ����ʾ����
void choice_kz();//ѡ����Ʋ���
void mode();//���뿪�ع��ܺ���ѡ��
uint8 Menu_active(char **menuText, uint8 numItems);
uint8 scan_key();
void Testled();
int32 checkShizi(uint8 line);
void setCDSpeed();
void setZDSpeed();//����ֱ���ٶ�
void BiLi();
void displaySet();
void displayPar();
void displayBinary();
void displayBlineSimple();//��ʾ���߼��
void Reset();
void set_pid_D();
void set_dj_Kp();
void set_pid_P();
void setThreshold();
void djSet();
void sendFlashData();
void readSDData();

void setArgFloat(char *str,float * Arg,const float step);//���ø����Ͳ���
void setArgUchar(char *str,unsigned char * Arg,const unsigned char step);//�����޷���char�Ͳ���
void setArgInt(char *str,int * Arg,const int step);//�������Ͳ���
void test_threshold();//���ԻҶ�ֵ����
void choice_par();//ѡ�����ú���
void set_par();//���ò�������
void displayBline();//��ʾ����
void LCD_P6x8Int(byte x,byte y,byte ch[],int num);//��ʾ������

void delay(uint32 );
void delay2();
void display1();
void test_lcd();
void delay1(uint8 t);


void findleftpoint();
void findrightpoint();
void findleftwhite();
void findrightwhite();
int CrossRecognition();//�ж��Ƿ���ʮ���Լ��������ʮ�ֵ�״̬��
void CrossConduct(int flag);//ʮ�ִ�����


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