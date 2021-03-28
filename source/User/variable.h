#ifndef _VARIABLE_H_
#define _VARIABLE_H_

#include "include.h"

//////////////////////////main.c����//////////////////
extern unsigned int row;	//����ͷ�м��������240
extern uint8 video[H][V];	//�����������
extern uint8 video_deal[H][V];	//�����������
extern uint16 Bline_left[H];	//����ߴ������
extern uint16 Bline_right[H];	//�ұ��ߴ������
extern uint16 Pick_table[H];	//�����ߴ������
extern uint16 PrBline_left[H];  //ԭʼ������ߴ������
extern uint16 PrBline_right[H]; //ԭʼ���ұ��ߴ������
extern uint8  Pick_flag[H];//�����Ƿ��ҵ����߱�־����
extern uint8  Deal_flag[H];//���������Ƿ���Ч��־����
extern uint16 lost_already;
extern uint8 Pick_line;
//uint8 PickCenter_flag=0;	//��ȡ���߱�־
extern uint8 Lost_Line_count;
extern uint8 Lost_left_count;
extern uint8 Lost_right_count;
extern uint8 Near_lost;//������ʧ��־
extern uint8 Shi_zi_line;
extern uint8 Shi_zi_num;
extern uint8 Shi_zi_flag;
extern uint16 last_position;
extern uint8 stop_num;
extern int Position_diff;
extern float biLi;

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
extern uint8 start_end_flag;	//�����߱�־
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
extern uint8 start_end_flag;
extern uint8 roadFlag;//������־
extern uint8 last_roadFlag;
extern uint8 temp_shizi;//��ʱ��
extern uint32 run_time;//��������ʱ�䣨������
extern uint8 ls_flag;//СS��־
extern uint8 S_road_flag;
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
extern int D_slope_near;
extern int D_slope;		//���б�ʿ���ֵ
extern int Curve;
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
///////////////////////main.c  End/////////////////////////////

#endif