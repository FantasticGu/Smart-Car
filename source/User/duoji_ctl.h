#ifndef __DUOJI_CTL_H__
#define __DUOJI_CTL_H__

#include "include.h"

void saidao_judge();
void saidao_judge_new();
void xielv_jiaodu(uint8 xielv_num,uint8 way_flag , uint8 *table);
void old_ctl();
void new_ctl_1();
void new_ctl_2();
void new_ctl();
void If_straight_new();
void way_control();
void test_start();
void saidao_judge_new();
void saidao_judge_new_1();
void saidao_judge_new_2();
void If_straight();//�ж��Ƿ���ֱ��
void find_lost();
void test_start();
void test_start_new();
void get_vl_new();//��ȡ��Ч��
void send_xielv();
void get_even_diff();//��ȡ��Ч����������ƫ��ƽ��ֵ
void seg_pid();//����ֶ�PID����
void get_Kp();//��ö������Kpֵ
void get_D_slope();
void speed_ctl();
void find_lost_new();
void find_s();  //����СS
void new_ctl_1();//
void way_control();
void get_valid_line();
void get_vl_new1();
void get_vl_new2();
void get_xielv();
void speed_ctl_stable();
void get_D_slope_stable();
void seg_pid_stable();
void get_even_diff_s();
void get_even_diff_near();
void get_Curve();//���Curveֵ
void get_Curve_whole();
void get_even_diff_new();//�������ƫ����Գ���
void get_Kp_stable();
void find_S_road();
void stop_car();//ͣ��
void stop_brake(uint8 speed);
void Is_out();
//void jiaodu_count();
void find_wandao();
void brake(uint8 speed);
void far_wan();
void make_it_test();
void If_straight_new();
void If_LStraight();
void lost_count();//���㶪ʧ��
void xielv_lvbo();// б�ʷ��˲�
void bDistance();//������ȷ��˲�
void judgeRoad();//�ж���������
void get_LR_diff();
void test_start_line(uint8 line);//���������
void getBlineCenter();//�õ���������
void averageLvBo();//��ֵ�˲�
#endif 