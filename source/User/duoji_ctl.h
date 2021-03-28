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
void If_straight();//判断是否是直道
void find_lost();
void test_start();
void test_start_new();
void get_vl_new();//获取有效行
void send_xielv();
void get_even_diff();//获取有效行内中心线偏离平均值
void seg_pid();//舵机分段PID控制
void get_Kp();//获得舵机控制Kp值
void get_D_slope();
void speed_ctl();
void find_lost_new();
void find_s();  //发现小S
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
void get_Curve();//获得Curve值
void get_Curve_whole();
void get_even_diff_new();//获得中心偏差测试程序
void get_Kp_stable();
void find_S_road();
void stop_car();//停车
void stop_brake(uint8 speed);
void Is_out();
//void jiaodu_count();
void find_wandao();
void brake(uint8 speed);
void far_wan();
void make_it_test();
void If_straight_new();
void If_LStraight();
void lost_count();//计算丢失行
void xielv_lvbo();// 斜率法滤波
void bDistance();//赛道宽度法滤波
void judgeRoad();//判断赛道类型
void get_LR_diff();
void test_start_line(uint8 line);//检测起跑线
void getBlineCenter();//得到黑线中心
void averageLvBo();//均值滤波
#endif 