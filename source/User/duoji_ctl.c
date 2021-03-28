#include "duoji_ctl.h"

extern unsigned char const adjust_table[];//����ʧ�������������
extern uint8 start_write;
extern uint8 speed_flag;
extern uint8 big_left,big_right;//���ң������־
extern uint8 zhidao_count_flag;
extern uint8 zhidao_state_num;
extern uint8 zhidao_count_flag;
extern uint8 ls_flag;
#define test_port UART_0
extern int simple_ql;//����
short int count_xielv[15];//�����жϺͼ�������б��
extern uint8 all_speed_flag;
uint8 start_line;//�����߼�����ʼ����Ч��־
uint8 end_line;//�����߼�����ʼ����Ч��־
//uint8 lost_left_start=1;//����߶�ʧ��ʼ�б�־
//uint8 lost_right_start=1;//�ұ��߶�ʧ��ʼ�б�־
uint8 lost_left_end=50;//����߶�ʧ�н�����־
uint8 lost_right_end=50;//�ұ��߶�ʧ�н�����־
uint8 near_zhidao=0;
uint8 middle_zhidao=0;
uint32 left_lost=0;
uint32 right_lost=0;
extern int judge_two[H];
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
�������ƣ�seg_pid
��������: ����ֶ�PID����
������
*******************************************************************************/
void seg_pid()
{
  get_Kp();
  get_D_slope();
  if(zhidao_count_flag==1)
  {
	if(valid_line>53)//100CM
	  {
		all_speed_flag=7;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff);
	  }
	  else if(valid_line>47)//80CM
	  {
		all_speed_flag=6;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff);
	  }
	  else if(valid_line>32)//60CM
	  {
		all_speed_flag=5;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff);
	  }
	  else if(valid_line>20)//40CM
	  {
		all_speed_flag=4;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff);
	  }
	  else if(valid_line>5)//20CM
	  {
		all_speed_flag=3;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff);
	  }
	  else if(valid_line>3)//10CM
	  {
		all_speed_flag=2;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff);
	  }
	  else
	  {
		all_speed_flag=1;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff);
	  }
  }
  else
  {
	if(valid_line>53)//100CM
	  {
		all_speed_flag=7;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff+D_slope);
	  }
	  else if(valid_line>47)//80CM
	  {
		all_speed_flag=6;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff+D_slope);
	  }
	  else if(valid_line>32)//60CM
	  {
		all_speed_flag=5;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff+D_slope);
	  }
	  else if(valid_line>20)//40CM
	  {
		all_speed_flag=4;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff+D_slope);
	  }
	  else if(valid_line>5)//20CM
	  {
		all_speed_flag=3;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff+D_slope);
	  }
	  else if(valid_line>3)//10CM
	  {
		all_speed_flag=2;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff);
	  }
	  else
	  {
		all_speed_flag=1;
		dj_pid_num=(short int)(dj_center+dj_Kp*even_diff);
	  }
  }
  //dj_pid_num=dj_pid_num+(dj_pid_num-last_turn);
  if(dj_pid_num>dj_center)//dj_center_max��ͷ�ļ��ж���Ϊ����������ֵ
  {
		pick_way=0;
		jiaodu_num=(int)(dj_pid_num+(dj_pid_num-dj_center)*1.5);
  }
  else if(dj_pid_num<dj_center)
  {
		pick_way=2;
		jiaodu_num=(int)(dj_pid_num-(int)(dj_center-dj_pid_num)*1.5);
  }
  else
  {
	pick_way=1;
  }
  if(jiaodu_num>dj_left_max)//dj_left_max��ͷ�ļ��ж���Ϊ����������ֵ
	jiaodu_num=dj_left_max;
  if(jiaodu_num<dj_right_max)//dj_right_max��ͷ�ļ��ж���Ϊ����������ֵ
	jiaodu_num=dj_right_max;
}
/*******************************************************************************
�������ƣ�get_even_diff
��������: ��ȡ��Ч����������ƫ��ƽ��ֵ
������
*******************************************************************************/
void get_even_diff()
{
  uint8 line;
  int diff=0;
  for(line=1;line<valid_line;line++)
  {
		diff=diff+(Pick_table[line]-V/2);
  }
  if(valid_line>52)
  {
	for(line=48;line<53;line++)
    {
		far_diff=far_diff+(Pick_table[line]-V/2);
    }
  }
  else
  {
	far_diff=V/2;
  }
  far_diff=far_diff/5;
  even_diff=diff/valid_line/2;//���Դ˴���2
}
/*******************************************************************************
�������ƣ�get_Kp
��������: ��ȡPID ���Ʋ���Kp
������
*******************************************************************************/
void get_Kp()
{
  if(zhidao_count_flag==1)//�����ֱ�� ��Ч������Խ�࣬kֵ����Сһ�㣬���ٲ���Ҫ�ĵ���
 {
	  if(valid_line>53)
	  {
		dj_Kp=(0.8+(59-valid_line)/32.0+Bmq/300);
	  }
	  else if(valid_line>47)
	  {
		dj_Kp=(0.9+(59-valid_line)/30.0+Bmq/280);
	  }
	  else if(valid_line>32)
	  {
		dj_Kp=(1.0+(59-valid_line)/28.0+Bmq/220);
	  }
	  else if(valid_line>20)
	  {
		dj_Kp=(1.0+(59-valid_line)/26.0+Bmq/190);
	  }
	  else if(valid_line>5)
	  {
		dj_Kp=(1.1+(59-valid_line)/24.0+Bmq/190);
	  }
	  else if(valid_line>3)
	  {
		dj_Kp=(1.2+(59-valid_line)/22.0+Bmq/180);
	  }
	  else 
	  {
		dj_Kp=(1.2+(59-valid_line)/20.0+Bmq/150);
	  }
  }
  else//����Ļ�k��һЩ�����ڼ�ʹ���иı�
  {
	  if(valid_line>53)
	  {
		dj_Kp=(1.1+(59-valid_line)/18.0+Bmq/195);
	  }
	  else if(valid_line>47)
	  {
		dj_Kp=(1.2+(59-valid_line)/18.0+Bmq/145);
	  }
	  else if(valid_line>32)
	  {
		dj_Kp=(1.4+(59-valid_line)/16.0+Bmq/135);
	  }
	  else if(valid_line>20)
	  {
		dj_Kp=(1.6+(59-valid_line)/15.0+Bmq/130);
	  }
	  else if(valid_line>5)
	  {
		dj_Kp=(1.6+(59-valid_line)/13.0+Bmq/135);
	  }
	  else if(valid_line>3)
	  {
		dj_Kp=(1.7+(59-valid_line)/10.0+Bmq/135);
	  }
	  else 
	  {
		dj_Kp=(1.6+(59-valid_line)/8.0+Bmq/100);
	  }
  }
}
/*******************************************************************************
�������ƣ�get_D_slope
��������: ��ȡPID ���Ʋ���б��
������
*******************************************************************************/
void get_D_slope()
{
   if(valid_line>53)//
  {
		D_slope=10*(Pick_table[valid_line-2]-Pick_table[valid_line-4])/valid_line;
  }
  else if(valid_line>47)
  {
		D_slope=80*(Pick_table[valid_line-1]-Pick_table[valid_line-3])/valid_line;
  }
  else if(valid_line>32)
  {
		D_slope=120*(Pick_table[valid_line-1]-Pick_table[valid_line-3])/valid_line;
  }
  else if(valid_line>20)
  {
		D_slope=100*(Pick_table[valid_line-1]-Pick_table[valid_line-3])/valid_line;
  }
  else if(valid_line>5)
  {
		D_slope=80*(Pick_table[valid_line-1]-Pick_table[valid_line-3])/valid_line;
  }
}
/*******************************************************************************
�������ƣ�speed_ctl()
��������: �ٶȿ���
������
*******************************************************************************/
void speed_ctl()
{
  /*   if((valid_line<52)&&(Bmq>120))
	{
	 	buzzer_on();
	 	brake(40);
		// buzzer_off();
	}
	else if((valid_line<48)&&(Bmq>120))
	{
	    buzzer_on();
	 	brake(40);
	}
	else if(zhidao_count_flag==1)//�޸�
	{
	    //center_led();
		ideal_Bmq=zhidao_speed;
		//zhidao_count++;
	}
	else if(slow_down_num>0)
	{
	  slow_down_num--;
	  ideal_Bmq=40;
	}
	else*/ 
	{
	  if(all_speed_flag==7)
		{
		  ideal_Bmq=CD_speed;
		}
		else if(all_speed_flag==6)
		{
		  ideal_Bmq=CD_speed;
		}
		else if(all_speed_flag==5)
		{
		  ideal_Bmq=CD_speed-1;
		}
		else if(all_speed_flag==4)
		{
		  ideal_Bmq=CD_speed-1;
		}
		else if(all_speed_flag==3)
		{
		  ideal_Bmq=CD_speed-1;
		}
		else if(all_speed_flag==2)
		{
		  ideal_Bmq=CD_speed-1;
		}
		else if(all_speed_flag==1)
		{
		  ideal_Bmq=CD_speed-1;
		}
	}
	/////////////�������ֱ��////////////
	/*if(zhidao_count_flag==1)//�޸�
	{
	    //center_led();
		ideal_Bmq=zhidao_speed;
		zhidao_count++;
	}
	else
	{
	  if(zhidao_count>3&&valid_line<45)
		slow_down_num=2;
	  else
		zhidao_count=0;
	}*/
}
/*******************************************************************************
�������ƣ�saidao_judge_new
��������: �����ж��·���
������
*******************************************************************************/
void saidao_judge_new()
{
	uint8 line,i;
	for(line=5,i=0;line<valid_line;line++)
	{
	  judge_xielv[i]=Pick_table[line+1]-Pick_table[line];
	  i++;
	}
}
/*******************************************************************************
�������ƣ�test_start_line
��������: �����ж��·���
������
*******************************************************************************/
void test_start_line(uint8 line)
{
	uint16 start_pixel;
	uint16 end_pixel;
	uint16 pixel;
	uint16 judgeHedge1=0;//������
	uint16 judgeHedge2=0;
	
	uint16 judgeLedge1=0;
	uint16 judgeLedge2=0;
	uint32 black_count=0;
	if(Bline_right[line-1]>15)
		start_pixel=Bline_right[line-1]+5;
	else
	    start_pixel=Bline_right[line-1];
	if(Bline_left[line-1]<(V-15))
		end_pixel=Bline_left[line-1]-5;
	else
		end_pixel=Bline_left[line-1];
	for(pixel=start_pixel;pixel<end_pixel;pixel++)
	{
	  if(abs_sub(video[line][pixel],video[line][pixel+3])<80)
	  {
		if((video[line][pixel]-video[line][pixel+3])<-20)
		  {
			if(judgeHedge1==0)
				judgeHedge1=pixel;
			else 
			{
			    if((pixel-judgeHedge1)>15)
				{
				  judgeHedge2=pixel;
				}
				else
				{
				  return;
				}
			}
		  }
		  if((video[line][pixel]-video[line][pixel+3])>20)
		  {
			if(judgeLedge1==0)
				judgeLedge1=pixel;
			else
			{
				if((pixel-judgeLedge1)>15)
			  	{
				  judgeLedge2=pixel;
			   	}
			   	else
				{
				  return;
				}
			}
		  }
	  }
	}
	//if((judgeHedge2!=0)&&(judgeLedge2!=0))
	{
	  start_pixel+=3;
	  end_pixel-=3;
	  for(pixel=start_pixel;pixel<end_pixel;pixel++)
		{
		  if(video[line][pixel]<Cmp)
		  {
			black_count++;
		  }
		}
	  if(black_count>45)
	  //LCD_P6x8Int(64,7,"me:",(int)(black_count));
	  if(black_count>45)
	  	start_end_flag=1;
	}
}
/*******************************************************************************
�������ƣ�find_s
��������: �����ж��·���
������
*******************************************************************************/
void find_s()
{
	uint8 line;
	uint8 change_flag=0;
	uint8 change_num=0;
	uint8 left_flag=0;
	uint8 right_flag=0;
	//uint8 change[55];
	for(line=5;line<(valid_line);line++)
	{
	  if(Pick_table[line+1]>Pick_table[line])
	  {
		left_flag=1;
		if(right_flag==1)
		{
		  right_flag=0;
		  change_flag=1;
		}
		else
		   change_flag=0;
	  }
	  else if(Pick_table[line+1]<Pick_table[line])
	  {
		right_flag=1;
		if(left_flag==1)
		{
		  left_flag=0;
		  change_flag=1;
		}
		else
		  change_flag=0;
	  }
	  if(change_flag==1)
	  {
		//change[change_num]=line;
		change_num++;
	  }
	}
	if((change_num)>0)
	{
	  lcd_int(8,"cha:",change_num);
	  if(abs(even_diff)<50)
	  {
		ls_flag=1;
		//LCD_Print(88,6,"Y-s");
	  }
	}
	else
	{
	  ls_flag=0;
	  //LCD_Print(88,6,"N-s");
	}
	
}
/*******************************************************************************
�������ƣ�If_straight
��������: �ж�С���Ƿ���ֱ���Ϸ���
��������
*******************************************************************************/
void If_straight()
{
  uint8 line;
  uint8 count=0;
  if(valid_line>53)
  {
	for(line=1;line<valid_line;line++)
	{
	  if(abs_sub(Pick_table[line],V/2)>20)
		count++;
	}
	if(LR_diff<70&&(count<2))
	  zhidao_count_flag=1;
  }
  else
  {
	zhidao_count_flag=0;
  }
  last_zhidao_flag=zhidao_count_flag;
}
/*******************************************************************************
�������ƣ�If_LStraight
��������: �ж�С���Ƿ��ڳ�ֱ���Ϸ���
��������
*******************************************************************************/
void If_LStraight()
{
  uint8 line,break_line=0;
  //uint8 count=0;
  for(line=H-5;line>H-valid_line;line--)
  {
	  if(abs_sub(Pick_table[line+1],V/2)>10)
	  {
		if(abs_sub(Pick_table[line+1],V/2)>15)
			break_line=line;
		break;
	  }
          break_line = line;
  }
  if(break_line<H-45)
	  zhidao_count_flag=1;
  else
  {
	zhidao_count_flag=0;
  }
  //uart_printf(test_port,"break_line:%d\n",break_line);
  //uart_printf(test_port,"zhidao__flag:%d\n",zhidao_count_flag);
  last_zhidao_flag=zhidao_count_flag;
}
/*******************************************************************************
�������ƣ�If_straight_new
��������: �ж�С���Ƿ���ֱ�����·���
��������
*******************************************************************************/
void If_straight_new()
{
 uint8 i,line,zd_xielv[11],N_zhidao=0;
 // uint16 xielv_temp=0;
  if(valid_line>45)
  {
	for(line=2,i=0;line<44;line=line+4,i++)
	  {
		
		zd_xielv[i]=abs_sub(Pick_table[line+4],Pick_table[line]);
	  }
	  for(i=0;i<11;i++)
		 if(zd_xielv[i]>4)
		   N_zhidao++;
	  if(N_zhidao<4)
	  {
		zhidao_count_flag=1;
	  }
	  else
	  {
		zhidao_count_flag=0;
	  }
  }
  else
  {
	zhidao_count_flag=0;
  }
  last_zhidao_flag=zhidao_count_flag;
}
/*******************************************************************************
�������ƣ�judgeRoad
��������: �ж���������
������
*******************************************************************************/
void judgeRoad()
{
  if(Shi_zi_flag==1)
  {
	  roadFlag=2;
  }
  else
  {
	  if(zhidao_count_flag==1)
	  {
		roadFlag=0;
	  }
	  else
	  {
		roadFlag=1;
	  }
  }
}
/*******************************************************************************
�������ƣ�get_valid_line
��������: �����Ч��
������
*******************************************************************************/
void get_valid_line()
{
  uint8 line;
  for(line=2;line<60;line++)
  {
	if((abs_sub(Pick_table[line],Bline_left[line])<55)
	   ||(abs_sub(Pick_table[line],Bline_right[line])<55))//ע�ⶪʧ�������߶�����Ϊ140
	{
	  valid_line=line;
	  break;
	}
  }
}
/*******************************************************************************
�������ƣ�get_vl_new
��������: �����Ч���º���
������
*******************************************************************************/
void get_vl_new1()
{
  uint8 line,lost_flag_temp=0;; //ע�⣺ȱ��ǰ�����жϺ���
  for(line=2;line<H;line++)
  {
	if(Bline_left[line]<Bline_right[line])//ע�ⶪʧ�������߶�����Ϊ140
	{
		valid_line=line-1;//���ֵ��ֱ�����кܺõ��ȶ���
		break;
	}
  }
  if(line==H)
	  valid_line=59;
  else if(valid_line>8)
  {
	for(line=valid_line-5;line<valid_line;line++)
  	{
		if((abs_sub(Bline_left[line],Bline_left[line+1])>8)
	   		||(abs_sub(Bline_right[line],Bline_right[line+1])>8))//ע�ⶪʧ�������߶�����Ϊ140
		{
	 	 lost_flag_temp++;
		}
 	}
	if(lost_flag_temp>1)
	  valid_line=valid_line-8;
  }
}
/*******************************************************************************
�������ƣ�get_vl_new
��������: �����Ч���º���
������
*******************************************************************************/
void get_vl_new2()
{
  uint8 line; //ע�⣺ȱ��ǰ�����жϺ���
  for(line=2;line<H;line++)
  {
	if(abs_sub(Bline_left[line],Bline_right[line])<20)//ע�ⶪʧ�������߶�����Ϊ140
	{
		valid_line=line-1;//���ֵ��ֱ�����кܺõ��ȶ���
		break;
	}
  }
  if(line==H)
	  valid_line=59;
}
/*******************************************************************************
�������ƣ�send_xielv
��������: ����б��
������
*******************************************************************************/
void send_xielv()
{
  uint8 i;
  for(i=0;i<15;i++)
  {
	//send_data(count_xielv[i]);
  }
  //uart_send1(UART0,'\n');
}
/*******************************************************************************
�������ƣ�way_control
��������: 
������
*******************************************************************************/
void way_control()
{
  	if(abs_sub(jiaodu_num,last_turn)>200)//���ÿ���dj_center���ߵ�
	{
	  	slow_down_num=3;
	}
	else
	  slow_down_num=0;
 	if(pick_way==0)
	{
		//uart_printf(test_port, "left\n");
                //uart_printf(test_port,"%d\n",jiaodu_num);
                duoji_control(jiaodu_num,left_way);  
	} 
	else if(pick_way==2)
	{
		//uart_printf(test_port, "right\n");
                //uart_printf(test_port,"%d\n",jiaodu_num);
                duoji_control(jiaodu_num,right_way);	
	}
	else
	{
		//uart_printf(test_port, "is_center\n");
                duoji_control(dj_center,center_way);//ע������ ����ȥ��
	}
	///////////////////////�ٶȿ��Ʋ�������//////////////////////////
	/*if(last_turn>1384&&jiaodu_num<1304||last_turn<1304&&jiaodu_num>1384)
	{
	  slow_down_num=4;
	}
	else*/
	last_turn=jiaodu_num;
	last_pick_way=pick_way;
}
/*******************************************************************************
�������ƣ�stop_car
��������:ͣ��
������
*******************************************************************************/
void stop_car()
{
    if(Bmq>30)
	 //buzzer_on();
	stop_brake(0);
	if(Bmq<30)
		//disable_pit_int(PIT1);//�ر�PIT1�ж�
	//disable_pit_int(PIT0);//�ر�PIT0�ж�
	FTM2_C0V=0;
    FTM2_C1V=0;
}
/*******************************************************************************
�������ƣ�test_start
��������:���������
������
*******************************************************************************/
void test_start()
{
  uint8 line,temp1_flag=0,temp2_flag=0,start_flag1=0,start_flag2=0;
  uint8 temp_line1,temp_line2;
  uint16 pixel_1,pixel_2;
  for(line=30;line>1;line--)
  { 
	pixel_1=100;
	pixel_2=180;
	if(video[line][pixel_1]>180)
	{
	  temp1_flag++;
	}
	if(video[line][pixel_2]>180)
	{
	  temp2_flag++;
	}
	if(temp1_flag>1)
	{
   	 if(video[line][pixel_1]<Cmp)
		{
	  		start_flag1++;
			temp_line1=line;
		}
	}
	if(temp2_flag>1)
	{
   	 if(video[line][pixel_2]<Cmp)
		{
	  		start_flag2++;
			temp_line2=line;
		}
	}
	if(start_flag1>0&&start_flag2>0)
	  break;
	pixel_1+=2;
	pixel_2-=2;
  }
  if(start_flag1>0&&start_flag2>0)
  {
	if((video[temp_line1][30]>180||video[temp_line1+1][30]>180)
	   &&(video[temp_line2][V-30]>180||video[temp_line1+1][V-30]>180))
		start_end_flag=1;
  }
}

/*******************************************************************************
�������ƣ�test_start_new
��������:���������
������
*******************************************************************************/
void test_start_new()
{
  uint8 i,N_start=0,line,write_flag=0;
  uint8 start_left=0,start_right=0,second_flag_left=0,second_flag_right=0;
  uint16 pixel_1,pixel_2;
  if(valid_line>40)
  {
	  for(i=0;i<25;i++)
	  {
		if(abs(judge_xielv[i])>5)
		  N_start++;
	  }
	  //if(N_start<5)
	  {
		for(line=2;line<32;line++)
		{
		  pixel_1=(Bline_left[line]+Pick_table[line])/2;
		  if(Image_Data[line][pixel_1-10]<Cmp)
		  {
			start_left=1;
		  }
		  pixel_2=(Bline_right[line]+Pick_table[line])/2;
		  if(Image_Data[line][pixel_2+10]<Cmp)
		  {
			start_right=1;
		  }
		  if(start_left==1&&start_right==1)
		  {
			for(i=0;i<10;i++)
			{
			  if(Image_Data[line+1][pixel_1-i]<Cmp)
			  		second_flag_left++;
			}
			for(i=0;i<10;i++)
			{
			  if(Image_Data[line+1][pixel_2+i]<Cmp)
			  		second_flag_right++;
			}
			for(i=0;i<10;i++)
			{
			  if(Image_Data[line+3][Pick_table[line+3]-5+i]>start_write)
			  		write_flag++;
			}
			if(second_flag_left>5&&second_flag_right>5&&write_flag>5)
			{
			  start_end_flag=1; 
			  break;
			}
			else
			{
			  second_flag_left=0;
			  second_flag_right=0;
			  start_left=0;
			  start_right=0;
			}
		  }
		  else
		  {
			start_left=0;
			start_right=0;
		  }
		}
	  }
  }
  if(start_end_flag==1)
  {
  	//LCD_Print(56,6,"start!  ");
   }
  else
  {
	//LCD_Print(56,6,"n_start!");
  }
}
/*******************************************************************************
�������ƣ�brake
��������:ɲ��
������
*******************************************************************************/
void brake(uint8 speed)
{
  uint16 brake_time=0;
  if(speed>25)//�ٶȴ���25Ϊɲ��
  	ideal_Bmq=speed-25;
  else//С�ڵ���25Ϊͣ��
	ideal_Bmq=0;
  //Kp=55;
  //Ki=5;
  //Kd=25;//ɲ��pidֵ
  while(Bmq>speed)
  {
	brake_time++;
	if(brake_time>1000)
	  break;//��ֹ������ѭ��
  }
  //Kp=25;
  //Ki=5;
  //Kd=5;//������pidֵ
}
/*******************************************************************************
�������ƣ�brake
��������:ɲ��
������
*******************************************************************************/
void stop_brake(uint8 speed)
{
  uint16 brake_time=0;
  if(speed>25)//�ٶȴ���25Ϊɲ��
  	ideal_Bmq=speed-25;
  else//С�ڵ���25Ϊͣ��
	ideal_Bmq=0;
  //Kp=205;
  //Ki=5;
  //Kd=55;//ɲ��pidֵ
  while(Bmq>speed)
  {
	brake_time++;
	if(brake_time>1000)
	  break;//��ֹ������ѭ��
  }
  //Kp=25;
  //Ki=5;
  //Kd=5;//������pidֵ
}
/*******************************************************************************
�������ƣ�Is_out
��������: �жϸó�ͼ���Ƿ���Ч
������
*******************************************************************************/
void Is_out()
{
  uint8 i,flag_temp=0;
  for(i=1;i<5;i++)
  {
	if((Bline_left[i]==V/2)&&(Bline_right[i]==V/2))//10Ϊ�������������С��
	  flag_temp++;
  }
  if(flag_temp>2)
  {
	Out_flag=1;
	//show_miss(1);
  }
  else
  {
	Out_flag=0;
	//show_miss(0);
  }
  if(lost_w_count>30)
  {
	Out_flag=1;
  }
  else
  {
	Out_flag=0;
  }
  
  
}
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
�������ƣ�seg_pid
��������: ����ֶ�PID����
������
*******************************************************************************/
void seg_pid_stable()
{
  //uart_printf(test_port, "1");
  get_Kp_stable();
  //uart_printf(test_port, "2");
  get_D_slope_stable();
  //uart_printf(test_port, "3");
  D_slope=0;
  even_diff=0;
  even_diff1=0;
  if(Shi_zi_flag==1)
  {
	//LED_H;
	if(valid_line>30)
	{
	  dj_pid_num=(int)(dj_center+2*(Pick_table[10]-Pick_table[2]));
	}
	else if(valid_line>10)
	{
	   	dj_pid_num=(int)(dj_center+2*(Pick_table[10]-Pick_table[2]));
	}
	if((dj_pid_num-dj_center)>21)
	   		dj_pid_num=dj_center+21;
		else if((dj_pid_num-dj_center)<-21)
			dj_pid_num=dj_center-21;
  }
  else
  {
	  if(zhidao_count_flag==1)
	  {
		if(valid_line>45)//100CM
		  {
			all_speed_flag=7;
//			if(even_diff>0)
//				dj_pid_num=(int)(dj_center+dj_Kp*even_diff*even_diff+dj_Kp*even_diff1/3+3*D_slope/valid_line);
//			else if(even_diff<0)
			    dj_pid_num=(int)(dj_center+dj_Kp*even_diff+dj_Kp*even_diff1/3+3*D_slope/valid_line);
		  }
		  else //80CM
		  {
			all_speed_flag=6;
//			if(even_diff>0)
//				dj_pid_num=(int)(dj_center+dj_Kp*even_diff*even_diff+dj_Kp*even_diff1/3+3*D_slope/valid_line);
//			else if(even_diff<0)
			    dj_pid_num=(int)(dj_center+dj_Kp*even_diff+dj_Kp*even_diff1/3+3*D_slope/valid_line);
		  }
		 if((dj_pid_num-dj_center)>36)
			  dj_pid_num=dj_center+36;
		 else if((dj_pid_num-dj_center)<-36)
			dj_pid_num=dj_center-36;
		 //LED_H;
	  }
	  else
	  {
		
		  {
			 //LED_L;
                         //uart_printf(test_port, "%d\n",valid_line);
			 if(valid_line>53)//100CM
			  {
				all_speed_flag=7;
				dj_pid_num=(int)(dj_center+(dj_Kp)*even_diff+dj_Kp*even_diff1/2+5.0*D_slope_near);
			  }
			  else if(valid_line>45)//100CM
			  {
				all_speed_flag=7;
				dj_pid_num=(int)(dj_center+(dj_Kp)*even_diff+dj_Kp*even_diff1/2+5.0*D_slope_near);
			  }
			  else if(valid_line>35)//80CM
			  {
				  all_speed_flag=6;
				  dj_pid_num=(int)(dj_center+(dj_Kp)*even_diff+dj_Kp*even_diff1/2+8.0*D_slope_near);	
			  }
			   else if(valid_line>25)//80CM
			  {
				  all_speed_flag=6;
				  dj_pid_num=(int)(dj_center+(dj_Kp)*even_diff+dj_Kp*even_diff1/2+8.5*D_slope_near);	
			  }
			  else 
			  {
				dj_pid_num=(int)(dj_center+(dj_Kp)*even_diff+dj_Kp*even_diff1/2+7*D_slope_near);
			  }
		  }
	  }
  }
  //uart_printf(test_port, "4");
  //if(valid_line<53)
  dj_pid_num=dj_pid_num+0.1*Position_diff;
  //dj_pid_num=(int)(dj_center+dj_Kp*(Pick_table[2]+Pick_table[3]-V)/5);
  /*if(abs(even_diff)>15&&(abs(jiaodu_num-dj_center)<25))
  {
	jiaodu_num=jiaodu_num+0*D_slope_near;
  }*/
  //uart_printf(test_port, "%d\n",dj_pid_num);
  if(zhidao_count_flag==0)
  {
	  if(dj_pid_num>dj_center)//dj_center_max��ͷ�ļ��ж���Ϊ����������ֵ
	  {
		pick_way=0;
		jiaodu_num=(int)(dj_pid_num+(dj_pid_num-dj_center)*biLi);
	  }
	  else if(dj_pid_num<dj_center)
	  {
		jiaodu_num=(int)(dj_pid_num+(dj_pid_num-dj_center)*biLi);
		pick_way=2;
	  }
	  else
	  {
		pick_way=1;
	  }
  }
  //uart_printf(test_port, "5");
  //jiaodu_num = dj_center + (jiaodu_num - dj_center) * 0.04;
  //uart_printf(test_port, "%d\n",jiaodu_num);
  if(jiaodu_num>dj_left_max)//dj_left_max��ͷ�ļ��ж���Ϊ����������ֵ
	jiaodu_num=dj_left_max;
  if(jiaodu_num<dj_right_max)//dj_right_max��ͷ�ļ��ж���Ϊ����������ֵ
	jiaodu_num=dj_right_max;
  //uart_printf(test_port, "6");
}
/*******************************************************************************
�������ƣ�xielv_lvbo
��������: б�ʷ��˲� ȥ����Ч��
��������
*******************************************************************************/
void xielv_lvbo()
{
  uint8 line;
  for(line=1;line<H;line++)
  {
	if((abs_sub(Bline_left[line],Bline_left[line-1])>JUDGE_DIFF)
	   ||(abs_sub(Bline_right[line],Bline_right[line-1])>JUDGE_DIFF))
	{
	  Deal_flag[line] |= INVALID_LINE;
	}
  }
}
/*******************************************************************************
�������ƣ�bDistance
��������: ������ȷ��˲� ȥ����Ч��
��������
*******************************************************************************/
void bDistance()
{
  uint8 line;
  uint16 distance = 100; 
  for(line=1;line<H;line++)
  {
	if(((Bline_left[line]-Bline_right[line])>distance)||(Bline_left[line]<Bline_right[line]))
	{
	  Deal_flag[line] |= INVALID_LINE;
	}
  }
}
/*******************************************************************************
�������ƣ�get_even_diff
��������: ��ȡ��Ч����������ƫ��ƽ��ֵ
������
*******************************************************************************/
void get_even_diff_new()
{
  uint8 line;
  uint8 v_line1=0;//����
  uint8 v_line2=0;
  int diff1=0;
  int diff2=0;
  even_diff=0;
  even_diff1=0;
  
  if(valid_line>30)
  {
	  for(line=1;line<30;line++)
	  {
		if((Deal_flag[line]&INVALID_LINE)==0)
		{
		  v_line1++;
		  diff1=diff1+(Pick_table[line]-V/2)*(1+(int)(line/60.0));
		}
	  }
	  even_diff=(diff1/v_line1);
	  for(line=30;line<valid_line;line++)
	  {
		if((Deal_flag[line]&INVALID_LINE)==0)
		{
		  v_line2++;
		  diff2=diff2+(Pick_table[line]-V/2)*(1+(int)(line/60.0));
		}
	  }
	  even_diff1=diff2/v_line2;
  }
  else //С��30�� ��ȡ����Ч�н�ֹ
  {
	for(line=1;line<valid_line;line++)
	  {
		if((Deal_flag[line]&INVALID_LINE)==0)
		{
		  v_line1++;
		  diff1=diff1+(Pick_table[line]-V/2)*(1+(int)(line/60.0));
		}
	  }
	  even_diff=(diff1/v_line1);
  }
}


/*******************************************************************************
�������ƣ�getBlineCenter
��������: ��ȡ����������
��������
*******************************************************************************/
void getBlineCenter()
{
  uint8 line;
  for(line=0;line<H;line++)//ɨ��ÿһ�еõ��ڱ�����
  {
	/*if(((Deal_flag[line]&DEAL_LEFT_LOST)==0)&&((Deal_flag[line]&DEAL_RIGHT_LOST)!=0))//�ұ��߶�ʧ
	{
	  	Pick_table[line]=Bline_left[line]-30;
	}
	else if(((Deal_flag[line]&DEAL_LEFT_LOST)!=0)&&((Deal_flag[line]&DEAL_RIGHT_LOST)==0))//����߶�ʧ
	{
	  	Pick_table[line]=Bline_right[line]+30;
	}
	else
	{
	  Pick_table[line]=(Bline_left[line]+Bline_right[line])/2;
	}*/
        if(Pick_flag[line] == RIGHT_LOST_W)//RIGHT_LOST_W=0x4u
        {
            Bline_right[line] = V-1;
        }
        if(Pick_flag[line] == LEFT_LOST_W)//0x1u
        {
            Bline_left[line] = 0;
        }
        
        Pick_table[line]=(Bline_left[line]+Bline_right[line])/2;
  }
}

/*******************************************************************************
�������ƣ�averageLvBo
��������: ��ֵ�˲�
��������
*******************************************************************************/
void averageLvBo()
{
  uint8 line;
  for(line=2;line<(H-2);line++)
  {
	Pick_table[line]=(Pick_table[line-1]+Pick_table[line-2]+Pick_table[line]
					  +Pick_table[line+1]+Pick_table[line+2])/5;
  }
}

/*******************************************************************************
�������ƣ�get_even_diff
��������: ��ȡ��Ч����������ƫ��ƽ��ֵ
������
*******************************************************************************/
void get_even_diff_s()
{
  uint8 line;
  uint8 v_line1=0;//����
  int diff1=0;
  even_diff=0;
  even_diff1=0;
  for(line=V-1;line>V-valid_line;line--)
  {
		if((Deal_flag[line]&INVALID_LINE)==0)
		{
		  v_line1++;
		  diff1=diff1+(Pick_table[line]-V/2)*(1+(int)((H-line)/120.0));
		}
	}
  even_diff=(diff1/v_line1);
}
/*******************************************************************************
�������ƣ�get_even_diff
��������: ��ȡ��Ч����������ƫ��ƽ��ֵ
������
*******************************************************************************/
void get_even_diff_near()
{
  uint8 line;
  uint8 v_line1=0;//����
  int diff1=0;
  for(line=V;line>V-30;line--)
  {
		if((Deal_flag[line]&INVALID_LINE)==0)
		{
		  v_line1++;
		  diff1=diff1+(Pick_table[line]-V/2)*(1+(int)(line/60.0));
		}
	}
  even_diff_near=(diff1/v_line1);
}
/*******************************************************************************
�������ƣ�void find_S_road
��������: ����S���
������
*******************************************************************************/
void find_S_road()
{
  uint8 line;
  uint8 get_line[10]={0};
  uint8 v_line1=0;//����
  for(line=1;line<valid_line;line++)
  {
		if((Deal_flag[line]&INVALID_LINE)==0)
		{
		  if(Pick_table[line]==(V/2+even_diff))
		  {
			if(v_line1<10)
			{
			  get_line[v_line1]=line;
			  v_line1++;
			}
		  }
		}
	}
  if(v_line1==2)
  {
	if(abs_sub(get_line[0],get_line[1])>5)
	  S_road_flag=1;
	else
	  S_road_flag=0;
  }
}
/*******************************************************************************
�������ƣ�get_Curve
��������: ��ȡCurveֵ
������
*******************************************************************************/
void get_Curve()
{
  uint8 line;
  uint8 v_line1=0;//����
  uint16 temp_diff;
  temp_diff=V/2+even_diff;
  volatile int curve_temp=0;
  Curve_near=0;
  Curve_middle=0;
  Curve_far=0;
  if(valid_line<=20)
  {
	  for(line=0;line<valid_line;line--)
	  {
			if((Deal_flag[line]&INVALID_LINE)==0)
			{
			  v_line1++;
			  curve_temp=curve_temp+abs(Pick_table[line]-temp_diff);
			}
		}
	  Curve_near=(curve_temp/v_line1);
	  return ;
  }
  else
  {
	 for(line=1;line<20;line++)
	  {
			if((Deal_flag[line]&INVALID_LINE)==0)
			{
			  v_line1++;
			  curve_temp=curve_temp+abs(Pick_table[line]-temp_diff);
			}
		}
	  Curve_near=(curve_temp/v_line1);
	  v_line1=0;
	  for(line=20;line<valid_line;line++)
	  {
			if((Deal_flag[line]&INVALID_LINE)==0)
			{
			  v_line1++;
			  curve_temp=curve_temp+abs(Pick_table[line]-temp_diff);
			}
		}
	  Curve_middle=(curve_temp/v_line1);
  }
}
/*******************************************************************************
�������ƣ�get_Curve
��������: ��ȡCurveֵ���۲�棩
������
*******************************************************************************/
void get_Curve_whole()
{
  uint8 line;
  uint8 v_line1=0;//����
  int temp_diff;
  temp_diff=V/2+even_diff;
  int curve_temp=0;
  for(line=1;line<valid_line;line++)
  {
		if((Deal_flag[line]&INVALID_LINE)==0)
		{
		  v_line1++;
		  curve_temp=curve_temp+abs(Pick_table[line]-temp_diff);
		}
	}
  Curve=(curve_temp/(int)v_line1);
}
/*******************************************************************************
�������ƣ�get_Kp
��������: ��ȡPID ���Ʋ���Kp
������
*******************************************************************************/
void get_Kp_stable()
{
  //float Kp_temp;
  if(zhidao_count_flag==1)
 {
	  if(valid_line>50)
	  {
		dj_Kp=(3.8+(59-valid_line)/40.0+Bmq/300.0);
		//dj_Kp=(uint8)dj_Kp;
	  }
	  else 
	  {
		dj_Kp=(3.0+(59-valid_line)/40.0+Bmq/280.0);
		//dj_Kp=(uint8)dj_Kp;
	  }
  }
  else
  {
	  if(valid_line>53)
	  {
		dj_Kp=(1.5+(59-valid_line)/20.0+Bmq/165.0);
		//dj_Kp=(uint8)dj_Kp;
	  }
	  else if(valid_line>40)
	  {
		dj_Kp=(4.5+(59-valid_line)/20.0+Bmq/165.0);
		//dj_Kp=(uint8)dj_Kp;
	  }
	  else if(valid_line>30)
	  {
		dj_Kp=(4.6+(59-valid_line)/20.0+Bmq/185.0);
		//dj_Kp=(uint8)dj_Kp;
	  }
	  else if(valid_line>20)
	  {
		dj_Kp=(4.7+(59-valid_line)/20.0+Bmq/175.0);
		//dj_Kp=(uint8)dj_Kp;
	  }
	  else 
	  {
		dj_Kp=(3.8+(59-valid_line)/20.0+Bmq/185.0);
		//dj_Kp=(uint8)dj_Kp;
	  }
  }
}
/*******************************************************************************
�������ƣ�get_D_slope
��������: ��ȡPID ���Ʋ���б��
������
*******************************************************************************/
void get_D_slope_stable()
{
  /*if(valid_line>53)
  {
	D_slope=Pick_table[valid_line-2]-Pick_table[4];
  }
  else if(valid_line>47)
  {
	D_slope=Pick_table[valid_line-2]-Pick_table[4];
  }
  else if(valid_line>32)
  {
	D_slope=Pick_table[valid_line-2]-Pick_table[4];
  }
  else if(valid_line>20)
  {
	D_slope=Pick_table[valid_line-2]-Pick_table[4];
  }
  else if(valid_line>5)
  {
	D_slope=Pick_table[valid_line-1]-Pick_table[2];
  }
  else if(valid_line>4)
  {
	D_slope=Pick_table[valid_line]-Pick_table[2];
  }*/
  if(valid_line>53)
  {
	D_slope=Pick_table[H-valid_line+2]-Pick_table[H-2];
  }
  else
  {
	D_slope=(Pick_table[H-valid_line+2]-Pick_table[H-2]);
  }
  D_slope_near=(Pick_table[H-25]-Pick_table[H-2]);
  D_slope_near=regression(Pick_table,2,valid_line)*2;
}
/*******************************************************************************
�������ƣ�speed_ctl()
��������: �ٶȿ���
������
*******************************************************************************/
void speed_ctl_stable()
{
  uint16 count_Bmq;
  static uint16 last_count_Bmq;
     if((valid_line<40)&&(Bmq>120))
	{
	 	//buzzer_on();
	 	//brake(60);
		// buzzer_off();
	}
	
	else
    if(Shi_zi_flag==1)
	{
	  count_Bmq=CD_speed+3;
	}
    else if(slow_down_num==3)
	{
	  count_Bmq=CD_speed-5;
	}
	else
	{
	  count_Bmq=CD_speed-LR_diff/10;
	}
	/////////////�������ֱ��////////////
	if(zhidao_count_flag==1)//�޸�
	{
	    //center_led();
	    if(valid_line>53)
			count_Bmq=zhidao_speed;
		else if(valid_line>45)
			count_Bmq=zhidao_speed-5;
	}
	if(last_count_Bmq>=CD_speed&&((count_Bmq-last_count_Bmq)<-15))
	{
	  	ideal_Bmq=0;
		//brake(CD_speed-5);
	}
	else
    count_Bmq=zhidao_speed;
    ideal_Bmq=count_Bmq;
    last_count_Bmq=ideal_Bmq;
}

/*******************************************************************************
�������ƣ�get_LR_diff
��������: ��ȡLR_diffֵ
������
*******************************************************************************/
void get_LR_diff()
{
  uint8 line;
  uint16 pick_line_min=V/2;
  uint16 pick_line_max=V/2;
  //if(valid_line<=20)
  {
	  for(line=1;line<valid_line;line++)
	  {
			if((Deal_flag[line]&INVALID_LINE)==0)
			{
			  if(Pick_table[line]<pick_line_min)
				pick_line_min=Pick_table[line];
			  if(Pick_table[line]>pick_line_max)
				pick_line_max=Pick_table[line];
			}
	  }
  }
  LR_diff=pick_line_max-pick_line_min;
}
/*******************************************************************************
�������ƣ�lost_count()
��������: ͳ�ƶ�ʧ��
������
*******************************************************************************/
void lost_count()
{
  uint32 i;
  left_lost=0;
  right_lost=0;
  for(i=V-1;i>V-valid_line-1;i--)
  {
	if(Bline_left[i]==0)
	  left_lost++;
	if(Bline_right[i]==187)
	  right_lost++;
  }
  left_lost=(int)(((float)left_lost/valid_line)*10);
  right_lost=(int)(((float)right_lost/valid_line)*10);
  //lcd_int("diff:",(int)even_diff);
}