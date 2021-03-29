/*******************************************************************************
��ƽ    ̨������K66FX���ܳ�VDĸ��
����    д��CHIUSIR
��E-mail  ��chiusir@163.com
�������汾��V1.0
�������¡�2018��4��28��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��IAR7.80.4������
��Target  ��K66FX1M0VLQ18
��Crystal �� 50.000Mhz
��busclock��100.000MHz
��pllclock��200.000MHz
20180424�����µ�DMA������ʽ
******************************************************************************/
#include "include.h"
#define test_port UART_0

uint8 Image_Data[H][V];      //ͼ��ԭʼ���ݴ��
uint8_t Image_Raw[IMAGEH][IMAGEW];
volatile u8 Image_Use[LCDH][LCDW]; //ѹ����֮�����ڴ����Ļ��ʾ����94*60
u8 Pixle[LCDH][LCDW];              //��ֵ��������OLED��ʾ������
uint8_t Threshold;                  //OSTU��򷨼����ͼ����ֵ
u8  Line_Cont=0;          //�м���
u8  Field_Over_Flag=0;    //����ʶ

int OFFSET0=0;      //��Զ������������ֵ�ۺ�ƫ����
int OFFSET1=0;      //�ڶ���
int OFFSET2=0;      //�����������
int TXV=0;          //���ε���߶ȣ��Ҹ߶�




//����ͷͼ��ɼ��жϴ�������
void PORTD_ISR(void)
{     
  //���ж�PTD14
  /*if((PORTD_ISFR & 0x4000))//���ж� (1<<14)
  {    
    PORTD_ISFR |= 0x4000;  //����жϱ�ʶ
    // �û�����            
    //DMA�ɼ����������޸Ĳɼ��ӿ�PTD_BYTE0_IN��D0--D7��   PLCK�ӵ���PTD13��   
    //DMA_PORTx2BUFF_Init (DMA_CH4, (void *)&PTD_BYTE0_IN,(void*)Image_Data[Line_Cont], PTD13, DMA_BYTE1, IMAGEW, DMA_rising);  
    DMATransDataStart(DMA_CH4,(uint32_t)(&Image_Data[0][0]));   
    if(Line_Cont > H)  //�ɼ�����
    { 
      Line_Cont=0; 
      return ;
    } 
    ++Line_Cont;            //�м���
    return ;
  }*/
  //���ж�PTD15
  if((PORTD_ISFR & 0x8000))//(1<<15)
  {
    PORTD_ISFR |= 0x8000; //����жϱ�ʶ   
    DMATransDataStart(DMA_CH4,(uint32_t)(&Image_Data[0][0]));// �û����� 
    Field_Over_Flag=1;     //��������ʶ

  } 
}
//�Ķ���ȡ�����У��Ķ��˵������Ķ���,�Ķ���ҡͷ���Ƶ��
/*******************************************************************************
�������ƣ�get_vline
��������: �õ���Ч��
��������
*******************************************************************************/
void get_vline()
{
  if(Shi_zi_num==0)
  {
	valid_line=120-Lost_Line_count;
        //uart_printf(test_port, "judge_vl:%d\n",judge_vl);
  }
  else 
  {
	valid_line=20;
  }
}
/*******************************************************************************
�������ƣ�stable_control
��������: ���ݴ������Ƴ���
��������
*******************************************************************************/
void stable_control()
{
  uint8 weight[]={1,1,1,1,1,1,1,1,1,1,
                1,1,1,1,1,1,1,1,1,1,
                1,1,1,2,2,2,2,3,3,3,
                3,3,3,3,3,3,3,3,3,3,
                3,3,3,2,2,2,2,2,2,2,
                1,1,1,1,1,1,1,1,1,1               
                }; 
  int weight_sum=0;

  //for(i=0;i<60;i++) weight_sum+=weight[i];
       uint8 line;
       getBlineCenter();//�õ����� ����Pick_table[line]��������
       int lost_count = 0;//
       int Bline_right_all = 0;
       int Bline_left_all = 0;
       //get_vline();//������û��ʮ�ֱ�־������Ч��
       //uart_printf(test_port,"%d\n",H - valid_line);
       /*{
	  positive_num=0;
	  decrease_num=0;
	  for(line=V-2;line>V-valid_line;line--)//V=188����Ӧ����H��Pick_table����¼�������룩
	  {
		if(Pick_table[line]>Pick_table[line+1])
		  positive_num++;
		else if(Pick_table[line]<Pick_table[line-1])
		  decrease_num++;
	  }
       }*/
       //uart_printf(test_port,"run_time:%d\n",run_time);
       if(run_time>=0)
       {
           run_time = 0;
           get_LR_diff();//û����
           judgeRoad();//��·��� ��Ӌ��
           //lost_count();
           int cursion_all = 0;
           int lost_count=0;
           
           for(line = H-1; line > 0; line--)//�������Ͽ����������4��Ϊ��Ч�У���ô��Ч�о���������Щ
           {
               if(Pick_flag[line] == ALL_LOST)
               {
                   lost_count = lost_count+1;
               }
               else
               {
                   lost_count = 0;
               }
               if(lost_count > 3)
               {
                   valid_line = H - line;
                   break;
               }
           }
           
           for(line = H-1; line >=H - valid_line; line--)
           {
                //uart_printf(test_port,"%d����ߣ�%d\n",H-line,Bline_left[line]);
                //uart_printf(test_port,"%d�ұ��ߣ�%d\n",H-line,Bline_right[line]);
                //uart_printf(test_port,"%d�����ߣ�%d\n",H-line,Pick_table[line]);
                //uart_printf(test_port,"%d���߲�ࣺ%d\n",H-line,Pick_table[line] - V/2);
                Bline_left_all += Bline_left[line];
                Bline_right_all += Bline_right[line];
                if(Pick_table[line] > 188 || Pick_table[line] < 0)
                {
                    Pick_table[line] = 94;
                }
                cursion_all = cursion_all + (Pick_table[line] - 94)*weight[line-60];//*����
                weight_sum+=weight[line-60];
                
                if((Pick_table[line] - Pick_table[line+1] > 30 || Pick_table[line+1] - Pick_table[line] > 30)&&(line < H-1))//�����һ������
                {
                    valid_line = H - line;//��Ч�о��Ǵ��������ҵ�������һ��
                    break;
                }
                
           }
           int change=0;
           int Bline_left_avg = Bline_left_all/valid_line;
           int Bline_right_avg = Bline_right_all/valid_line;
           int cursion_avg = cursion_all/weight_sum*1.1;
           
           
           /*if(Bline_left_avg == 0 && Bline_right_avg > 130 && valid_line>25){
              cursion_avg = -16;
              shizi_left = 10;
              change = 1;
           //uart_printf(test_port,"��ƽ��:%d\n",Bline_left_avg);
           //uart_printf(test_port,"��ƽ��:%d\n",Bline_right_avg);
           }
           if(Bline_right_avg == 188 && Bline_right_avg < 50 && valid_line>25){
              cursion_avg = 16;
              shizi_right = 10;
              change = 1;
           //uart_printf(test_port,"��ƽ��:%d\n",Bline_left_avg);
           //uart_printf(test_port,"��ƽ��:%d\n",Bline_right_avg);
           }*/
           /*if(shizi_left>0&&change == 0){
              cursion_avg = -8;
              shizi_left = shizi_left-1;
           }
           if(shizi_right>0&&change == 0){
              cursion_avg = 8;
              shizi_right = shizi_right-1;
           }*/ //ʮ�ֶ�ʱ������
           
           int Shizi_left = 0, Shizi_right = 0;
           int down = 0, up = 0;
           
           
           //�ж������Ҳ��Ƿ񿴲���������Ե
           if(Bline_left[H-1] == 0)
           {
               Shizi_left = 1;
               for(int shizi_i = H-1; shizi_i > H-10; shizi_i--)
               {
                   if(Bline_left[shizi_i] != 0 )
                   {
                      Shizi_left = 0;
                      //uart_printf(test_port,"111\n");
                   }
               }
           }
           
           if(Bline_right[H-1] == V-1)
           {
                Shizi_right = 1;
                for(int shizi_i = H-1; shizi_i > H-10; shizi_i--)
                {
                     if(Bline_right[shizi_i] != V-1 )//��V-1
                     {
                        Shizi_right = 0;
                        //uart_printf(test_port,"222\n");
                     }
                }
           }
           
           //��������ı�־��� �������࿴�������Ҳ࿴�õ�������Ե
           
           if(Shizi_left && !Shizi_right)//��಻�ɼ� �Ҳ�ɼ�
           {
                for(int shizi_i = H-1; ; shizi_i--)//���������Ͽ�ʼ��
                {
                   if(Bline_right[shizi_i] <= Bline_right[shizi_i+1] && shizi_i < H-1 && up == 0)//�Ҳ���� ���
                      down++;
                   
                   else if(Bline_right[shizi_i] >= Bline_right[shizi_i+1] && shizi_i < H-1)//�Ҳ�����ҹ�
                      up++;
                   
                   if(Bline_left[shizi_i] != 0)//��������������ʱ���˳�
                      break;
                }
           }
           
           else if(Shizi_right && !Shizi_left)//���ɼ� �Ҳ಻�ɼ�
           {
                for(int shizi_i = H-1; ; shizi_i--)
                {
                   if(Bline_left[shizi_i] >= Bline_left[shizi_i+1] && shizi_i < H-1 && up == 0)//������ �ҹ�
                      down++;
                   
                   else if(Bline_left[shizi_i] <= Bline_left[shizi_i+1] && shizi_i < H-1)//������ ���
                      up++;
                   
                   if(Bline_right[shizi_i] != 0)//�����Ҳ��������ʱ���Ƴ�
                      break;
                }
           }
           
           if(up >= 15 && down >= 15)//�е��Ժ���ʲô�����up|down����
           {
                 if(Shizi_left && !Shizi_right)//��಻�ɼ� �Ҳ�ɼ�ʱ
                 {
                    cursion_avg = -30;
                    uart_printf(test_port,"111\n");
                 }
                 else if(Shizi_right && !Shizi_left)
                 {
                    cursion_avg = 30;
                    uart_printf(test_port,"222\n");
                 }
           }
           
           mid_before = V/2 + cursion_avg;
           //cursion_avg = cursion_all/valid_line;
           
           if(cursion_avg <= -6)
           {
              //uart_printf(test_port,"111\n");
              direction_control(-cursion_avg);
           }
           else if(cursion_avg >= 6)//������Ĳ�ת��Χ��Ҫ�ǹ��ֶ���������������䣬�����
           {
              //uart_printf(test_port,"222\n");
              direction_control(-cursion_avg);
           }
           else//���ƫ���С
           {
              //uart_printf(test_port,"333\n");
              //uart_printf(test_port,"���ƽ��:%d\n",cursion_avg);
              duoji_control(dj_center,center_way);//�궨��dj_center885������λ��.�е�����870զ����?
           } 
           direction_control(-cursion_avg);//����error,����pid���ƣ�
           
           //uart_printf(test_port,"�����:%d\n",shizi_left);
           //uart_printf(test_port,"�ҳ���:%d\n",shizi_right);
           //uart_printf(test_port,"��ƽ��:%d\n",Bline_right_avg);
           //uart_printf(test_port,"��Ч��:%d\n",valid_line);
           //uart_printf(test_port,"��ƫ��:%d\n",cursion_all);
           //uart_printf(test_port,"���ƽ��:%d\n",cursion_avg);
           //uart_printf(test_port,"�Ƕ�:%d\n\n",jiaodu_num);
           
           last_turn=jiaodu_num;
           last_pick_way=pick_way;
           change = 0;
       }
       run_time++;
}


/*******************************************************************************
�������ƣ�stable_del
��������: ��ǰ�����ݴ������Ƴ���
��������
*******************************************************************************/
void stable_del()
{
  	uint8 line;
////////////////////////////////////////////////////////////////////////////////
	//buzzer_ctl();
	get_vline();//�õ���Ч��
        //uart_printf(test_port, "��Ч�У�%d\n",valid_line);
	//xielv_lvbo();//�����ֵ���Ʒ��˲� ȥ����Ч��
	//lvbo(5);
   	//ti_jiaozheng();
	bDistance();//�������ȷ��˲� ȥ����Ч��
   	/*for(line=0;line<H;line++)
	{
	  Pick_table[line]=(Bline_left[line]+Bline_right[line])/2;//
	}
	*/
	getBlineCenter();
	averageLvBo();
	//center_led();
	center_buxian();
	{
	  positive_num=0;
	  decrease_num=0;
	  for(line=V-2;line>V-valid_line;line--)
	  {
		if(Pick_table[line]>Pick_table[line+1])
		  positive_num++;
		else if(Pick_table[line]<Pick_table[line-1])
		  decrease_num++;
	  }
	}
	//put_get_hex_whole(Pick_table,Bline_left,Bline_right,H);  
	if(run_time>3)
	{
		//Is_out();
	    //Out_flag=0;
		/*if(Out_flag==1)
		{
		  //GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(17));/// ��Чͼ��ָʾ���� LED4
		  jiaodu_num=last_turn;
		  slow_down_num=1;
		  //find_shizi(valid_line);
		  way_control(); //�������duoji_controlҪ��
		  /*if(start_end_flag==1)//������������߱�־����ͣ��
			stop_car();
		  else*/
		  	//speed_ctl_stable();
		  stop_num++;
		  /*if(stop_num>5)
                    stop_car();
		  if(run_time>500)
				test_start_new();*/
		/*}
		else
		{*/
		    //GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(17));	//IO��������ߵ�ƽ LED4
			//find_s();
			//area=get_area(Pick_table[5],Pick_table[(valid_line)/2],Pick_table[valid_line],5,(valid_line)/2,valid_line);
			//lcd_int(8,"Xl:",(int)area);
                        //uart_printf(test_port, "1");
			get_LR_diff();
                        //uart_printf(test_port, "2");
			//If_straight();//�ж��Ƿ�Ϊֱ��
			judgeRoad();//�ж���������
                        //uart_printf(test_port, "3");
			//zhidao_count_flag=0;
			lost_count();//���㶪ʧ��
			if(valid_line>10)
			Position_diff=Pick_table[10]-last_position;
			last_position=Pick_table[10];
			whole_area=getWholeArea();
                        //uart_printf(test_port, "4");
			tx_area=getTxArea();
                        //uart_printf(test_port, "5");
			get_even_diff_s();
                        //uart_printf(test_port, "6");
			get_even_diff_near();
                        //uart_printf(test_port, "7");
			If_LStraight();
                        //uart_printf(test_port."zhidao_count_flag:%d\n",zhidao_count_flag);
                        //uart_printf(test_port, "8");
			//find_S_road();
                        //uart_printf(test_port, "9");
			//get_Curve_whole();
                        //uart_printf(test_port, "10");
			//get_Curve();
                        //uart_printf(test_port, "11");
			if(Near_lost==0)
			seg_pid_stable();
                        //(test_port, "12");
			way_control();
                        //uart_printf(test_port, "13");
			//servo_ctl();
			last_vline=valid_line;
                        run_time = 0;
/////////////////////////////////�ٶȿ���/////////////////////////////////
			/*if(start_end_flag==1)//������������߱�־����ͣ��
			{
			  	stop_car();
			}
			else*/
			//if(FRONT>1500&&Bmq<5)//��ײ�� ����ȥ��
			//{
				//ideal_Bmq=1;
			//}
			//else 
			//speed_ctl_stable();
			/*if(run_time>500)
				test_start_new();*/
		}
                run_time++;
	//}
}
/*******************************************************************************
�������ƣ�clearDelPar
��������: ���㴦������
��������
*******************************************************************************/
void clearDelPar()
{
 	startline_F=0;//������ʼ�б�־��0
 	//endline_F=0;//���ֽ����б�־��0
	//PickCenter_flag=0;//����ͼ��ʼѰ���߱�־
	lost_already=0;
	Lost_Line_count=0;
	Lost_left_count=0;
	Lost_right_count=0;
	Pick_line=0;
	Shi_zi_line=0;
	Bline_diff=0;
	maxBline_diff=0;
	Shi_zi_flag=0;
	lost_w_count=0;
	lost_b_count=0;
	near_xielv_left=0;
	near_xielv_right=0;
       
}

/***************************************************************
* 
* �������ƣ�SendPicture 
* ����˵��������ͼ����λ�� ����ͬ����λ��ע���޸Ķ�Ӧ�����ݽ���Э��
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2018��3��27�� 
* �� ע�� 
***************************************************************/ 
void UARTSendPicture(uint8_t tmImage[H][V]) 
{ 
  int i = 0, j = 0; 
  uint8_t threshhold = 40;
  uart_putchar(UART_0,0xAB); //����֡ͷ��־ DEMOK��λ��  
  uart_putchar(UART_0,0xAA); //����֡ͷ��־ DEMOK��λ��  
  uart_putchar(UART_0,0xAB); //����֡ͷ��־ DEMOK��λ��  
  uart_putchar(UART_0,0xAA); //����֡ͷ��־ DEMOK��λ��  
  uart_putchar(UART_0,0xAB); //����֡ͷ��־ DEMOK��λ��  
  int index_in_byte = 0;
  uint8_t image_byte = 0;
  uint8_t bit_map[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
  for(i=0;i < H; i++) 
  { 
    for(j=0;j <V;j++) 
    { 
      
      if (tmImage[i][j] > threshhold)
      {
        image_byte |= bit_map[index_in_byte];
      }
      else
      {
        image_byte &= (~bit_map[index_in_byte]);
      }
      index_in_byte++;
      if (index_in_byte > 7)
      {
        index_in_byte = 0;
        if (image_byte == 0xAA) 
        { 
          uart_putchar(UART_0,0xAB); 
          uart_putchar(UART_0,0xAA); 
          uart_putchar(UART_0,0xAA); 
          uart_putchar(UART_0,0xAB); 
        } 
        else
        {
          uart_putchar(UART_0,image_byte); 
        }
      }
    } 
  }
} 
/*************************************************************************
*                    �����������ܿƼ� 
*
*  �������ƣ�void SendPicture()
*  ����˵��������ͷ���ݷ���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺
*  ��    ע��
*************************************************************************/
/*void SendPicture(void)
{
  int i = 0, j = 0;
  uart_putchar(UART_0,0xff);//����֡ͷ��־
  for(i=0;i<IMAGEH;i++)      //���
  {
    for(j=0;j<IMAGEW;j++)    //����ӵ�0�е��У��û�����ѡ���Ե�������ʵ�����
    {
      if(Image_Data[i][j]==0xff)
      {
        Image_Data[i][j]=0xfe;//��ֹ���ͱ�־λ
      }
      uart_putchar(UART_0,Image_Data[i][j]);
    }
  }
}*/
void Test_Print_Road(void)
{
  for(int tmp_line = 0; tmp_line < H; tmp_line++){
                                  uart_printf(test_port, "������%d��:%d\n",tmp_line+1,Bline_left[tmp_line]);
                                  uart_printf(test_port, "������%d��:%d\n",tmp_line+1,Bline_right[tmp_line]);
                                  uart_printf(test_port, "������%d�У�%d\n",tmp_line+1,Pick_table[tmp_line]);
                                  //uart_printf(test_port, "��%d��:", tmp_line);
                                  //for(int tmp_row = 0; tmp_row < V; tmp_row++){
                                     //uart_printf(test_port, "%d ",(int)Image_Data[tmp_line][tmp_row]);
                                  //}
                                  //uart_printf(test_port,"\n");
    }
}
void Test_Print_Vline(void)
{
  uart_printf(test_port, "�����Ч�У�%d\n",valid_line);
}
void Test_Print_JudgeRoad(void)
{
  uart_printf(test_port, "�����Ч�У�%d\n",valid_line);
  uart_printf(test_port, "СS��־��%d\n",ls_flag);
  uart_printf(test_port, "���������%d\n",pick_way);
  uart_printf(test_port, "ֱ���жϱ�־��%d\n",zhidao_count_flag);
}



void SendPicture(void)
{
  
  uart_putchar(test_port, 0x01);
  uart_putchar(test_port, 0xFE);
  for (int i = 0; i < IMAGEH; i++) 
  { 
    for (int j = 0; j < IMAGEW; j++) 
    { 
      if(Image_Data[i][j]>Cmp)
      {
        Image_Data[i][j] = 0xFF;
        uart_putchar(test_port,0xFF);
      }
      else
      {
        Image_Data[i][j] = 0x00;
        uart_putchar(test_port,0x00);
      }
    } 
  } 
  
  int i = 0, j = 0;

  //uart_putchar(UART_0,0xff);//����֡ͷ��־
  for(i=0;i<IMAGEH;i++)      //���
  {
    for(j=0;j<IMAGEW;j++)    //����ӵ�0�е��У��û�����ѡ���Ե�������ʵ�����
    {
      //uart_putchar(test_port,Image_Data[i][j]);
    }
  }
  uart_putchar (test_port, 0xFE);
  uart_putchar (test_port, 0x01);
}

  
void SendPicture_char(void)
{
  int i = 0, j = 0;
  for(i=0;i<IMAGEH;i++)      //���
  {
    for(j=0;j<IMAGEW;j++)    //����ӵ�0�е��У��û�����ѡ���Ե�������ʵ�����
    {
      if(Image_Data[i][j]>Cmp)
        uart_printf(test_port,"1 ");
      else
        uart_printf(test_port,"0 ");
    }
      uart_printf(test_port,"\n");
  }
   uart_printf(test_port,"\n\n\n\n");
}



#define TRIG PTE2 
#define ECHO PTE3


int rstatus = 0;
int lstatus = 0;
int havestop = 0;
int incross = 0;
int cnt_gzp = 0;

int Mmin(int a,int b)
{
  return a<b?a:b;
}



void imagineProcess(void)
{
 

  //setbinary();
    
  while(1)
  {
    int i = H-11;
    int j = 0;
    
       if(Field_Over_Flag)    //���һ��ͼ��ɼ�����ʾ���������ݵ���λ��
      {
          //exti_disable(PTD14); //���жϹر�
          exti_disable(PTD15); //���жϹر� 
          
          clearDelPar();//���㴦������
         //SendPicture();
          find_edge();//
          //valid_line=GetValidLine();
         //uart_printf(UART_0,"line =  %d\n",valid_line);
         /*for(;i>H-41;i--)
          {
            uart_printf(UART_0,"%d %d %d \n",Bline_left[i],Pick_table[i],Bline_right[i]);
            delayms(50);
          }
         uart_printf(UART_0,"\n\n");*/
          ele_direction_control(); 
          cnt_gzp += 1;
          
          
          /*for(j = 0;j<V-1;j++)
          {
            if(Image_Data[H-41][j]>Cmp && Image_Data[H-41][j+1]<Cmp)
            {
              jumppoint++;
            }
          }
          
          if(jumppoint)
          {
            if(jumppoint>=4 && !havestop)
            {
              setpoint1 = 0;
              setpoint2 = 0;
              delayms(2000);
              havestop = 1;
            }
          jumppoint = 0;
          }*/
          
            /*for(i = H-11;i>H-41;i--)
            {
              if(Bline_right[i-2]>180 && Bline_right[i-1]>180 && Bline_right[i]<170 && Bline_right[i+1]<170
                 && abs(Bline_right[i-2]-Bline_right[i-1])<5 && abs(Bline_right[i-1]-Bline_right[i])<5
                    && abs(Bline_right[i]-Bline_right[i+1])<5)
              {
                PWMSetSteer(990);
                delayms(1300);
                readyright = 1;
                setpoint1 = setpoint2 = 0;
              }
            }*/
           int lostRight = 0;
           int haveRight = 0;
           int rightVal = 0;
            for(i = H-31;i>H-41;i--)
            {
              rightVal += Bline_right[i];
            }
            rightVal /= 10;
            if(rightVal > 180)
            {
              lostRight = 1;
            }
            rightVal = 0;
            for(i = H-16;i>H-26;i--)
            {
             rightVal += Bline_right[i]; 
            }
            rightVal /= 10;
            if(rightVal < 170)
            {
              haveRight = 1;
            }
            rightVal = 0;
              if(lostRight && haveRight)
              {
                lostRight = 0;
                haveRight = 0;
                int p1 = 0;
                int p2 = 0;
                int p3 = 0;
                for(int j = H-11;j>H-14;j--)
                {
                  if (Bline_left[j]>20)
                  {
                     p1 = Bline_left[j];
                     break;
                  }
                  
                } 
                for(int j = H-19;j>H-22;j--)
                {
                  if (Bline_left[j]>25)
                  {
                     p2 = Bline_left[j];
                     break;
                  }
                  
                } 
                for(int j = H-26;j>H-30;j--)
                {
                  if (Bline_left[j]>30)
                  {
                     p3 = Bline_left[j];
                     break;
                  }
                  
                }
                if(p1 && p2 && p3 && abs(p2-p1)<10 && abs(p3-p2)<10 && abs(p3-p1)<20)
                {
                  int l = 0;
                  for(int j = H-11;j>H-41;j--)
                  {
                    if(Bline_left[j]>25)
                    {
                      l++;
                    }
                  }
                  if(l>20)
                  {
                     rstatus = 1;
                  }
                }            
              }
              if(rstatus == 1)
              {
                int haveRight = 0;
                int rightVal = 0;
                for(i = H-35;i>H-45;i--)
                {
                  rightVal += Bline_right[i];
                }
                rightVal /= 10;
                if(rightVal < 150)
                {
                  haveRight = 1;
                }
                /*rightVal = 0;
                for(i = H-5;i>H-15;i--)
                {
                  rightVal += Bline_right[i];
                }
                rightVal /= 10;
                if(rightVal < 180 && haveRight == 1)
                {
                  haveRight = 2;
                }*/
                rightVal = 0;
                if(haveRight)
                {
                  rstatus = 2;
                  /*setpoint1 = setpoint2 = 0;
                  delayms(1000);
                  setpoint1 = setpoint2 = 300;*/
                }
                
              }
              if(rstatus == 2)
              {
                int lostRight = 0;
                int rightVal = 0;
                for(i = H-25;i>H-35;i--)
                {
                  rightVal += Bline_right[i];
                }
                rightVal /= 10;
                if(rightVal > 180)
                {
                  lostRight = 1;
                }
                rightVal = 0;
                if(lostRight)
                {
                  rstatus = 3;
                }
                
              }
              if(rstatus==3)
              {
                delayms(200);
                PWMSetSteer(850);
                delayms(600);
                cnt_gzp = 0;
                rstatus = 4;
              }
              
              if(rstatus==4 && cnt_gzp>100)
              {
                int lostLeft = 0;
                int leftVal = 0;
                for(i = H-15;i>H-25;i--)
                {
                  leftVal += Bline_left[i];
                }
                leftVal /= 10;
                if(leftVal < 10)
                {
                  lostLeft = 1;
                }
                leftVal = 0;
                if(lostLeft)
                {
                  lostLeft = 0;
                  PWMSetSteer(850);
                  delayms(800);
                  rstatus = 0;
                }
              }
              
              
              
           int lostLeft = 0;
           int haveLeft = 0;
           int leftVal = 0;
            for(i = H-31;i>H-41;i--)
            {
              leftVal += Bline_left[i];
            }
            leftVal /= 10;
            if(leftVal < 10)
            {
              lostLeft = 1;
            }
            leftVal = 0;
            for(i = H-16;i>H-26;i--)
            {
             leftVal += Bline_left[i]; 
            }
            leftVal /= 10;
            if(leftVal > 20)
            {
              haveLeft = 1;
            }
            leftVal = 0;
              if(lostLeft && haveLeft)
              {
                lostLeft = 0;
                haveLeft = 0;
                int p1 = 0;
                int p2 = 0;
                int p3 = 0;
                for(int j = H-11;j>H-14;j--)
                {
                  if (Bline_right[j]<168)
                  {
                     p1 = Bline_right[j];
                     break;
                  }
                  
                } 
                for(int j = H-19;j>H-22;j--)
                {
                  if (Bline_right[j]<165)
                  {
                     p2 = Bline_right[j];
                     break;
                  }
                  
                } 
                for(int j = H-26;j>H-30;j--)
                {
                  if (Bline_right[j]<160)
                  {
                     p3 = Bline_right[j];
                     break;
                  }
                  
                }
                if(p1 && p2 && p3 && abs(p2-p1)<10 && abs(p3-p2)<10 && abs(p3-p1)<20)
                {
                  int r = 0;
                  for(int j = H-11;j>H-41;j--)
                  {
                    if(Bline_right[j]<165)
                    {
                      r++;
                    }
                  }
                  if(r>20)
                  {
                     lstatus = 1;
                  }
                }            
              }
              if(lstatus == 1)
              {
                int haveLeft = 0;
                int leftVal = 0;
                for(i = H-35;i>H-45;i--)
                {
                  leftVal += Bline_left[i];
                }
                leftVal /= 10;
                if(leftVal > 30)
                {
                  haveLeft = 1;
                }
                /*rightVal = 0;
                for(i = H-5;i>H-15;i--)
                {
                  rightVal += Bline_right[i];
                }
                rightVal /= 10;
                if(rightVal < 180 && haveRight == 1)
                {
                  haveRight = 2;
                }*/
                leftVal = 0;
                if(haveLeft)
                {
                  lstatus = 2;
                  /*setpoint1 = setpoint2 = 0;
                  delayms(1000);
                  setpoint1 = setpoint2 = 300;*/
                }
                
              }
              if(lstatus == 2)
              {
                int lostLeft = 0;
                int leftVal = 0;
                for(i = H-25;i>H-35;i--)
                {
                  leftVal += Bline_left[i];
                }
                leftVal /= 10;
                if(leftVal < 10)
                {
                  lostLeft = 1;
                }
                leftVal = 0;
                if(lostLeft)
                {
                  lstatus = 3;
                }
                
              }
              if(lstatus==3)
              {
                delayms(200);
                PWMSetSteer(1150);
                delayms(600);
                lstatus = 4;
                cnt_gzp = 0;
              }
              
              if(lstatus==4 && cnt_gzp>100)
              {
                int lostRight = 0;
                int rightVal = 0;
                for(i = H-15;i>H-25;i--)
                {
                  rightVal += Bline_right[i];
                }
                rightVal /= 10;
                if(rightVal > 180)
                {
                  lostRight = 1;
                }
                rightVal = 0;
                if(lostRight)
                {
                  lostRight = 0;
                  PWMSetSteer(1150);
                  delayms(800);
                  lstatus = 0;
                }
              }
            
            
             /*for(i = H-11;i>H-41;i--)
            {
              if(Bline_left[i-2]<5 && Bline_left[i-1]<5 && Bline_left[i]>25 && Bline_left[i+1]>25
                 && abs(Bline_right[i-2]-Bline_right[i-1])<2 && abs(Bline_right[i-1]-Bline_right[i])<2
                    && abs(Bline_right[i]-Bline_right[i+1])<2)
              {
                setpoint1 = setpoint2 = 0;
              }
            }*/
            
      
          //uart_printf(UART_0,"right = %d\n",readyright);
  

          exti_enable(PTD15,IRQ_FALLING|PULLUP);    //���ж� 
          Field_Over_Flag= 0; 
   
     }
  

  }
}



//����������
void Test_LQV034(void)
{  
  
  //LCD_Show_Frame94();      //��ͼ�� LCDW*LCDH ���
  //uart_printf(test_port, "�����Ч��\n");
  LQMT9V034_Init();        //����ͷ��ʼ��
  //uart_printf(test_port, "�����Ч��2\n");
  int line = 0;
  /*while(1){
    if(Field_Over_Flag){
       //���ڷ������ݷǳ�����ע�͵�OLEDˢ�ºܿ�
      exti_disable(PTD14); //���жϹر�
      exti_disable(PTD15); //���жϹر�  
      UARTSendPicture(Image_Data);     //�������ݵ���λ����ע��Э���ʽ����ͬ����λ����ԭ������Ӧ�޸ģ�//��ʹ��ʱ��ر�
      exti_enable(PTD14,IRQ_RISING|PULLDOWN);   //���ж�
      exti_enable(PTD15,IRQ_FALLING|PULLUP);    //���ж� 
      
      Get_Use_Image();     //�ɼ�ͼ�����ݴ������
      Get_01_Value();      //��ֵ��ͼ������
                  
      //Threshold = GetOSTU(Image_Data);   //OSTU��� ��ȡȫ����ֵ
      //BinaryImage(Image_Data,Threshold); //��ֵ��ͼ������
      //Pixle_Filter();
      //TFTSPI_Show_Pic3(18, 8, 94, 60, Pixle); //����TFT1.8��ʾ��ֵ��ͼ��
      //Draw_Road();                         //����OLEDģ����ʾ��̬ͼ��
      Field_Over_Flag= 0;
    }
  }*/
  while(1)
  { 
    //LED_Ctrl(LED1, RVS);   //LEDָʾ��������״̬
    if(Field_Over_Flag)    //���һ��ͼ��ɼ�����ʾ���������ݵ���λ��
    {
      //���ڷ������ݷǳ�����ע�͵�OLEDˢ�ºܿ�
      exti_disable(PTD14); //���жϹر�
      exti_disable(PTD15); //���жϹر�  
      //uart_printf(test_port, "111�����Ч�У�%d\n",valid_line);
      //SendPicture();
      //UARTSendPicture(Image_Data);     //�������ݵ���λ����ע��Э���ʽ����ͬ����λ����ԭ������Ӧ�޸ģ�//��ʹ��ʱ��ر�
      //LED_Ctrl(LED1, RVS);       //LEDָʾ��������״̬
      
    	  ///////////////////////////////ͼ��ɼ�����///////////////////////////////
                        //uart_printf(test_port, "�����Ч�У�%d\n",valid_line);
      
                        clearDelPar();//���㴦������
                          //Binary_line(line);//��ֵ���òɼ���
                        
                        find_edge();
                        valid_line=GetValidLine();
                         
                         
			
                        
                        
                        
                        
			for(line=H-1;line>=0;line--) //��ȡ�������ĵ㲢��������
			{
				row_F[line]=0;//����ɼ���ɱ�־λ
				
				if(line==H-4)
				{
					//PickCenter_near(); 
				  	Near_lost=PickCenter_near_advance();
                                        //uart_printf(test_port, "�����Ч�У�%d\n",valid_line);
				  	if(Near_lost==1)//���ֻ���ֶ���һ��
					{
					  Far_find_flag=FAR_FIND_MIDDLE;
					  Shi_zi_line=4;
					}
				}
                              
				if(line<=H-5)//��������������
				{
                                          PickCenter_diff_advance(line);
                                          if(lost_w_count>2)
                                          {
                                                
                                          }
				  
				}
////////////////////////////////ʮ���б�ʽ///////////////////////////////////////
				if((Pick_flag[line]&ALL_LOST_W))//���߶��Ҳ�������Ѱ��Ϊȫ������ͳ��
				{
                                        lost_w_count++;
                                        Shi_zi_flag=1;
                                      //buzzer_on();
				}
				if(Shi_zi_flag==0&&lost_already==0)//����Ч�ж�ʧ֮ǰ�ж϶��������ж�ʮ�ֵ�
				{
                                        if(lost_w_count>5)
                                        { 
                                           Shi_zi_flag=1;
					  //buzzer_on();
                                         }
				}
    
                                //uart_printf(test_port, "�����Ч�У�%d\n",valid_line);

///////////////////////////��Ч���ж�///////////////////////////////////////������B��β�ı�־λ������ı�ģ�����
                                
				if((Pick_flag[line]&LEFT_LOST_B)||(Pick_flag[line]&RIGHT_LOST_B)||((Bline_left[line]-Bline_right[line]<5)&&(Bline_left[line]-Bline_right[line]>-5)))
				{
				  if((Pick_flag[line-1]&LEFT_LOST_B)||(Pick_flag[line-1]&RIGHT_LOST_B)||((Bline_left[line]-Bline_right[line]<5)&&(Bline_left[line]-Bline_right[line]>-5)))
					lost_b_count++;
				}
                                //Bline_diff��������֮��Ĳ��
				if(line>H-25)
				{
				  if(maxBline_diff<Bline_diff)
				  	maxBline_diff=Bline_diff;
				}
                                //uart_printf(test_port, "�����Ч�У�%d\n",valid_line);
				if(lost_already==0)
				{
					if(lost_b_count>3||(Bline_left[line]<5)||(Bline_right[line]>(V-5))||((Bline_left[line]-Bline_right[line]<5)&&(Bline_left[line]-Bline_right[line]>-5)))
					{
                                                //uart_printf(test_port, "����%d�У�%d\n",line,Bline_right[line]);
                                                lost_already=1;
                                                judge_vl=H-line;
                                                lost_b_count = 0;
                                                if(judge_vl < 20)
                                                {
                                                    lost_already = 0;
                                                    lost_b_count = 0;
                                                }
					}
				}
			 }
                        //Test_Print_Road();
			//TEST_IO_L;
	  //////////////////////////////ͼ���������Ʋ���//////////////////////////
			 //uart_printf(test_port, "�����Ч�У�%d\n",valid_line);
                         valid_line=IMAGEH-Lost_Line_count;
          
                         stable_control();
    
                         Lost_Line_count = 0;
      //uart_printf(test_port, "111�����Ч�У�%d\n",valid_line);
      exti_enable(PTD14,IRQ_RISING|PULLDOWN);   //���ж�
      exti_enable(PTD15,IRQ_FALLING|PULLUP);    //���ж� 
      
      //Get_Use_Image();     //�ɼ�ͼ�����ݴ������
      //Get_01_Value();      //��ֵ��ͼ������     
      //Threshold = GetOSTU(Image_Data);   //OSTU��� ��ȡȫ����ֵ
      //BinaryImage(Image_Data,Threshold); //��ֵ��ͼ������
      //Pixle_Filter();
      //TFTSPI_Show_Pic3(18, 8, 94, 60, Pixle); //����TFT1.8��ʾ��ֵ��ͼ��
      //Draw_Road();                         //����OLEDģ����ʾ��̬ͼ��
      Field_Over_Flag= 0;       
    }    
  }
}

// MT9V034 Port Init
void LQMT9V034_Init(void)
{     
  uint16_t data = 0;  
  
  //����ͷ�Ĵ�������
  //TFTSPI_CLS(u16BLUE);
  SCCB_Init();                     //������ַ�ڶ����ߣ�ΪMT9V034_I2C_ADDR  
  if(SCCB_RegRead(MT9V034_I2C_ADDR>>1,MT9V034_CHIP_VERSION,&data) == 0)//��ȡ����ͷ�汾�Ĵ��� 
  {     
    if(data != MT9V034_CHIP_ID)                                  //оƬID����ȷ��˵��û����ȷ��ȡ�����ݣ��ȴ�      
    { 
      //TFTSPI_P8X8Str(0,0,(u8*)"034 dpi!!", u16RED, u16BLUE);                       //����ͷʶ��ʧ�ܣ�ֹͣ����
      //uart_printf(UART_0, "error,id = %d!\n",data); 
      while(1)
      {
        uart_printf(test_port, "һ\n");
      } 
    } 
    else                                                   //оƬID��ȷ
    {
      //TFTSPI_P8X8Str(0,0,(u8*)"V034 OK", u16RED, u16BLUE);
      //uart_printf(UART_0, "correct!\n"); 
    }
  } 
  else 
  { 
    while(1){
      uart_printf(test_port, "��\n"); 
      } ; //����ͷʶ��ʧ�ܣ�ֹͣ����
  }  

  MT9V034_SetFrameResolution(IMAGEH, IMAGEW);//��������ͷͼ��4*4��Ƶ���PCLK, 27/4 = 6.75M ,BIT4,5��������:�������Ҿ�����   
  MT9V034_SetAutoExposure(1);  
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0xAC, 0x0001);
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0xAD, 0x01E0);  
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x2C, 0x0004);
  
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x7F, 0x3000);           // test pattern

  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ANALOG_CTRL, MT9V034_ANTI_ECLIPSE_ENABLE);
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x0F, 0x0000);  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_HDR_ENABLE_REG, 0x0101); // 0x0F bit8:1HDR,0linear; bit1:1��ɫ,0�Ҷ�;bit0:1HDR,0linear
  //MT9V034_WriteReg16(MT9V034_HDR_ENABLE_REG, 0x0103);     // 0x0F bit8:1HDR,0linear; bit1:1��ɫ,0�Ҷ�;bit0:1HDR,0linear
  //0x07 Chip Control bit2-0:0����ɨ��,1��Ч��2����3������bit5:1�����Ӿ�ʹ��,bit7:1����ʹ�ܣ�bit8:1ͬ��ģʽ;bit9:1����������bit15:0A/1B
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_CHIP_CONTROL, 0x0188);          //Context A  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ROW_NOISE_CORR_CTRL_REG, 0);   //0x70  0x0000 
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE_REG, 0x0303);   //0xAF  AEC/AGC A~bit0:1AE;bit1:1AG/B~bit2:1AE;bit3:1AG
  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MIN_EXPOSURE_REG, 0x0380);     //0xAC  ��С�ֿ��ſ���   0x0001
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_EXPOSURE_REG, 0x0480);     //0xAD  ���׿��ſ���   0x01E0-480
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_GAIN_REG, 60);             //0xAB  ���ģ������     64
  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AGC_AEC_PIXEL_COUNT_REG, 188*120);//0xB0  ����AEC/AGCֱ��ͼ������Ŀ,���44000   4096=320*240  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ADC_RES_CTRL_REG, 0x0303);     //0x1C  here is the way to regulate darkness :)    
  
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x13,0x2D2E);//We also recommended using R0x13 = 0x2D2E with this setting for better column FPN.  
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x20,0x03C7);//0x01C7�ԱȶȲ���ף�0x03C7�Աȶ���� Recommended by design to improve performance in HDR mode and when frame rate is low.
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x24,0x0010);//Corrects pixel negative dark offset when global reset in R0x20[9] is enabled.
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x2B,0x0003);//Improves column FPN.
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x2F,0x0003);//Improves FPN at near-saturation.  
  
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x08, 0x03D4);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x09, 0x03E7);
  //100DB //�����ع�ʱ��
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_SHUTTER_WIDTH1,0x01BB);        //0x08 Coarse Shutter IMAGEW 1
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_SHUTTER_WIDTH2,0x01D9);        //0x09 Coarse Shutter IMAGEW 2
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_SHUTTER_WIDTH_CONTROL,0x0064); //0x0A Coarse Shutter IMAGEW Control 
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_V2_CTRL_REG_A, 0x001A);        //0x32   0x001A  
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_TOTAL_SHUTTER_WIDTH,0x0100);   //0x0B Coarse Shutter IMAGEW Total
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_HDR_ENABLE_REG,0x0103);        //0x0F High Dynamic Range enable,bit is set (R0x0F[1]=1), the sensor uses black level correction values from one green plane, which are applied to all colors.
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AGC_AEC_DESIRED_BIN_REG, 64); //0xA5  ͼ������  50  1-64
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_ANALOG_GAIN,0x8010);           //0x35
  
  //80dB HDR
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x08, 0x03CA);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x09, 0x03DE);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x0A, 0x0064);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x0B, 0x03E8);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x0F, 0x0103);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x35, 0x8010);   

  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_RESET, 0x03);          //0x0c  ��λ 
  
  //GPIO�ڳ�ʼ��
  exti_enable(PTD14,IRQ_RISING|PULLDOWN);   //���ж�
  exti_enable(PTD15,IRQ_FALLING|PULLUP);    //���ж� 
  gpio_init(PTD0,GPI,0);          //��λ���������      
  gpio_init(PTD1,GPI,0);  
  gpio_init(PTD2,GPI,0);
  gpio_init(PTD3,GPI,0);
  gpio_init(PTD4,GPI,0);
  gpio_init(PTD5,GPI,0);
  gpio_init(PTD6,GPI,0);
  gpio_init(PTD7,GPI,0);
  //��ʼ��DMA�ɼ�  
  dma_portx2buff_init(DMA_CH4, (void *)&PTD_BYTE0_IN,(void*)Image_Data, PTD13, DMA_BYTE1, IMAGEW*IMAGEH, DMA_RISING);  
}     

void setbinary()
{
    
    if(Field_Over_Flag)    //���һ��ͼ��ɼ�����ʾ���������ݵ���λ��
      {
          exti_disable(PTD14); //���жϹر�
          exti_disable(PTD15); //���жϹر�  
          
          uint8_t binary=GetOSTU(Image_Data);
          Cmp=binary;
    
          exti_enable(PTD14,IRQ_RISING|PULLDOWN);   //���ж�
          exti_enable(PTD15,IRQ_FALLING|PULLUP);    //���ж� 
          Field_Over_Flag= 0;   
          
      }
}



void MT9V034_SetFrameResolution(uint16_t height,uint16_t width)
{
  uint16_t data = 0;
  
  if((width*4)<=MAX_IMAGE_WIDTH && (height*4)<=MAX_IMAGE_HEIGHT)
  {
    width *= 4;
    height *= 4;
    data |= MT9V034_READ_MODE_ROW_BIN_4;
    data |= MT9V034_READ_MODE_COL_BIN_4;
  }
  else if((width*2)<=MAX_IMAGE_WIDTH && (height*2)<=MAX_IMAGE_HEIGHT)
  {
    width *= 2;
    height *= 2;
    data |= MT9V034_READ_MODE_ROW_BIN_2;
    data |= MT9V034_READ_MODE_COL_BIN_2;
  }
  
  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x01,MT9V034_COLUMN_START_DEF);     // Column Start
  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x02,MT9V034_ROW_START_DEF);        // Row Start  
  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x03,MT9V034_WINDOW_HEIGHT_DEF);    // height 
  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x04,MT9V034_WINDOW_WIDTH_DEF);     // width  
  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x05,MT9V034_HORIZONTAL_BLANKING_MIN);   // Horizontal Blanking  809-640
  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x06,MT9V034_VERTICAL_BLANKING_MIN);     // Vertical Blanking    499-480 
  
  //����ͼ�������СΪ120*188������ʧ��Ұ��ͬʱ�������ܳ���Ҫ
  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x0D,0x030A);   //��������ͷͼ��4*4��Ƶ���PCLK, 27/4 = 6.75M ,BIT4,5�������� 
  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x0D,0x033A);   //��������ͷͼ��4*4��Ƶ���PCLK, 27/4 = 6.75M ,BIT4,5��������:�������Ҿ����� 
  
  data |= (MT9V034_READ_MODE_ROW_FLIP|MT9V034_READ_MODE_COLUMN_FLIP);       //LQ-MT9V034 needs vertical mirror to capture correct image
  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_READ_MODE, data);
  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_WINDOW_WIDTH,  width);
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_WINDOW_HEIGHT, height);
  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_COLUMN_START, (MAX_IMAGE_WIDTH-width)/2+MT9V034_COLUMN_START_MIN);
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ROW_START, (MAX_IMAGE_HEIGHT-height)/2+MT9V034_ROW_START_MIN);
}

void MT9V034_SetAutoExposure(char enable)
{
  uint16_t reg =0;
  SCCB_RegRead(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE,&reg);
  if(1 == enable)
  {
    SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE, reg|MT9V034_AEC_ENABLE|MT9V034_AGC_ENABLE);
  }
  else
  {
    SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE, reg&~(MT9V034_AEC_ENABLE|MT9V034_AGC_ENABLE));
  }
}
void MT9V034_SetFrameRate(uint8_t frameRate)
{
  
}

void MT9V034_Reset(void)
{
  //Reset MT9V034, but register config will not change.
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_RESET, 0x0001);
  time_delay_ms(10);
  
  //Unlock MT9V034, allowing user to initiate register settings and readout
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_CHIP_CONTROL, 0x0188);
  
  //Reset Again.
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_RESET, 0x0001);
  time_delay_ms(10);
}

void MT9V034_SetReservedReg(void)
{
  //Here we write some reserved registers as recommendations from Rev.G datasheet, Table.8
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x13, 0x2D2E);
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x20, 0x03C7);
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x24, 0x001B);
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x2B, 0x0003);
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x2F, 0x0003);
}


// ��ȡ��Ҫ��ͼ������
__ramfunc void Get_Use_Image(void)
{
  int i = 0,j = 0,row = 0,line = 0;
  
  for(i = 0; i  < IMAGEH; i+=2)  //120�У�ÿ2�вɼ�һ�У�
  {
    for(j = 0;j < IMAGEW; j+=2)  //188��
    {        
      Image_Use[row][line] = Image_Data[i][j];         
      line++;        
    }      
    line = 0;
    row++;      
  }  
}

//���վ�ֵ�ı������ж�ֵ��
void Get_01_Value(void)
{
  int i = 0,j = 0;
  u8 GaveValue;
  u32 tv=0;
  char txt[16];
  
  //�ۼ�
  for(i = 0; i <LCDH; i++)
  {    
    for(j = 0; j <LCDW; j++)
    {                            
      tv+=Image_Use[i][j];   //�ۼ�  
    } 
  }
  GaveValue=tv/LCDH/LCDW;     //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100 
  sprintf(txt,"%03d:%03d",Threshold,GaveValue);//ǰ��Ϊ�����õ���ֵ������Ϊƽ��ֵ  
  //TFTSPI_P8X8Str(50,0,(u8*)txt, u16RED, u16BLUE);
  //���վ�ֵ�ı������ж�ֵ��
  GaveValue=GaveValue*7/10+10;        //�˴���ֵ���ã����ݻ����Ĺ������趨 
  for(i = 0; i < LCDH; i++)
  {
    for(j = 0; j < LCDW; j++)
    {                                
      //if(Image_Use[i][j] >GaveValue)//ƽ��ֵ��ֵ
      if(Image_Use[i][j] >Threshold) //�����ֵ   ��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����    
        Pixle[i][j] =1;        
      else                                        
        Pixle[i][j] =0;
    }    
  }
}
//�������Ϸ���Χ��������
void Pixle_Filter(void)
{  
  int nr; //��
  int nc; //��
  
  for(nr=1; nr<LCDH-1; nr++)
  {  	    
    for(nc=1; nc<LCDW-1; nc=nc+1)
    {
      if((Pixle[nr][nc]==0)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]>2))         
      {
        Pixle[nr][nc]=1;
      }	
      else if((Pixle[nr][nc]==1)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]<2))         
      {
        Pixle[nr][nc]=0;
      }	
    }	  
  }  
}

/***************************************************************************
*                                                                          *
*  �������ƣ�int Seek_Road(void)                                           *
*  ����˵����Ѱ�Ұ�ɫ����ƫ��ֵ                                            *
*  ����˵������                                                            *
*  �������أ�ֵ�Ĵ�С                                                      *
*  �޸�ʱ�䣺2017-07-16                                                    *
*  ��    ע�����м�Ϊ0������һ���Ҳ��һ����ֵ����1�����                *
*            ��������ӵ�һ�п�ʼ�������ڶ��н�����                        *
*            ������Ϊ��������ֵԽ��˵��Խƫ��ߣ�                        *
*            ������Ϊ��������ֵԽ��˵��Խƫ�ұߡ�                        *
***************************************************************************/ 
void Seek_Road(void)
{  
  int nr; //��
  int nc; //��
  int temp=0;//��ʱ��ֵ
  //for(nr=1; nr<MAX_ROW-1; nr++)
  temp=0;
  for(nr=8; nr<24; nr++)
  {  	    
    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        ++temp;
      }			   
    }
    for(nc=0; nc<MAX_COL/2; nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        --temp;
      }			   
    }		  
  }
  OFFSET0=temp;
  temp=0;
  for(nr=24; nr<40; nr++)
  {  	    
    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        ++temp;
      }			   
    }
    for(nc=0; nc<MAX_COL/2; nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        --temp;
      }			   
    }		  
  }
  OFFSET1=temp;    	
  temp=0;
  for(nr=40; nr<56; nr++)
  {  	    
    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        ++temp;
      }			   
    }
    for(nc=0; nc<MAX_COL/2; nc=nc+1)
    {
      if(Pixle[nr][nc])
      {
        --temp;
      }			   
    }		  
  }
  OFFSET2=temp;   	
  return;  
}

u8 zb[48],yb[48];
void FindTiXing(void)
{
  int nr; //��
  int nc; //��     
  
  for(nr=0; nr<48; nr++)
  {  	    
    zb[nr]=0;
    yb[nr]=100;   
  }  	
  for(nr=0; nr<48; nr++)
  {  	    
    for(nc=2;nc<MAX_COL-2;nc++)
    {
      if((Pixle[nr+8][nc-1]==0)&&(Pixle[nr+8][nc]==0)&&(Pixle[nr+8][nc+1]==1)&&(Pixle[nr+8][nc+2]==1))
      {
        zb[nr]=nc;//����أ�Խ��Խ��
      }
      if((Pixle[nr+8][nc-1]==1)&&(Pixle[nr+8][nc]==1)&&(Pixle[nr+8][nc+1]==0)&&(Pixle[nr+8][nc+2]==0))
      {
        yb[nr]=nc;//�ұ��أ�Խ��ԽС
      }                   
    }	    
  }
  TXV=0;
  for(nr=0; nr<47; nr++)
  {  	    
    if((zb[nr]>=zb[nr+1])&&(zb[nr]>0))   TXV++;          
    if((yb[nr]<=yb[nr+1])&&(yb[nr]<100)) TXV--;          
  }  	   
  return;  
}

/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�void SCCB_Init(void)
*  ����˵��������SCCB��������ΪGPIO���ܣ���ʱ���������ݷ���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
void SCCB_Init(void)
{
  gpio_init(PTB2,GPO,0);//����ΪGPIO����
  gpio_init(PTB3,GPO,0);//����ΪGPIO����  
}

/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�void SCCB_Wait(void)
*  ����˵����SCCB�ȴ���ʾ
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
void SCCB_Wait(void)
{
  uint8 i=0;
  for(i=0;i<100;i++)
  { 
    asm ("nop");
  }  
}

/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�void SCCB_Star(void)
*  ����˵������������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
void SCCB_Star(void)
{
  SCL_OUT();
  SDA_OUT();
  SCCB_Wait();
  SDA_High;
  SCL_High; 
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_Low; 
  SCCB_Wait();
}
/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�void SCCB_Stop(void)
*  ����˵����ֹͣ����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
void SCCB_Stop(void)
{
  SCL_OUT();
  SDA_OUT();
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_High; 
  SCCB_Wait();
  SDA_High;
  SCCB_Wait();  
}
/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�uint8 SCCB_SendByte(uint8 Data)
*  ����˵����SCCB�����ֽں���
*  ����˵����Ҫ���͵��ֽ�
*  �������أ�Ӧ���ź�
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
uint8 SCCB_SendByte(uint8 Data)
{
  uint8 i;
  uint8 Ack;
  SDA_OUT();
  for( i=0; i<8; i++)
  {
    if(Data & 0x80) SDA_High;
    else            SDA_Low;    
    Data <<= 1;
    SCCB_Wait();
    SCL_High;      
    SCCB_Wait();
    SCL_Low;
    SCCB_Wait();
  }
  SDA_High;
  SDA_IN();
  SCCB_Wait();
  
  SCL_High;
  SCCB_Wait();
  Ack = SDA_Data;
  SCL_Low;
  SCCB_Wait();
  return Ack;
}
/*************************************************************** 

* 
* �������ƣ�uint8 SCCB_ReadByte(void) 
* ����˵����SCCB��ȡ�ֽں��� 
* ����˵���� 
* �������أ���ȡ�ֽ� 
* �޸�ʱ�䣺2017��12��5�� 
* �� ע�� 
***************************************************************/ 
uint8 SCCB_ReadByte(void) 
{ 
  uint8 i; 
  uint8_t byte; 
  SCL_OUT(); 
  SDA_IN(); //ʹ������
  for( i=0; i<8; i++) 
  { 
    SCL_Low;
    SCCB_Wait(); 
    SCL_High;
    SCCB_Wait();
    byte = (byte<<1)|(SDA_Data & 1);
  }
  SCL_Low; 
  SDA_OUT();
  SCCB_Wait(); 
  return byte; 
} 
/*************************************************************** 

* 
* �������ƣ�static void SCCB_Ack(void) 
* ����˵����IIC�лظ��ź� 
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2017��12��5�� 
* �� ע�� 
***************************************************************/ 
static void SCCB_Ack(void) 
{ 
  SCL_OUT(); 
  SDA_OUT();
  SCL_Low;
  SDA_Low;
  SCCB_Wait();
  SCL_High;
  SCCB_Wait();
  SCL_Low;
  SCCB_Wait();
} 
/*************************************************************** 

* 
* �������ƣ�static void SCCB_NAck(void) 
* ����˵����IIC�޻ظ��ź� 
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2017��12��5�� 
* �� ע�� 
***************************************************************/ 
static void SCCB_NAck(void) 
{ 
  SCL_OUT(); 
  SDA_OUT();
  SCL_Low;
  SCCB_Wait();
  SDA_High;
  SCCB_Wait();
  SCL_High;
  SCCB_Wait();
  SCL_Low;
  SCCB_Wait();
} 

/*************************************************************************
* �����������ܿƼ� KV58���ܳ�ĸ��           
*
*  �������ƣ�void SCCB_RegWrite(uint8 Device,uint8 Address,uint16 Data)
*  ����˵�������豸д����
*  ����˵����Ҫ���͵��ֽ�
*  �������أ�Ӧ���ź�
*  �޸�ʱ�䣺2017��12��5��
*  ��    ע��
*************************************************************************/
void SCCB_RegWrite(uint8 Device,uint8 Address,uint16 Data)
{
  uint8 i;
  uint8 Ack;
  
  for( i=0; i<3; i++)
  {
    SCCB_Star();
    Ack = SCCB_SendByte(Device);
    if( Ack == 1 )
    {
      continue;
    }
    
    Ack = SCCB_SendByte(Address);
    if( Ack == 1 )
    {
      continue;
    }
    
    Ack = SCCB_SendByte((uint8)(Data>>8));
    Ack = SCCB_SendByte((uint8)Data);
    if( Ack == 1 )
    {
      continue;
    }
    
    SCCB_Stop();
    if( Ack == 0 ) break;
  }
}
/*************************************************************** 

* 
* �������ƣ�uint8_t SCB_RegRead(uint8_t Device,uint8_t Address,uint16_t *Data) 
* ����˵������ȡ���� 
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2017��12��5�� 
* �� ע�� 
***************************************************************/ 
uint8_t SCCB_RegRead(uint8_t Device,uint8_t Address,uint16_t *Data) 
{   
  uint8 Ack = 0;
  Device = Device<<1;
  SCCB_Star();
  Ack += SCCB_SendByte(Device);
  
  Ack += SCCB_SendByte(Address);
  
  SCCB_Star();
  Ack += SCCB_SendByte(Device + 1);
  
  *Data = SCCB_ReadByte();
  SCCB_Ack();
  *Data = *Data<<8;
  
  *Data += SCCB_ReadByte();
  SCCB_NAck();
  
  SCCB_Stop();
  
  return  Ack;
} 
/***************************************************************  
* 
* �������ƣ�int SCCB_Probe(uint8_t chipAddr) 
* ����˵������ѯ�õ�ַ���豸�Ƿ���� 
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2017��12��5�� 
* �� ע�� 
***************************************************************/ 
int SCCB_Probe(uint8_t chipAddr) 
{ 
  uint8_t err;
  err = 0;
  chipAddr <<= 1;
  
  SCCB_Star();
  err = SCCB_SendByte(chipAddr);
  SCCB_Stop();
  
  return err;
}
/***************************************************************
* 
* �������ƣ�SendPicture 
* ����˵��������ͼ����λ�� ����ͬ����λ��ע���޸Ķ�Ӧ�����ݽ���Э��
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2018��3��27�� 
* �� ע�� 
***************************************************************/ 
void UARTSendPicture2(uint8_t tmImage[IMAGEH][IMAGEW]) 
{ 
  int i = 0, j = 0; 
  uart_putchar(UART_0,0xFE); //����֡ͷ��־ WindowsFormsApplication1.exe
  uart_putchar(UART_0,0xEF); //����֡ͷ��־ WindowsFormsApplication1.exe
  for(i=0;i < IMAGEH; i++) 
  { 
    for(j=0;j <IMAGEW;j++) 
    { 
      if(tmImage[i][j]==0xff) 
      { 
        tmImage[i][j]=0xfe; //��ֹ���ͱ�־λ 
      } 
      uart_putchar(UART_0,tmImage[i][j]); 
    } 
  }
  uart_putchar(UART_0,0xEF); //����֡β��־ 
  uart_putchar(UART_0,0xFE); //����֡β��־ 
} 

/*void UARTSendPicture(uint8_t tmImage[IMAGEH][IMAGEW]) 
{ 
  int i = 0, j = 0; 
  uart_putchar(UART_0,0xFF); //����֡ͷ��־ DEMOK��λ��  
  for(i=0;i < IMAGEH; i++) 
  { 
    for(j=0;j <IMAGEW;j++) 
    { 
      if(tmImage[i][j]==0xFF) 
      { 
        tmImage[i][j]=0xFE; //��ֹ���ͱ�־λ 
      } 
      uart_putchar(UART_0,tmImage[i][j]); 
    } 
  }
} */
/*
void SendPicture(void)
{
  int i = 0, j = 0;
  UART_Put_Char(UART_4,0xff);//����֡ͷ��־
  for(i=0;i<Frame_Height;i++)      //���
  {
    for(j=0;j<Frame_Width;j++)    
    {
      if(Image_Data[i][j]==0xff)
      {
        Image_Data[i][j]=0xfe;//��ֹ���ͱ�־λ
      }
      UART_Put_Char(UART_4,Image_Data[i][j]);
    }
  }
}
*/

/*************************************************************** 
* 
* �������ƣ�uint8_t GetOSTU(uint8_t tmImage[IMAGEH][IMAGEW]) 
* ����˵��������ֵ��С 
* ����˵���� 
* �������أ���ֵ��С 
* �޸�ʱ�䣺2018��3��27�� 
* �� ע�� 
�ο���https://blog.csdn.net/zyzhangyue/article/details/45841255
      https://www.cnblogs.com/moon1992/p/5092726.html
      https://www.cnblogs.com/zhonghuasong/p/7250540.html     
Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ����
4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��ı���w0����ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����������) ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
6) i++��ת��4)��ֱ��iΪ256ʱ��������
7�������g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
ȱ��:OSTU�㷨�ڴ������ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
***************************************************************/ 
uint8_t GetOSTU(uint8 tmImage[IMAGEH][IMAGEW]) 
{ 
  int16_t i,j; 
  uint32_t Amount = 0; 
  uint32_t PixelBack = 0; 
  uint32_t PixelIntegralBack = 0; 
  uint32_t PixelIntegral = 0; 
  int32_t PixelIntegralFore = 0; 
  int32_t PixelFore = 0; 
  double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��; 
  int16_t MinValue, MaxValue; 
  uint8_t Threshold = 0;
  uint8_t HistoGram[256];              //  

  for (j = 0; j < 256; j++)  HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ��ÿ���Ҷȼ���0
  
  for (j = 0; j < IMAGEH; j++) 
  { 
    for (i = 0; i < IMAGEW; i++) 
    { 
      HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    } 
  } 
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ
      
  if (MaxValue == MinValue)     return MaxValue;         // ͼ����ֻ��һ���Ҷ�   
  if (MinValue + 1 == MaxValue)  return MinValue;        // ͼ����ֻ�ж����Ҷ�
    
  for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];   //��С�Ҷȵ����Ҷ���������
    
  PixelIntegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    PixelIntegral += HistoGram[j] * j;//�Ҷ�ֵ*���ظ���
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)//����ÿһ���Ҷ�ֵ
  {
    PixelBack = PixelBack + HistoGram[j];   //ǰ�����ص���
    PixelFore = Amount - PixelBack;         //�������ص���
    OmegaBack = (double)PixelBack / Amount;//ǰ�����ذٷֱ�
    OmegaFore = (double)PixelFore / Amount;//�������ذٷֱ�
    PixelIntegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
    PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
    MicroBack = (double)PixelIntegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
    MicroFore = (double)PixelIntegralFore / PixelFore;   //�����ҶȰٷֱ�
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//������䷽��
    if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //���������ֵ;
} 
/*************************************************************** 
* 
* �������ƣ�void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW]) 
* ����˵����ͼ�����ݶ�ֵ�� 
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2018��3��27�� 
* �� ע�� 
***************************************************************/ 
void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW],uint8_t ThresholdV) 
{ 
  int i = 0, j = 0; 
  for(i = 0;i < IMAGEH;i++) 
  { 
    for(j = 0; j< IMAGEW;j++) 
    { 
      if(tmImage[i][j] >= ThresholdV) 
      { 
        tmImage[i][j] = 0xFE; 
      } 
      else 
      { 
        tmImage[i][j] = 0X00; 
      } 
    } 
  } 
} 




/*************************************************************** 
* 
* �������ƣ�void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW]) 
* ����˵����ͼ�����ݶ�ֵ����������λ�����д��� ��
* ����˵���� 
* �������أ�void 
* �޸�ʱ�䣺2018��3��27�� 
* �� ע�� 
***************************************************************/ 
/*void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW],uint8_t ThresholdV) 
{ 
  int i = 0, j = 0; 
  for(i = 0;i < IMAGEH;i++) 
  { 
    for(j = 0; j< IMAGEW;j++) 
    { 
      if(tmImage[i][j] >= ThresholdV) 
      { 
        tmImage[i][j] = 0xFE; 
      } 
      else 
      { 
        tmImage[i][j] = 0X00; 
      } 
    } 
  } 
} 
*/







