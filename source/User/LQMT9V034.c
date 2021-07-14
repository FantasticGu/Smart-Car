/*******************************************************************************
��ƽ    ̨������K66FX���ܳ�VDĸ��
����    д��CHIUSIR
��E-mail  ��chiusir@163.com
������汾��V1.0
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






//����ͷͼ��ɼ��жϴ�����
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



#define TRIG PTE2 
#define ECHO PTE3


int rstatus = 0;
int lastrstatus = 0;
int lastlstatus = 0;
int lstatus = 0;
int havestop = 0;
int incross = 0;

int Mmin(int a,int b)
{
  return a<b?a:b;
}


int judgeRight(int begin,int end,int val)
{
  int rightVal = 0;
  for(int i = H-begin;i>H-end;i--)
  {
    rightVal += Bline_right[i];
  }
  rightVal /= (end-begin);
  if(rightVal > val)
  {
    return 1;
  }
  return 0;
  
}

int judgeLeft(int begin,int end,int val)
{
  int leftVal = 0;
  for(int i = H-begin;i>H-end;i--)
  {
    leftVal += Bline_left[i];
  }
  leftVal /= (end-begin);
  if(leftVal > val)
  {
    return 1;
  }
  return 0;
  
}

int speedflag = 0;
float left_slope_queue[5]={0};
float right_slope_queue[5]={0};
static unsigned int queue_cnt = 0;

#define START_STATUS 0
#define CIRCLE_STATUS 1
#define CROSS_STATUS 2
#define THREE_ROAD_STATUS 3
#define APRIL_TAG_STATUS 4
#define STOP_STATUS 5

int DIRECTION = 1; //0:��ʱ�룬1��˳ʱ�롣

/*int TOTAL_STATUS[] = {START_STATUS,
                     //CIRCLE_STATUS,CROSS_STATUS,CROSS_STATUS,CIRCLE_STATUS,
                      THREE_ROAD_STATUS,APRIL_TAG_STATUS,THREE_ROAD_STATUS,
                      CIRCLE_STATUS,CROSS_STATUS,CROSS_STATUS,CIRCLE_STATUS,STOP_STATUS,
                      THREE_ROAD_STATUS,APRIL_TAG_STATUS,THREE_ROAD_STATUS,
                      CIRCLE_STATUS,CROSS_STATUS,CROSS_STATUS,CIRCLE_STATUS,STOP_STATUS};*/

int TOTAL_STATUS[] =
{START_STATUS,
                      CIRCLE_STATUS,CROSS_STATUS,CROSS_STATUS,
                      THREE_ROAD_STATUS,APRIL_TAG_STATUS,THREE_ROAD_STATUS,
                      CIRCLE_STATUS,
                      STOP_STATUS,
                      CIRCLE_STATUS,CROSS_STATUS,CROSS_STATUS,
                      THREE_ROAD_STATUS,APRIL_TAG_STATUS,THREE_ROAD_STATUS,
                      CIRCLE_STATUS,STOP_STATUS
};

/*
{                     START_STATUS,
                      CIRCLE_STATUS,
  //APRIL_TAG_STATUS,
                      THREE_ROAD_STATUS,APRIL_TAG_STATUS,THREE_ROAD_STATUS,
                      CROSS_STATUS,CROSS_STATUS,CIRCLE_STATUS,
                      STOP_STATUS,
                      CIRCLE_STATUS,
                      THREE_ROAD_STATUS,APRIL_TAG_STATUS,THREE_ROAD_STATUS,
                      CROSS_STATUS,CROSS_STATUS,CIRCLE_STATUS,STOP_STATUS};

*/
//{START_STATUS,CIRCLE_STATUS,CROSS_STATUS,CROSS_STATUS,CIRCLE_STATUS,THREE_ROAD_STATUS,
//                     CIRCLE_STATUS,CROSS_STATUS,CROSS_STATUS,CIRCLE_STATUS,THREE_ROAD_STATUS,STOP_STATUS};

int shoot_error = 0;
int shoot_flag = 0;

int recv_msg_flag = 0;

int april_tag_position = 1; //left: 0, right: 1

int STATUS_NOW = 0;
int STATUS_LAST = 0;
    
int ready_to_stop = 0;

int signal_number = 2;
int real_signal_number = 2;
int april_tag_number = 0;
int april_tag_in_three_road = 1;
int is_fruit = 0;

int LAP_NUM = 0;

void queue_in(float* queue,float data)
{
  queue[(queue_cnt++)%5] = data;
}




void imagineProcess(void)
{
    
  while(1)
  {  
       if(Field_Over_Flag)    //���һ��ͼ��ɼ�����ʾ���������ݵ���λ��
      {
          //exti_disable(PTD14); //���жϹر�
          exti_disable(PTD15); //���жϹر� 
          
          
          //SendPicture();
          find_edge();//
          queue_in(left_slope_queue,regression(H-41,H-21,Bline_left));
          queue_in(right_slope_queue,regression(H-41,H-21,Bline_right));
          float left_slope = 0;
          float right_slope = 0;
          for(int i = 0;i<5;i++)
          {
            left_slope += left_slope_queue[i];
            right_slope += right_slope_queue[i];
          }
          left_slope /= 5;
          right_slope /= 5;
          static int start_cnt = 0;
          
          if(STATUS_LAST != STATUS_NOW)
          {
            //setpoint1 = setpoint2 = 0;
            //delayms(500);
            //setpoint1 = setpoint2 = NORMAL_SPEED;
            gpio_set(PTD12,1);
          }else
          {
            gpio_set(PTD12,0);
          }
          
          STATUS_LAST = STATUS_NOW;
          
          int status = TOTAL_STATUS[STATUS_NOW]; 
          
          
          /*if(1)
          {
            int back_left = H-11;
            int front_left = H-61
            for(int i = H-11;i<H-81;i--)
            {
              if(Bline_left[i] >= Bline_left[i+1] && Bline_left[i-1] >= Bline_left[i] && Bline_left[i-2]-Bline_left[i-1] < -3)
              {
                
                
              }
            }
          }*/
            
          
          
                    
         /*for(int i = H-11;i>H-61;i--)
          {
            uart_printf(UART_0,"%d %d %d \n",Bline_left[i],Pick_table[i],Bline_right[i]);
            delayms(50);
          }
         //uart_printf(UART_0,"%f %f \n",left_slope,right_slope);
         uart_printf(UART_0,"\n\n");*/
          
          if(status == CIRCLE_STATUS)
          {
            
            if(lastrstatus != rstatus || lastlstatus != lstatus)
            {
              gpio_set(PTD12,1);
            }else
            {
              gpio_set(PTD12,0);
            }
            
            lastrstatus = rstatus;
            lastlstatus = lstatus;
            
            
             if(judgeRight(31,41,180) == 1 && judgeRight(16,26,170) == 0 && !rstatus && !lstatus)
             {
               int p1 = 0;
               int p2 = 0;
               int p3 = 0;
               for(int j = H-11;j>H-14;j--)
               {
                 if (Bline_left[j]>5)
                 {
                    p1 = Bline_left[j];
                    break;
                 }
                 
               } 
               for(int j = H-19;j>H-22;j--)
               {
                 if (Bline_left[j]>10)
                 {
                    p2 = Bline_left[j];
                    break;
                 }
                 
               } 
               for(int j = H-26;j>H-30;j--)
               {
                 if (Bline_left[j]>15)
                 {
                    p3 = Bline_left[j];
                    break;
                 }
                 
               }
               if(p1 && p2 && p3 && abs(p2-p1)<15 && abs(p3-p2)<15 && abs(p3-p1)<30)
               {
                 int l = 0;
                 for(int j = H-11;j>H-41;j--)
                 {
                   if(Bline_left[j]>15)
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
               static int cnt_1 = 0;
               cnt_1++;
               for(int i = H-21;i>H-41;i--)
               {
                 Pick_table[i] = Bline_left[i]+60;
               }
               if(cnt_1 > 5 && judgeRight(20,30,168) == 0)
               {
                 cnt_1 = 0;
                 rstatus = 2;
               }
               if(cnt_1 > 25)
               {
                 cnt_1 = 0;
                 rstatus = 0;
               }
               
             }
         
             if(rstatus == 2)
             {
               static int cnt_2 = 0;
               cnt_2++;
               if(cnt_2 > 20)
               {
                 cnt_2 = 0;
                 rstatus = 0;
               }
               if(judgeRight(25,35,175)==1)
               {
                 cnt_2 = 0;
                 rstatus = 3;
               }
               
             }
             
             if(rstatus==3)
             {
                static int cnt_3 = 0;
                 cnt_3++;
                  for(int i = H-11;i>H-41;i--)
                 {
                   Pick_table[i] = Bline_right[i]-45;
                 }
                 if(cnt_3 > 8)
                 {
                   cnt_3 = 0;
                   rstatus = 4;
                   
                 }
             }
             
             if(rstatus == 4)
             {
               static int cnt_4 = 0;
                   cnt_4++;
                   if(cnt_4>20)
                   {
                     if(judgeLeft(15,25,10) == 0)
                     {
                       cnt_4 = 0;
                       rstatus = 5;
                       
                     }
                   }
               
             }
             
             if(rstatus == 5)
             {
               static int cnt_6 = 0;
               cnt_6++;
               for(int i = H-11;i>H-41;i--)
               {
                 Pick_table[i] = Bline_right[i]-50;
               }
               if(cnt_6 > 12 && judgeLeft(15,25,15) == 1)
               {
                 cnt_6 = 0;
                 rstatus = 7;
               }
               
             }
                       
                 
             if(rstatus == 7)
             {
               static int cnt_7 = 0;
               cnt_7 ++;
               for(int i = H-11;i>H-41;i--)
               {
                 Pick_table[i] = Bline_left[i]+50;
               }
               if(cnt_7 > 30 || cnt_7 > 15 && judgeLeft(20,40,20) == 1 && judgeRight(20,40,170) == 0)
               {
                 cnt_7 = 0;
                 rstatus = 0;
                 STATUS_NOW++;
               }
             }
          
              
              
            /*for(int i = H-15;i>H-35;i--)
            {
              if(Bline_left[i+1]>30 && Bline_left[i]>30 && Bline_left[i-1]< 5 && Bline_left[i-2] < 5)
              {
                if(right_slope > 0.1)
                {
                  lstatus = 1;
                  break;
                }
              }
              
            }*/
             
             
            if(judgeLeft(31,41,8) == 0 && judgeLeft(16,26,18) == 1 && !rstatus && !lstatus)
            {
              /*if(right_slope > 0.1)
              {
                lstatus = 1;
              }*/
              int p1 = 0;
              int p2 = 0;
              int p3 = 0;
              for(int j = H-11;j>H-14;j--)
              {
                if (Bline_right[j]<185)
                {
                   p1 = Bline_right[j];
                   break;
                }
                
              } 
              for(int j = H-19;j>H-22;j--)
              {
                if (Bline_right[j]<180)
                {
                   p2 = Bline_right[j];
                   break;
                }
                
              } 
              for(int j = H-26;j>H-30;j--)
              {
                if (Bline_right[j]<175)
                {
                   p3 = Bline_right[j];
                   break;
                }
                
              }
              if(p1 && p2 && p3 && abs(p2-p1)<15 && abs(p3-p2)<15 && abs(p3-p1)<30)
              {
                int r = 0;
                for(int j = H-11;j>H-41;j--)
                {
                  if(Bline_right[j]<180)
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
              static int cnt_1 = 0;
              cnt_1++;
              for(int i = H-11;i>H-41;i--)
              {
                Pick_table[i] = Bline_right[i]-60;
              }
              if(cnt_1 > 5 && judgeLeft(20,30,20) == 1)
              {
                cnt_1 = 0;
                lstatus = 2;
              }
              if(cnt_1 > 25)
              {
                cnt_1 = 0;
                lstatus = 0;
              }
              
            }
            if(lstatus == 2)
            {               
              static int cnt_2 = 0;
              cnt_2++;
              if(cnt_2 > 20)
              {
                cnt_2 = 0;
                lstatus = 0;
              }
              if(judgeLeft(25,35,15) == 0)
              {
                cnt_2 = 0;
                lstatus = 3;
              }
              
            }
            if(lstatus==3)
            {
              static int cnt_3 = 0;
              cnt_3++;
               for(int i = H-11;i>H-41;i--)
              {
                Pick_table[i] = Bline_left[i]+45;
              }
              if(cnt_3 > 8)
              {
                  cnt_3 = 0;
                  lstatus = 4;
              }
            }
            
            if(lstatus==4)
            {
              static int cnt_4 = 0;
              cnt_4++;
              if(cnt_4>20)
              {
                if(judgeRight(15,25,170) == 1)
                {
                  cnt_4 = 0;
                  lstatus = 5;
                }   
              }
            }
            if(lstatus == 5)
            {
              static int cnt_5 = 0;
              cnt_5++;
              for(int i = H-11;i>H-41;i--)
              {
                Pick_table[i] = Bline_left[i]+50;
              }
              if(cnt_5> 12 && judgeRight(25,35,170)==0)
              {
                cnt_5 = 0;
                lstatus = 7;
              }
            }
              
            
            if(lstatus == 7)
            {
              static int cnt_7 = 0;
              cnt_7 += 1;
              for(int i = H-11;i>H-41;i--)
              {
                Pick_table[i] = Bline_right[i]-50;
              }
              if(cnt_7 > 30 || cnt_7 >15 && judgeLeft(15,25,20) == 1)
              {
                cnt_7 = 0;
                lstatus = 0;
                STATUS_NOW++;
              }
            }
          }
            
          static int cstatus = 0;       
          static int angle_status = 0;
          
          if(status == THREE_ROAD_STATUS)
          {
            if(left_slope > 0 && right_slope < 0
            && angle_status < 2)
            {
               angle_status = 1;
               int jumpl=94,jumpr=94;
               int matchpointl = 0,matchpointr = 0;
               for(int i = H-51;i>H-81;i--)
               {
                 for(int j = 30;j<V/2-1;j++)
                 {
                   if(Image_Data[i][j-2] > Cmp &&Image_Data[i][j-1] > Cmp 
                      && Image_Data[i][j]<Cmp && Image_Data[i][j+1]<Cmp 
                      && j - jumpl< -1)
                   {
                     matchpointl++;
                     if(jumpl-j<20)
                     {
                     jumpl = j;
                     }
                     continue;
                   }
                 }
                 for(int j = V-30;j>V/2+1;j--)
                 {
                   if(Image_Data[i][j-2] < Cmp &&Image_Data[i][j-1] < Cmp 
                      && Image_Data[i][j]>Cmp && Image_Data[i][j+1]>Cmp
                      && j-jumpr>1 )
                   {
                     matchpointr++;
                     if(j - jumpr < 20)
                     {
                       jumpr = j;
                     }

                     continue;
                   }
                 }
               }
               if(matchpointl >= 5 && matchpointr >= 5)
               {
                 if(LAP_NUM == 0)
                 {
                   setpoint1 = setpoint2 = -100;
                   delayms(500);
                   setpoint1 = setpoint2 = 0;
                   uart_putchar(UART_0,'n');
                   while(1)
                   {
                     delayms(100);
                     if(recv_msg_flag)
                     {
                       gpio_set(PTD12,1);
                       delayms(100);
                       gpio_set(PTD12,0);
                       real_signal_number = signal_number;
                       recv_msg_flag = 0;
                       break;
                     }
                   }
                   setpoint1 = setpoint2 = -30;
                   delayms(500);
                   
                   if(real_signal_number % 2)
                   {
                      angle_status = 2; 
                   }
                   else
                   {
                     angle_status = 5;
                   }
                   setpoint1 = setpoint2 = NORMAL_SPEED;
                 }
                 else
                 {
                   if( real_signal_number % 2)
                   {
                      angle_status = 5; 
                   }
                   else
                   {
                     angle_status = 2;
                   }
                 }
               }
            }
            
            if(angle_status == 2)
            {
               static int angle_in_cnt = 0;
               for(int i = H-21;i>H-41;i--)
                {
                   Pick_table[i] = Bline_left[i]+50;
                }
               angle_in_cnt++;
               if(angle_in_cnt > 20 && LAP_NUM ==0 || angle_in_cnt > 10 && LAP_NUM ==1)
               {
                 angle_status = 3;
                 STATUS_NOW++;
                 if(april_tag_position == 1 && april_tag_in_three_road)
                 {
                   STATUS_NOW++;
                 }
                 angle_in_cnt = 0;
               }
            }
            
            if(angle_status == 3)
            {
              if(left_slope > 0 && right_slope < 0)
              {
                angle_status = 4;
              }
            }
            
            if(angle_status == 4)
            {
              static int angle_out_cnt = 0;
               for(int i = H-21;i>H-41;i--)
                {
                   Pick_table[i] = Bline_left[i]+50;
                }
               angle_out_cnt++;
               if(angle_out_cnt > 10)
               {
                 angle_status = 0;
                 angle_out_cnt = 0;
                 STATUS_NOW++;
               }
            }
            
            if(angle_status == 5)
            {
               static int angle_in_cnt = 0;
               for(int i = H-21;i>H-41;i--)
                {
                   Pick_table[i] = Bline_right[i]-50;
                }
               angle_in_cnt++;
               if(angle_in_cnt > 23  && LAP_NUM ==0 || angle_in_cnt > 10 && LAP_NUM ==1)
               {
                 angle_status = 6;
                 angle_in_cnt = 0;
                 STATUS_NOW++;
                 if(april_tag_in_three_road && april_tag_position == 0)
                 {
                   STATUS_NOW++;
                 }
               }
            }
            
            if(angle_status == 6)
            {
              if(left_slope > 0 && right_slope < 0)
              {
                angle_status = 7;
              }
            }
            
            if(angle_status == 7)
            {
              static int angle_out_cnt = 0;
               for(int i = H-21;i>H-41;i--)
                {
                   Pick_table[i] = 130;
                }
               angle_out_cnt++;
               if(angle_out_cnt > 5)
               {
                 angle_status = 0;
                 angle_out_cnt = 0;
                 STATUS_NOW++;
               }
            }
            
          }
          
          
          
          if(status == CROSS_STATUS)
          {
             static int cnt_c1 = 0;
             if(judgeLeft(15,35,5) == 0 && judgeRight(15,35,183) == 1 && !cstatus)
             {
               cstatus = 1;
               lstatus = rstatus = 0;
             }
          
             if(cstatus == 1)
             {
               /*if(Bline_left[H-10] > 15 && Bline_right[H-10] < 175 && Bline_left[H-11] > 15 && Bline_right[H-11] < 175)
               {
                 int valid_line_left = H-81;
                 int valid_line_right = H-81;
                 int find_valid_flag_left = 0;
                 int find_valid_flag_right = 0;
                 for(int i = H-81;i > H-1;i--)
                 {
                   if(Bline_left[i] > 15 && Bline_left[i+1] < 5)
                   {
                     valid_line_left = i;
                     find_valid_flag_left = 1;
                     break;
                   }
                 }
                 
                 for(int i = H-81;i > H-1;i--)
                 {
                   if(Bline_right[i] < 170 && Bline_right[i+1] > 180)
                   {
                     valid_line_right = i;
                     find_valid_flag_right = 1;
                     break;
                   }
                 }
                 
                 if(find_valid_flag_left && find_valid_flag_right)
                 {
                   double k_left = (Bline_left[valid_line_left]-Bline_left[H-10])/(valid_line_left - (H - 10));
                   double k_right = (Bline_right[valid_line_right]-Bline_right[H-10])/(valid_line_right - (H - 10));
                   for(int i = valid_line_left+1; i <= H-21; i++)
                   {
                     Bline_left[i] = Bline_left[i-1]+k_left*(i-valid_line_left);
                   }
                   for(int i = valid_line_right+1; i <= H-21; i++)
                   {
                     Bline_right[i] = Bline_right[i-1]+k_right*(i-valid_line_right);
                   }
                   for(int i = H-41; i <= H-21; i++)
                   {
                     Pick_table[i] = (Bline_left[i] + Bline_right[i])/2;
                   }
                     
                 }
                 
               }
               else
               {*/
                 int valid_line = H-61;
                 int find_valid_flag = 0;
                 for(int i = H-21;i > H-61;i--)
                 {
                   if(Bline_left[i] > 20 && Bline_right[i] < 170)
                   {
                     valid_line = i;
                     find_valid_flag = 1;
                     break;
                   }
                 }
                 
                 if(find_valid_flag)
                 {
                   for(int i = valid_line+1; i <= H-21; i++)
                   {
                     Bline_left[i] = Bline_left[i-1];
                     Bline_right[i] = Bline_right[i-1];
                     Pick_table[i] = (Bline_left[i] + Bline_right[i])/2;
                   }
                 }
                 
               //}


               
               cnt_c1++;
               if(cnt_c1 > 10 || cnt_c1>5 && judgeLeft(15,25,15) == 1 && judgeRight(15,25,173) == 0)
               {
                 cnt_c1 = 0;
                 cstatus = 0;
                STATUS_NOW++;
               }
             }
          }
              
           
         if(status != START_STATUS && !ready_to_stop && !cstatus && !rstatus && !lstatus)
         {
           
           if(speedflag != 1)
           {
              setpoint1 = setpoint2 = NORMAL_SPEED;
              speedflag = 1;
           }
         }
         else if(rstatus || lstatus)
         {
           if(speedflag != 2)
           {
              setpoint1 = setpoint2 = CIRCLE_SPEED;
              speedflag = 2;
           }
         }
         else if(cstatus)
         {
           if(speedflag != 3)
           {
              //PWMSetSteer(1250);
              setpoint1 = setpoint2 = CROSS_SPEED;
              speedflag = 3;
           }
         }
        else if(status == START_STATUS)
        {
          setpoint1 = setpoint2 = START_SPEED;
          speedflag = 4;
        }
        else if(ready_to_stop)
        {
          if(speedflag != 5)
          {
            setpoint1 = setpoint2 = STOP_SPEED;
              speedflag = 5;
          }
        }
           
           
          static int sstatus=1;
          
          if(status == START_STATUS)
          {
            
            if(sstatus == 1)
            {
               start_cnt++;
               
               if(start_cnt <= 10)
               {
                 PWMSetSteer(1250);
               }
               
               if(start_cnt < 30 && start_cnt > 10)
               {
                 if(!DIRECTION)
                  {
                    for(int i = H-21;i>H-41;i--)
                    {
                       Pick_table[i] = Bline_left[i]+50;
                    }           
                  }
                  else
                  {
                    for(int i = H-21;i>H-41;i--)
                    {
                       Pick_table[i] = Bline_right[i]-50;
                    } 
                  }
               }
              
              if(start_cnt > 30)
              {
                start_cnt = 0;
                STATUS_NOW++;
                sstatus = 0;
              }        
            }
          }
          
           if(status == STOP_STATUS)
           {
              static int stopflag = 0;
              int jumppoint = 0;
              if(!stopflag)
              {
                for(int i = H-11;i>H-61;i--)
                {
                  for(int j = V-10;j>V-185;j--)
                  {
                    if(Image_Data[i][j-2]>Cmp && Image_Data[i][j-1]>Cmp 
                      && Image_Data[i][j]<Cmp && Image_Data[i][j+1]<Cmp )
                    {
                      jumppoint++;
                    }
                  }
                  if(jumppoint > 4)
                  {
                    if(!LAP_NUM)
                    {
                      stopflag = 1;
                      LAP_NUM = 1;
                      break;
                    }
                    else
                    {
                      ready_to_stop = 1;
                      setpoint1 = setpoint2 = STOP_SPEED;
                      if(!DIRECTION)
                      {
                        stopflag = 3;
                      }
                      else
                      {
                        stopflag = 4;
                      }
                      break;
                    }
                  }
                    jumppoint = 0;
                    if(stopflag)
                    {
                      break;
                    }
                }
              }
              
              if(stopflag == 1)
              {
                 static int stop_cnt = 0;
                 if(!DIRECTION)
                 {
                   for(int i = H-61;i>H-21;i++)
                   {
                      if(Bline_right[i+1] - Bline_right[i] < 0)
                      {
                        Bline_right[i+1] = Bline_right[i];
                      }
                   }
                   for(int i = H-21;i>H-41;i--)
                   {
                      Pick_table[i] = Bline_right[i]-50;
                   }
                 }
                 else
                 {
                   for(int i = H-61;i>H-21;i++)
                   {
                      if(Bline_left[i+1] - Bline_left[i] > 0)
                      {
                        Bline_left[i+1] = Bline_left[i];
                      }
                   }
                   for(int i = H-21;i>H-41;i--)
                   {
                      Pick_table[i] = Bline_left[i]+50;
                   }
                 }
                 
                 stop_cnt++;
                 if(stop_cnt>15)
                 {
                   stop_cnt = 0;
                   stopflag = 0;
                   STATUS_NOW++;
                 }
              }
              
              if(stopflag == 3)
              {
                 static int black_cnt = 0;
                 for(int i = H-21;i>H-41;i--)
                 {
                    Pick_table[i] = Bline_left[i]+40;
                 }
                 black_cnt++;
                 if(black_cnt>15)
                 {
                   havestop=1;
                   setpoint1 = setpoint2 = 0;
                   delayms(100000);
                 }
                 
              }
              
              if(stopflag == 4)
              {
                 static int black_cnt = 0;
                 for(int i = H-21;i>H-41;i--)
                 {
                    Pick_table[i] = Bline_right[i]-50;
                 }
                 black_cnt++;
                 if(black_cnt>15)
                 {
                   havestop=1;
                   setpoint1 = setpoint2 = 0;
                   delayms(100000);
                 }
                 
              }
              
           }
          static int april_tag_cnt = 0;
          static int april_tag_status = 0;
          if(status == APRIL_TAG_STATUS)
          {
            if(april_tag_status == 0)
            {
              int jumppoint = 0;
              for(int i = H-21;i>H-41;i--)
              {
                for(int j = Bline_left[i]+5;j<Bline_right[i]-5;j++)
                {
                  if(Image_Data[i][j-1]>Cmp && Image_Data[i][j]<Cmp 
                     || Image_Data[i][j-1]<Cmp && Image_Data[i][j]>Cmp)
                  {
                    jumppoint++;
                  }
                }
                if(jumppoint >= 5)
                {
                  int steer_status = 1800;
                  PWMSetSteer(1250);
                  setpoint1 = setpoint2 = -50;
                  delayms(1000);
                  setpoint1 = setpoint2 = 0;
                  uart_putchar(UART_0,'a');
                  while(1)
                  {
                     delayms(100);
                     if(recv_msg_flag)
                     {
                       gpio_set(PTD12,1);
                       delayms(100);
                       gpio_set(PTD12,0);
                       april_tag_number = signal_number;
                       recv_msg_flag = 0;
                       break;
                     }
                   }
                  setpoint1 = setpoint2 = 50;
                  delayms(750);
                  setpoint1 = setpoint2 = 0;
                  if(!(april_tag_number % 2))
                  {
                    my_steer_set(550);
                    steer_status = 550;
                  }
                  else
                  {
                    my_steer_set(3050);
                    steer_status = 3050;
                  }
                  delayms(500);
                  uart_putchar(UART_0,'f');
                  while(1)
                  {
                     delayms(100);
                     if(recv_msg_flag)
                     {
                       gpio_set(PTD12,1);
                       //delayms(500);
                       gpio_set(PTD12,0);
                       is_fruit = signal_number;
                       recv_msg_flag = 0;
                       break;
                     }
                   }
                  if(!is_fruit)
                  {
                    //delayms(2000);
                  }
                  else
                  {
                    for(int i=0;i<5;i++)
                    {
                    gpio_set(PTD12,1);
                    delayms(100);
                    gpio_set(PTD12,0);
                    //delayms(200);
                    }
                    if(april_tag_number % 2)
                    {
                    rl_again:
                    shoot_flag = 1;
                    uart_putchar(UART_0,'r');
                    while(1)
                    {
                       delayms(100);
                       if(recv_msg_flag)
                       {
                         //gpio_set(PTD12,1);
                         //delayms(500);
                         //gpio_set(PTD12,0);
                         recv_msg_flag = 0;
                         break;
                       }
                     }
                    shoot_error-=51;
                    if(shoot_error < -10)
                    {
                      setpoint1 = setpoint2 = -30;
                      delayms(200);
                      setpoint1 = setpoint2 = 0;
                      delayms(200);
                      shoot_error = 0;
                      goto rl_again;
                      
                    }
                    else if(shoot_error >= 0)
                    {
                      //my_steer_set(1800);
                      //delayms(300);
                      switch(shoot_error/10)
                      {
                      case 0:
                        my_steer_set(3050-4*(shoot_error % 10));
                        break;
                      case 1:
                        my_steer_set(3010-3*(shoot_error % 10));
                        break;
                      case 2:
                        my_steer_set(2985-3*(shoot_error % 10));
                        break;
                      case 3:
                        my_steer_set(2960-3*(shoot_error % 10));
                        break;
                      case 4:
                        my_steer_set(2930-3*(shoot_error % 10));
                        break;
                      case 5:
                        my_steer_set(2900-3*(shoot_error % 10));
                        break;
                      case 6:
                        my_steer_set(2870-3*(shoot_error % 10));
                        break;
                      case 7:
                        my_steer_set(2840-3*(shoot_error % 10));
                        break;
                      case 8:
                        my_steer_set(2810-3*(shoot_error % 10));
                        break;
                      case 9:
                        my_steer_set(2780-3*(shoot_error % 10));
                        break;
                      default:
                        setpoint1 = setpoint2 = 30;
                        delayms(100);
                        setpoint1 = setpoint2 = 0;
                        goto rl_again;     
                      }
                    }         
                    delayms(500);
                    ftm_pwm_duty(FTM3,FTM_CH7,5000);
                    delayms(1500);
                    ftm_pwm_duty(FTM3,FTM_CH7,0);
                    delayms(500);
                    shoot_error = 0;          
                  }
                    
                    else
                    {
                      rr_again:
                    shoot_flag = 1;
                    uart_putchar(UART_0,'r');
                    while(1)
                    {
                       delayms(100);
                       if(recv_msg_flag)
                       {
                         //gpio_set(PTD12,1);
                         //delayms(500);
                         //gpio_set(PTD12,0);
                         recv_msg_flag = 0;
                         break;
                       }
                     }
                    shoot_error-=51;
                    if(shoot_error > 10)
                    {
                      setpoint1 = setpoint2 = -30;
                      delayms(200);
                      setpoint1 = setpoint2 = 0;
                      delayms(200);
                      shoot_error = 0;
                      goto rr_again;
                      
                    }
                    else if(shoot_error <= 0)
                    {
                      //my_steer_set(1800);
                      //delayms(300);
                      shoot_error *= -1;
                      switch(shoot_error/10)
                      {
                      case 0:
                        my_steer_set(550+10*(shoot_error % 10));
                        break;
                      case 1:
                        my_steer_set(650+5*(shoot_error % 10));
                        break;
                      case 2:
                        my_steer_set(680+5*(shoot_error % 10));
                        break;
                      case 3:
                        my_steer_set(710+4*(shoot_error % 10));
                        break;
                      case 4:
                        my_steer_set(740+5*(shoot_error % 10));
                        break;
                      case 5:
                        my_steer_set(785+4*(shoot_error % 10));
                        break;
                      case 6:
                        my_steer_set(820+3*(shoot_error % 10));
                        break;
                      case 7:
                        my_steer_set(850+3*(shoot_error % 10));
                        break;
                      case 8:
                        my_steer_set(875+3*(shoot_error % 10));
                        break;
                      case 9:
                        my_steer_set(910+3*(shoot_error % 10));
                        break;
                      default:
                        setpoint1 = setpoint2 = 30;
                        delayms(100);
                        setpoint1 = setpoint2 = 0;
                        goto rr_again;     
                      }
                    }         
                    delayms(500);
                    ftm_pwm_duty(FTM3,FTM_CH7,5000);
                    delayms(1500);
                    ftm_pwm_duty(FTM3,FTM_CH7,0);
                    delayms(500);
                    shoot_error = 0;          
                  }
                    }

                  
                  
                  my_steer_set(1800);
                  steer_status = 1800;
                  delayms(1000);
                  setpoint1 = setpoint2 = NORMAL_SPEED;              
                  april_tag_status = 1;
                  break;
                }
              }
            }
            
            if(april_tag_status == 1)
            {
              april_tag_cnt++;
              if(april_tag_cnt > 8)
              {
                april_tag_cnt = 0;
                april_tag_status = 0;
                STATUS_NOW++;
              }
            }
            
          }
            
      
          //uart_printf(UART_0,"right = %d\n",readyright);
  
         ele_direction_control();
          
          exti_enable(PTD15,IRQ_FALLING|PULLUP);    //���ж� 
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

  /*MT9V034_SetFrameResolution(IMAGEH, IMAGEW);//��������ͷͼ��4*4��Ƶ���PCLK, 27/4 = 6.75M ,BIT4,5��������:�������Ҿ�����   
  MT9V034_SetAutoExposure(0);  
  
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
  
  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MIN_EXPOSURE_REG, 0x0380); //0x0380   //0xAC  ��С�ֿ��ſ��   0x0001
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_EXPOSURE_REG, 0x0480); //0x0480     //0xAD  ���׿��ſ��   0x01E0-480
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_GAIN_REG, 64);             //0xAB  ���ģ������     64
  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AGC_AEC_PIXEL_COUNT_REG, 188*120);//0xB0  ����AEC/AGCֱ��ͼ������Ŀ,���44000   4096=320*240  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ADC_RES_CTRL_REG, 0x0303);     //0x1C  here is the way to regulate darkness :)    
  
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x13,0x2D2E);//We also recommended using R0x13 = 0x2D2E with this setting for better column FPN.  
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x20,0x01C7);//0x01C7�ԱȶȲ���ף�0x03C7�Աȶ���� Recommended by design to improve performance in HDR mode and when frame rate is low.
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
    */
  
  MT9V034_SetFrameResolution(IMAGEH, IMAGEW);

    /* ����֡�� */
    //MT9V034_SetFrameRate(25);

    /* �ع����� */
    MT9V034_SetAutoExposure(0);
    
    SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MIN_EXPOSURE_REG, 0x0380); //0x0380   //0xAC  ��С�ֿ��ſ��   0x0001
    SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_EXPOSURE_REG, 0x0480); //0x0480     //0xAD  ���׿��ſ��   0x01E0-480

	SCCB_RegWrite(MT9V034_I2C_ADDR, 0x2C, 0x0004);  //�ο���ѹ����   1.4v
    SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ANALOG_CTRL, MT9V034_ANTI_ECLIPSE_ENABLE);  //����ʴ
    SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ADC_RES_CTRL_REG, 0x0303);      //0x1C  here is the way to regulate darkness :)
    ////
    SCCB_RegWrite(MT9V034_I2C_ADDR,0x13,0x2D2E);//We also recommended using R0x13 = 0x2D2E with this setting for better column FPN.
    SCCB_RegWrite(MT9V034_I2C_ADDR,0x20,0x01c7);//0x01C7�ԱȶȲ���ף�0x03C7�Աȶ���� Recommended by design to improve performance in HDR mode and when frame rate is low.
    SCCB_RegWrite(MT9V034_I2C_ADDR,0x24,0x0010);//Corrects pixel negative dark offset when global reset in R0x20[9] is enabled.
    SCCB_RegWrite(MT9V034_I2C_ADDR,0x2B,0x0003);//Improves column FPN.
    SCCB_RegWrite(MT9V034_I2C_ADDR,0x2F,0x0003);//Improves FPN at near-saturation.

    SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_V2_CTRL_REG_A, 0x001A);        //0x32   0x001A
    SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_HDR_ENABLE_REG,0x0103); 



  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_AGC_AEC_DESIRED_BIN_REG, 64);
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_RESET, 0x03);   
  
  //GPIO�ڳ�ʼ��
  //exti_enable(PTD14,IRQ_RISING|PULLDOWN);   //���ж�
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
    if(enable)
    {
        /* �����Զ��ع��Զ����� */
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE, reg|MT9V034_AEC_ENABLE|MT9V034_AGC_ENABLE);
        /* ����ع�ʱ�� �޸���������޸ıȽϰ�ʱ��ͼ����������*/
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_EXPOSURE_REG, CAMERA_MAX_EXPOSURE_TIME);
        /* ��С�ع�ʱ�� �޸���������޸�����ǿ��ʱ��ͼ����������*/
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MIN_EXPOSURE_REG, CAMERA_MIN_EXPOSURE_TIME);
        /* ������� �������� ͼ��ƫ��������±��������ϸ�� ���ǿ��ܲ������ 0-60*/
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_GAIN_REG, 20);
        /* 0xB0  ����AEC/AGCֱ��ͼ������Ŀ,22560 ���44000  */
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AGC_AEC_PIXEL_COUNT_REG, 22560);
         
    }
    else
    {
        /* �ر��Զ��ع� */
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE, reg&~(MT9V034_AEC_ENABLE|MT9V034_AGC_ENABLE));
        
        /* 0xAB  ���ģ������     64 */
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_GAIN_REG, 64);  
        
        /* 0x0B �����ع�ʱ�� 0�C32765 */
        SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_TOTAL_SHUTTER_WIDTH,CAMERA_EXPOSURE_TIME);   
  
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
ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
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








