//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4
/*!                    �����ʵ��ѧ K66 ѧϰ����
 *  �ļ����ƣ�       isr.c
 *      ���ߣ�       maddevil
 *      ˵����       �����ڲ�ѧϰʹ�ã������⴫
 *  �ο����ϣ�       ����ѧ�����롢ɽ��K60�⡢����K66ģ�塢����KEAģ��
 *    �汾�ţ�       V1.0.0
 *  �����£�       2018-12-21 13:41
 */
//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4

#include "isr.h"

//UART0~UART4 �����жϷ�����
void UART2_ISR(void){}
void UART1_ISR(void){}

void UART0_ISR(void)
{    
  DisableInterrupts ;//�����ж�
  char Data=0;
  UARTn_e uratn = UART_0;//UART0 �жϷ�����
  if(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK)   //�������ݼĴ�����
  {
    Data=uart_getchar(UART_0);
    signal_number = Data - '0';
    //setpoint1 = setpoint2 = -10;
    //my_steer_set(3050);
    //uart_putchar(UART_0,Data);
  }
  if(UART_S1_REG(UARTN[uratn]) & UART_S1_TDRE_MASK)  //�������ݼĴ�����
  {
    //uart_putchar(UART_0,Data);
  }
  EnableInterrupts;   //�����ж�
}

void UART3_ISR(void){}
void UART4_ISR(void)
{
    UARTn_e uratn = UART_4;//UART0 �жϷ�����
    if(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK)   //�������ݼĴ�����
    {
        //�û���Ҫ�����������
      uart_putchar(uratn,uart_getchar(uratn));//����

    }

    if(UART_S1_REG(UARTN[uratn]) & UART_S1_TDRE_MASK )  //�������ݼĴ�����
    {
        //�û���Ҫ����������
      uart_sendbuffer_isr(uratn);//���ͻ����жϴ�����

    }

}
/***********************************************************************************/
//PORTA~PORTE �˿��жϷ�����
void PORTA_ISR(void){}
void PORTB_ISR(void)
{

    uint8  n = 3;    //���ź�
    //PTB3
   
    if(PORTB_ISFR & (1 << n))           //PTE0�����ж�
    {
        PORTB_ISFR  |= (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */


        /*  ����Ϊ�û�����  */
    }
}
void PORTC_ISR(void){}

void PORTE_ISR(void)
{
}



/*!
 *  ��������	LPTMR_ISR
 *  ��  �ܣ�	LPTMR�жϷ�����
 *  ��  �أ�	void
 *  ʱ  �䣺	2018-12-21 20:17
 */
void LPTMR_ISR(){}



/*!
 *  ��������	DMA0_ISR
 *  ��  �ܣ�	DMA0�жϷ�����
 *  ��  �أ�	void
 *  ʱ  �䣺	2018-12-22 16:17
 */
void DMA0_ISR()
{

}


/*!
 *  ��������	FTM0_ISR
 *  ��  �ܣ�	FTM0�жϷ�����
 *  ��  �أ�	void
 *  ʱ  �䣺	2018-12-22 16:18
 */
void FTM0_ISR()
{
    uint8 s = FTM0_STATUS;             //��ȡ��׽�ͱȽ�״̬  All CHnF bits can be checked using only one read of STATUS.
    uint8 CHn;

    FTM0_STATUS = 0x00;             //���жϱ�־λ

    CHn = 0;
    if( s & (1 << CHn) )
    {
        //FTM_IRQ_DIS(FTM1, CHn);     //��ֹ���벶׽�ж�
        /*     �û�����       */

        /*********************/
        //FTM_IRQ_EN(FTM1, CHn); //�������벶׽�ж�

    }

    /* ������� n=1 ��ģ�棬����ģ������� */
    CHn = 1;
    if( s & (1 << CHn) )
    {
        //FTM_IRQ_EN(FTM1, CHn); //�������벶׽�ж�
        /*     �û�����       */


        /*********************/
        //�����������￪�����벶׽�ж�
        //FTM_IRQ_EN(FTM1, CHn); //�������벶׽�ж�
    }
}

/*!
 *  ��������	PIT0_ISR
 *  ��  �ܣ�	PIT0�жϷ�����
 *  ��  �أ�	void
 *  ʱ  �䣺	2018-12-22 16:45
 */




/*void PIT0_ISR()
{
    PIT_Flag_Clear(PIT0);//���жϱ�־λ 
    if(time_count == 5)
    {
      left_speed_in = (float)ftm_quad_get(FTM1);
      right_speed_in = -(float)ftm_quad_get(FTM2);
      pid_compute(pid1);//��
      pid_compute(pid2);//��
      PWMSetMotor2((s32)right_speed_out,(s32)left_speed_out); 
      //���ftm������
      ftm_quad_clean(FTM1);
      ftm_quad_clean(FTM2);
      time_count = 0;
    }
    time_count = time_count + 1;
    //����
    int adc1 = 0, adc2 = 0, adc3 = 0,adc4=0;
    adc1 = adc_once(ADC1_DM0, ADC_8bit);     //ͨ��drivers_cfg.h�ҵ�����E0��Ӧ��ADC��Դ ��ADC1_AD4a
    adc2 = adc_once(ADC0_DP3, ADC_8bit);
    adc3 = adc_once(ADC1_SE18, ADC_8bit);
    adc4 = adc_once(ADC1_SE16, ADC_8bit);
    direction_control(adc1+adc2 , adc3+adc4);    
}*/



void PIT0_ISR()
{
    PIT_Flag_Clear(PIT0);//���жϱ�־λ 
    float left_speed_in = (float)ftm_quad_get(FTM1);
    float right_speed_in = -(float)ftm_quad_get(FTM2);
    //uart_printf(UART_0,"in = %f %f\n", left_speed_in, right_speed_in);  
    
    pid_compute_new(left_speed_in,right_speed_in);  
    
    ftm_quad_clean(FTM1);
    ftm_quad_clean(FTM2);
        
}

void PIT1_ISR()
{
    PIT_Flag_Clear(PIT1);
    ele_direction_control();
    
    float left_speed_in = (float)ftm_quad_get(FTM1);
    float right_speed_in = -(float)ftm_quad_get(FTM2);
        
    pid_compute_new(left_speed_in,right_speed_in);         
    ftm_quad_clean(FTM1);
    ftm_quad_clean(FTM2);
}



#define TRIG1 PTE2 
#define ECHO1 PTE3
#define TRIG2 PTE8 
#define ECHO2 PTE9
//���������ģ�� 

uint8 sendflag=0;

uint8 leaveflagnum[10];
uint8 leaveroadflag=0;

uint8 continueflagnum[10];
uint8 continueflag=0;

uint16 distance1=0;
uint16 distance2=0;
int cntt1=0;
int cntt2=0;


void PIT2_ISR()
{
    PIT_Flag_Clear(PIT2);//���жϱ�־λ
    if(sendflag==0)//��δ����
    {
        gpio_set(TRIG1,1);
        gpio_set(TRIG2,1);
        cntt1=0;
        cntt2=0;
    }
    else{
      if(gpio_get(ECHO1)==1){cntt1++;}
      if(gpio_get(ECHO2)==1){cntt2++;}
      
      if(sendflag==1){
        gpio_set(TRIG1,0); 
        gpio_set(TRIG2,0); 
      }
      else if(sendflag==19){
        distance1=cntt1*340/2*2;
        distance2=cntt2*340/2*2;
        
       //uart_printf(UART_0,"%d\n",distance);
        
        
          //�������洢
          for(int i=0;i<9;i++)
          {
              leaveflagnum[i]=leaveflagnum[i+1];
          }
          if(distance1<700)
          {
              leaveflagnum[9]=1;
          }
          else
          {
              leaveflagnum[9]=0;
          }
          int sum=0;
          for(int i=0;i<10;i++)
            if(leaveflagnum[i]==1)
              sum++;
          if(sum>4)
          {
              leaveroadflag=1;
          }
              
          else
              leaveroadflag=0;
          
          
          //�������洢
          for(int i=0;i<9;i++)
          {
              leaveflagnum[i]=leaveflagnum[i+1];
          }
          if(distance2<700)
          {
              leaveflagnum[9]=1;
          }
          else
          {
              leaveflagnum[9]=0;
          }
          sum=0;
          for(int i=0;i<10;i++)
            if(leaveflagnum[i]==1)
              sum++;
          if(sum>4)
          {
              leaveroadflag=1;
          }
              
          else
              leaveroadflag=0;
      }
      
    }
    sendflag=(sendflag+1)%20;
}