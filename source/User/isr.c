//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4
/*!                    北京邮电大学 K66 学习例程
 *  文件名称：       isr.c
 *      作者：       maddevil
 *      说明：       仅做内部学习使用，请勿外传
 *  参考资料：       历届学长代码、山外K60库、龙邱K66模板、北邮KEA模板
 *    版本号：       V1.0.0
 *  最后更新：       2018-12-21 13:41
 */
//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4

#include "isr.h"

//UART0~UART4 串口中断服务函数
void UART2_ISR(void){}
void UART1_ISR(void){}

void UART0_ISR(void)
{    
  DisableInterrupts ;//关总中断
  char Data=0;
  UARTn_e uratn = UART_0;//UART0 中断服务函数
  if(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK)   //接收数据寄存器满
  {
    Data=uart_getchar(UART_0);
    signal_number = Data - '0';
    //setpoint1 = setpoint2 = -10;
    //my_steer_set(3050);
    //uart_putchar(UART_0,Data);
  }
  if(UART_S1_REG(UARTN[uratn]) & UART_S1_TDRE_MASK)  //发送数据寄存器空
  {
    //uart_putchar(UART_0,Data);
  }
  EnableInterrupts;   //开总中断
}

void UART3_ISR(void){}
void UART4_ISR(void)
{
    UARTn_e uratn = UART_4;//UART0 中断服务函数
    if(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK)   //接收数据寄存器满
    {
        //用户需要处理接收数据
      uart_putchar(uratn,uart_getchar(uratn));//回显

    }

    if(UART_S1_REG(UARTN[uratn]) & UART_S1_TDRE_MASK )  //发送数据寄存器空
    {
        //用户需要处理发送数据
      uart_sendbuffer_isr(uratn);//发送缓存中断处理函数

    }

}
/***********************************************************************************/
//PORTA~PORTE 端口中断服务函数
void PORTA_ISR(void){}
void PORTB_ISR(void)
{

    uint8  n = 3;    //引脚号
    //PTB3
   
    if(PORTB_ISFR & (1 << n))           //PTE0触发中断
    {
        PORTB_ISFR  |= (1 << n);        //写1清中断标志位

        /*  以下为用户任务  */


        /*  以上为用户任务  */
    }
}
void PORTC_ISR(void){}

void PORTE_ISR(void)
{
}



/*!
 *  函数名：	LPTMR_ISR
 *  功  能：	LPTMR中断服务函数
 *  返  回：	void
 *  时  间：	2018-12-21 20:17
 */
void LPTMR_ISR(){}



/*!
 *  函数名：	DMA0_ISR
 *  功  能：	DMA0中断服务函数
 *  返  回：	void
 *  时  间：	2018-12-22 16:17
 */
void DMA0_ISR()
{

}


/*!
 *  函数名：	FTM0_ISR
 *  功  能：	FTM0中断服务函数
 *  返  回：	void
 *  时  间：	2018-12-22 16:18
 */
void FTM0_ISR()
{
    uint8 s = FTM0_STATUS;             //读取捕捉和比较状态  All CHnF bits can be checked using only one read of STATUS.
    uint8 CHn;

    FTM0_STATUS = 0x00;             //清中断标志位

    CHn = 0;
    if( s & (1 << CHn) )
    {
        //FTM_IRQ_DIS(FTM1, CHn);     //禁止输入捕捉中断
        /*     用户任务       */

        /*********************/
        //FTM_IRQ_EN(FTM1, CHn); //开启输入捕捉中断

    }

    /* 这里添加 n=1 的模版，根据模版来添加 */
    CHn = 1;
    if( s & (1 << CHn) )
    {
        //FTM_IRQ_EN(FTM1, CHn); //开启输入捕捉中断
        /*     用户任务       */


        /*********************/
        //不建议在这里开启输入捕捉中断
        //FTM_IRQ_EN(FTM1, CHn); //开启输入捕捉中断
    }
}

/*!
 *  函数名：	PIT0_ISR
 *  功  能：	PIT0中断服务函数
 *  返  回：	void
 *  时  间：	2018-12-22 16:45
 */




/*void PIT0_ISR()
{
    PIT_Flag_Clear(PIT0);//清中断标志位 
    if(time_count == 5)
    {
      left_speed_in = (float)ftm_quad_get(FTM1);
      right_speed_in = -(float)ftm_quad_get(FTM2);
      pid_compute(pid1);//左
      pid_compute(pid2);//右
      PWMSetMotor2((s32)right_speed_out,(s32)left_speed_out); 
      //清空ftm计数器
      ftm_quad_clean(FTM1);
      ftm_quad_clean(FTM2);
      time_count = 0;
    }
    time_count = time_count + 1;
    //方向环
    int adc1 = 0, adc2 = 0, adc3 = 0,adc4=0;
    adc1 = adc_once(ADC1_DM0, ADC_8bit);     //通过drivers_cfg.h找到引脚E0对应的ADC资源 ：ADC1_AD4a
    adc2 = adc_once(ADC0_DP3, ADC_8bit);
    adc3 = adc_once(ADC1_SE18, ADC_8bit);
    adc4 = adc_once(ADC1_SE16, ADC_8bit);
    direction_control(adc1+adc2 , adc3+adc4);    
}*/



void PIT0_ISR()
{
    PIT_Flag_Clear(PIT0);//清中断标志位 
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
//超声波测距模块 

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
    PIT_Flag_Clear(PIT2);//清中断标志位
    if(sendflag==0)//尚未发送
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
        
        
          //缓冲区存储
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
          
          
          //缓冲区存储
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