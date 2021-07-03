#include "pid.h"
#include "fun.h"
#include "include.h"
//int begin_speed = 3000;//��ʼ�ٶ�
int lasterror = 0;//�ϴ����
int accerror = 0;//�ۻ����
//����pid

#define test_port UART_0
#define NUMPIX 188
extern uint16 bigbendflag;


float in1,out1,in2,out2,lastin1,lastin2=0;
float setpoint1=NORMAL_SPEED;
float setpoint2=NORMAL_SPEED;
float p1=10.0;
float p2=10.0;
float i1=0.0;
float i2=0.0;
float d1=0.1;
float d2=0.1;
float max_speed=7000;
float min_speed=-3000;
float iterm1=0;
float iterm2=0;


void speed_dynamic()
{

}

void pid_compute_new(float in1,float in2)
{ 
    float error1=setpoint1-in1;
    iterm1+=error1;
    float error2=setpoint2-in2;
    iterm2+=error2;
    
    if(iterm1>max_speed)
      iterm1=max_speed;
    else if(iterm1<min_speed)
      iterm1=min_speed;
    if(iterm2>max_speed)
      iterm2=max_speed;
    else if(iterm2<min_speed)
      iterm2=min_speed;
    
    float out1=p1*error1+i1*iterm1-d1*(in1-lastin1);
    float out2=p2*error2+i2*iterm2-d2*(in2-lastin2);
    
    if(out1>max_speed)
      out1=max_speed;
    else if(out1<min_speed)
      out1=min_speed;
    if(out2>max_speed)
      out2=max_speed;
    else if(out2<min_speed)
      out2=min_speed;
    
    lastin1=in1;
    lastin2=in2;
   // uart_printf(UART_0,"out = %f %f \n",out1,out2);
    
    PWMSetMotor2((s32)(out1),(s32)(out2)); 
}


      

      
    
//��ſ��Ʋ���
    

#define  AD_CH0                ADC0_SE16
#define  AD_CH1                ADC1_SE18
#define  g_HighestSpeed        100                      //���������صĶ�ֵ
#define  g_LowestSpeed         0                       //���������صĶ�ֵ


float  dir_control_P=0;                     // 4         2.7         7  
float  dir_control_D=0;          // 155        140        100
  
float     ADresult0=0,ADresult1=0;
int var[2];

int     dir_error=0,last_dir_error=0;
float   dir_P_value=0,dir_D_value=0;
float   g_fDirectionControlOut=0;

int     flag=0,lastflag=0,time=0;

int min(int a,int b)
{
  return a<b?a:b;
}

//����PID
    
void ele_direction_control()
{
        int i = H-21;
        ADresult0 = 0;
        
        for(;i>H-41;i--)
        {
          ADresult0 += Pick_table[i];     
        }
        
        ADresult1 = NUMPIX*10;
        dir_error = (int)(ADresult0-ADresult1)/20;
        
	 
        //uart_printf(UART_0,"dir_error =  %d\n",dir_error);

                       
        if(-15>dir_error>-35)
        {
            dir_control_P=12;
            dir_P_value=dir_control_P*dir_error;
            /*if(speedflag == 1)
            {
                setpoint1 = NORMAL_SPEED+10*dir_error;
                setpoint2 = NORMAL_SPEED;
            }*/
        }
        if(-8>=dir_error>=-15)
        {
            dir_control_P=10;
            dir_P_value=dir_control_P*dir_error;
            /*if(speedflag == 1)
            {
              setpoint1 = NORMAL_SPEED+8*dir_error;
                setpoint2 = NORMAL_SPEED;
            }*/
        }
        if(-2>dir_error>-8)
        {
            dir_control_P=4;
            dir_P_value=dir_control_P*dir_error;
            /*if(speedflag == 1)
            {
              setpoint1 = NORMAL_SPEED+6*dir_error;
                setpoint2 = NORMAL_SPEED;
            }*/
        } 
        else if(-2<=dir_error&&dir_error<=2)
        {
            dir_control_P=0;
            dir_P_value=dir_control_P*dir_error;
           /*if(speedflag == 1)
            {
              setpoint1 = setpoint2 = NORMAL_SPEED;
            }*/
        }else if(2<dir_error<8)
        {
            dir_control_P=4;
            dir_P_value=dir_control_P*dir_error;
           /*if(speedflag == 1)
            {
              setpoint2 = NORMAL_SPEED-6*dir_error;
                setpoint1 = NORMAL_SPEED;
            }*/
        }else if(15<=dir_error<=35)
        {
            dir_control_P=12;
            dir_P_value=dir_control_P*dir_error;
            /*if(speedflag == 1)
            {
              setpoint2 = NORMAL_SPEED-8*dir_error;
              setpoint1 = NORMAL_SPEED;
            }*/
        }
        else if(8<dir_error<15)
        {
            dir_control_P=10;
            dir_P_value=dir_control_P*dir_error;
            /*if(speedflag == 1)
            {
              setpoint2 = NORMAL_SPEED-10*dir_error;
              setpoint1 = NORMAL_SPEED;
            }*/
        }else
        {
          dir_control_P=20;
          dir_P_value=dir_control_P*dir_error;
          
        }
        
        
        if(dir_P_value<-150)dir_P_value=-150;
        if(dir_P_value>150)dir_P_value=150;
        
        //uart_printf(UART_0,"output = %f\n",dir_P_value);
        
        dir_control_D=0.1;
        dir_D_value=dir_control_D*(dir_error-last_dir_error);
        
        g_fDirectionControlOut=dir_P_value+dir_D_value; 
        last_dir_error=dir_error;

        
        ftm_pwm_duty(FTM3, FTM_CH1,(int)(990-g_fDirectionControlOut));
}





