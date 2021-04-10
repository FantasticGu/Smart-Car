#include "pid.h"
#include "fun.h"
#include "include.h"
//int begin_speed = 3000;//初始速度
int lasterror = 0;//上次误差
int accerror = 0;//累积误差
//方向环pid

#define test_port UART_0
#define NUMPIX 188
extern uint16 bigbendflag;

float direction_p = 4;
float direction_i = 0;
float direction_d = 18 ;


void pid_dynamic()
{
    if(bigbendflag==0)
    {
        direction_p = 1.1;
        direction_i = 0;
        direction_d = 7;
    }
    else if(bigbendflag==1)
    {
        direction_p = 2.2;
        direction_i = 0;
        direction_d = 18;
    }
    else if(bigbendflag==2)
    {
        direction_p = 5;
        direction_i = 0;
        direction_d = 36;
    }
    else if(bigbendflag==3)
    {
        direction_p = 8;
        direction_i = 0;
        direction_d = 40;
    }
    //uart_printf(test_port,"P:%d I:%d D:%d\n\n\n",direction_p,direction_i,direction_d);
    
}


//用不用改成增量pid？
void  direction_control(int error)
{
   pid_dynamic();
   int duty = 822;
   accerror = accerror + error;
   float pid = (float)(direction_p * error + direction_i * accerror + direction_d *(error - lasterror));
   lasterror = error;
   int s=duty + pid;
   if(s>922) s=922;
   if(s<722) s=722;
   ftm_pwm_duty(FTM3, FTM_CH1,s);
}



void direction_control_image(uint16 jiaodu,uint8 way)
{
  if(way==left_way)
  {
    //FTM1_C0V=jiaodu;//dj_center-(jiaodu-dj_center);
        ftm_pwm_duty(FTM3, FTM_CH1,jiaodu);
  }
  else if(way==right_way)
  {
        ftm_pwm_duty(FTM3, FTM_CH1,jiaodu);
  }
  else
  {  
    //FTM1_C0V=dj_center;//此处注意舵机数组修改
    //GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(2));//IO口输出亮高电平
    //GPIOC_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(10));//IO口输出亮高电平
	 //center_led();
    ftm_pwm_duty(FTM3, FTM_CH1,870);//正位置的pwm  ？885or 870
  } 
  //ftm_pwm_duty(FTM3, FTM_CH1,duty + pid);
}




//速度环PID
pid_t pid_create(pid_t pid, float* in, float* out, float* set, float kp, float ki, float kd, float omin, float omax)
{
	pid->input = in;
	pid->output = out;
	pid->setpoint = set;
        pid->Kp = kp;
        pid->Ki = ki;
        pid->Kd = kd;
        pid->omin = omin;
        pid->omax = omax;

	// Set default sample time to 100 ms

	return pid;
}

void  pid_compute(pid_t pid)
{
	float in = *(pid->input);
	// Compute error
	float error = (*(pid->setpoint)) - in;
	// Compute integral
	pid->iterm += error;
	if (pid->iterm > pid->omax)
		pid->iterm = pid->omax;
	else if (pid->iterm < pid->omin)
		pid->iterm = pid->omin;
	// Compute differential on input
	float dinput = in - pid->lastin;
	// Compute PID output
	float out = pid->Kp * error + pid->Ki *  pid->iterm - pid->Kd * dinput;
	// Apply limit to output value
	if (out > pid->omax)
		out = pid->omax;
	else if (out < pid->omin)
		out = pid->omin;
	// Output to pointed variable
	(*pid->output) = out;
	// Keep track of some variables for next execution
	pid->lastin = in;
}

float in1,out1,in2,out2,lastin1,lastin2=0;
float setpoint1=NORMAL_SPEED;
float setpoint2=NORMAL_SPEED;
float p1=7.0;
float p2=7.0;
float i1=0.5;
float i2=0.5;
float d1=0.3;
float d2=0.3;
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


      

      
    
//电磁控制参数
    

#define  AD_CH0                ADC0_SE16
#define  AD_CH1                ADC1_SE18
#define  g_HighestSpeed        100                      //编码器读回的定值
#define  g_LowestSpeed         0                       //编码器读回的定值


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

//方向环PID
    
void ele_direction_control()
{
  float value[120]=
{
    1,1,1,1,1,1,1,1,1,1,
    1.04,1.04,1.04,1.04,1.04,1.04,1.04,1.04,1.04,1.04,
    1.08,1.08,1.08,1.08,1.08,1.08,1.08,1.08,1.08,1.08,
    1.12,1.12,1.12,1.12,1.12,1.12,1.12,1.12,1.12,1.12,
    1.16,1.16,1.16,1.16,1.16,1.16,1.16,1.16,1.16,1.16,
    1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,1.2,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
};  
        int i = H-21;
        ADresult0 = 0;
        
        for(;i>H-41;i--)
        {
          ADresult0 += Pick_table[i];     
        }
        
        ADresult1 = NUMPIX*10;
        dir_error = (int)(ADresult0-ADresult1)/20;
        
	 
        //uart_printf(UART_0,"dir_error =  %d\n",dir_error);
       // uart_printf(UART_0,"line =  %d\n",30);
        
        
                
        if(-15>dir_error>-35)
        {
            dir_control_P=12;
            dir_P_value=dir_control_P*dir_error;
        }
        if(-8>=dir_error>=-15)
        {
            dir_control_P=10;
            dir_P_value=dir_control_P*dir_error;
        }
        if(-2>dir_error>-8)
        {
            dir_control_P=4;
            dir_P_value=dir_control_P*dir_error;
        } 
        else if(-2<=dir_error&&dir_error<=2)
        {
            dir_control_P=0;
            dir_P_value=dir_control_P*dir_error;
        }else if(2<dir_error<8)
        {
            dir_control_P=4;
            dir_P_value=dir_control_P*dir_error;
        }else if(15<=dir_error<=35)
        {
            dir_control_P=12;
            dir_P_value=dir_control_P*dir_error;
        }
        else if(8<dir_error<15)
        {
            dir_control_P=10;
            dir_P_value=dir_control_P*dir_error;
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





