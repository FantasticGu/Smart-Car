/*******************************************************************************
【平    台】龙邱K66FX智能车VD母板
【编    写】CHIUSIR
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2018年4月28日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR7.80.4及以上
【Target  】K66FX1M0VLQ18
【Crystal 】 50.000Mhz
【busclock】100.000MHz
【pllclock】200.000MHz
20180424采用新的DMA触发方式
******************************************************************************/
#include "include.h"
#define test_port UART_0

uint8 Image_Data[H][V];      //图像原始数据存放
uint8_t Image_Raw[IMAGEH][IMAGEW];
volatile u8 Image_Use[LCDH][LCDW]; //压缩后之后用于存放屏幕显示数据94*60
u8 Pixle[LCDH][LCDW];              //二值化后用于OLED显示的数据
uint8_t Threshold;                  //OSTU大津法计算的图像阈值
u8  Line_Cont=0;          //行计数
u8  Field_Over_Flag=0;    //场标识

int OFFSET0=0;      //最远处，赛道中心值综合偏移量
int OFFSET1=0;      //第二格
int OFFSET2=0;      //最近，第三格
int TXV=0;          //梯形的左高度，右高度




//摄像头图像采集中断处理函数
void PORTD_ISR(void)
{     
  //行中断PTD14
  /*if((PORTD_ISFR & 0x4000))//行中断 (1<<14)
  {    
    PORTD_ISFR |= 0x4000;  //清除中断标识
    // 用户程序            
    //DMA采集，在这里修改采集接口PTD_BYTE0_IN是D0--D7。   PLCK接的是PTD13。   
    //DMA_PORTx2BUFF_Init (DMA_CH4, (void *)&PTD_BYTE0_IN,(void*)Image_Data[Line_Cont], PTD13, DMA_BYTE1, IMAGEW, DMA_rising);  
    DMATransDataStart(DMA_CH4,(uint32_t)(&Image_Data[0][0]));   
    if(Line_Cont > H)  //采集结束
    { 
      Line_Cont=0; 
      return ;
    } 
    ++Line_Cont;            //行计数
    return ;
  }*/
  //场中断PTD15
  if((PORTD_ISFR & 0x8000))//(1<<15)
  {
    PORTD_ISFR |= 0x8000; //清除中断标识   
    DMATransDataStart(DMA_CH4,(uint32_t)(&Image_Data[0][0]));// 用户程序 
    Field_Over_Flag=1;     //场结束标识

  } 
}
//改动了取中线行，改动了调整，改动了,改动了摇头舵机频率
/*******************************************************************************
函数名称：get_vline
函数功能: 得到有效行
参数：无
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
函数名称：stable_control
函数功能: 数据处理控制程序
参数：无
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
       getBlineCenter();//得到中线 存在Pick_table[line]数组里面
       int lost_count = 0;//
       int Bline_right_all = 0;
       int Bline_left_all = 0;
       //get_vline();//根据有没有十字标志设置有效行
       //uart_printf(test_port,"%d\n",H - valid_line);
       /*{
	  positive_num=0;
	  decrease_num=0;
	  for(line=V-2;line>V-valid_line;line--)//V=188，不应该用H？Pick_table（记录赛道中央）
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
           get_LR_diff();//没看懂
           judgeRoad();//道路類型 有計數
           //lost_count();
           int cursion_all = 0;
           int lost_count=0;
           
           for(line = H-1; line > 0; line--)//从下向上看，如果连续4行为无效行，那么有效行就是下面这些
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
                //uart_printf(test_port,"%d左边线：%d\n",H-line,Bline_left[line]);
                //uart_printf(test_port,"%d右边线：%d\n",H-line,Bline_right[line]);
                //uart_printf(test_port,"%d中心线：%d\n",H-line,Pick_table[line]);
                //uart_printf(test_port,"%d单线差距：%d\n",H-line,Pick_table[line] - V/2);
                Bline_left_all += Bline_left[line];
                Bline_right_all += Bline_right[line];
                if(Pick_table[line] > 188 || Pick_table[line] < 0)
                {
                    Pick_table[line] = 94;
                }
                cursion_all = cursion_all + (Pick_table[line] - 94)*weight[line-60];//*矫正
                weight_sum+=weight[line-60];
                
                if((Pick_table[line] - Pick_table[line+1] > 30 || Pick_table[line+1] - Pick_table[line] > 30)&&(line < H-1))//如果这一行跳变
                {
                    valid_line = H - line;//有效行就是从最下面找到跳变这一行
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
           //uart_printf(test_port,"左平均:%d\n",Bline_left_avg);
           //uart_printf(test_port,"右平均:%d\n",Bline_right_avg);
           }
           if(Bline_right_avg == 188 && Bline_right_avg < 50 && valid_line>25){
              cursion_avg = 16;
              shizi_right = 10;
              change = 1;
           //uart_printf(test_port,"左平均:%d\n",Bline_left_avg);
           //uart_printf(test_port,"右平均:%d\n",Bline_right_avg);
           }*/
           /*if(shizi_left>0&&change == 0){
              cursion_avg = -8;
              shizi_left = shizi_left-1;
           }
           if(shizi_right>0&&change == 0){
              cursion_avg = 8;
              shizi_right = shizi_right-1;
           }*/ //十字定时器代码
           
           int Shizi_left = 0, Shizi_right = 0;
           int down = 0, up = 0;
           
           
           //判断左侧或右侧是否看不到赛道边缘
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
                     if(Bline_right[shizi_i] != V-1 )//？V-1
                     {
                        Shizi_right = 0;
                        //uart_printf(test_port,"222\n");
                     }
                }
           }
           
           //根据上面的标志结果 ，如果左侧看不到，右侧看得到赛道边缘
           
           if(Shizi_left && !Shizi_right)//左侧不可见 右侧可见
           {
                for(int shizi_i = H-1; ; shizi_i--)//从下面往上开始找
                {
                   if(Bline_right[shizi_i] <= Bline_right[shizi_i+1] && shizi_i < H-1 && up == 0)//右侧边线 左拐
                      down++;
                   
                   else if(Bline_right[shizi_i] >= Bline_right[shizi_i+1] && shizi_i < H-1)//右侧边线右拐
                      up++;
                   
                   if(Bline_left[shizi_i] != 0)//发现左侧出现赛道时，退出
                      break;
                }
           }
           
           else if(Shizi_right && !Shizi_left)//左侧可见 右侧不可见
           {
                for(int shizi_i = H-1; ; shizi_i--)
                {
                   if(Bline_left[shizi_i] >= Bline_left[shizi_i+1] && shizi_i < H-1 && up == 0)//左侧边线 右拐
                      down++;
                   
                   else if(Bline_left[shizi_i] <= Bline_left[shizi_i+1] && shizi_i < H-1)//左侧边线 左拐
                      up++;
                   
                   if(Bline_right[shizi_i] != 0)//发现右侧出现赛道时，推出
                      break;
                }
           }
           
           if(up >= 15 && down >= 15)//有点迷糊，什么情况下up|down都大
           {
                 if(Shizi_left && !Shizi_right)//左侧不可见 右侧可见时
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
           else if(cursion_avg >= 6)//测出来的不转范围，要是过分抖动或者弯道不拐弯，改这里。
           {
              //uart_printf(test_port,"222\n");
              direction_control(-cursion_avg);
           }
           else//如果偏差较小
           {
              //uart_printf(test_port,"333\n");
              //uart_printf(test_port,"差距平均:%d\n",cursion_avg);
              duoji_control(dj_center,center_way);//宏定义dj_center885，是正位置.有点问题870咋回事?
           } 
           direction_control(-cursion_avg);//输入error,进行pid控制，
           
           //uart_printf(test_port,"左持续:%d\n",shizi_left);
           //uart_printf(test_port,"右持续:%d\n",shizi_right);
           //uart_printf(test_port,"右平均:%d\n",Bline_right_avg);
           //uart_printf(test_port,"有效行:%d\n",valid_line);
           //uart_printf(test_port,"总偏差:%d\n",cursion_all);
           //uart_printf(test_port,"差距平均:%d\n",cursion_avg);
           //uart_printf(test_port,"角度:%d\n\n",jiaodu_num);
           
           last_turn=jiaodu_num;
           last_pick_way=pick_way;
           change = 0;
       }
       run_time++;
}


/*******************************************************************************
函数名称：stable_del
函数功能: 以前的数据处理控制程序
参数：无
*******************************************************************************/
void stable_del()
{
  	uint8 line;
////////////////////////////////////////////////////////////////////////////////
	//buzzer_ctl();
	get_vline();//得到有效行
        //uart_printf(test_port, "有效行：%d\n",valid_line);
	//xielv_lvbo();//跳变差值限制法滤波 去除无效行
	//lvbo(5);
   	//ti_jiaozheng();
	bDistance();//赛道宽度法滤波 去除无效行
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
		  //GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(17));/// 无效图像指示灯亮 LED4
		  jiaodu_num=last_turn;
		  slow_down_num=1;
		  //find_shizi(valid_line);
		  way_control(); //这里面的duoji_control要改
		  /*if(start_end_flag==1)//如果发现起跑线标志，则停车
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
		    //GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(17));	//IO口输出亮高电平 LED4
			//find_s();
			//area=get_area(Pick_table[5],Pick_table[(valid_line)/2],Pick_table[valid_line],5,(valid_line)/2,valid_line);
			//lcd_int(8,"Xl:",(int)area);
                        //uart_printf(test_port, "1");
			get_LR_diff();
                        //uart_printf(test_port, "2");
			//If_straight();//判断是否为直道
			judgeRoad();//判断赛道类型
                        //uart_printf(test_port, "3");
			//zhidao_count_flag=0;
			lost_count();//计算丢失行
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
/////////////////////////////////速度控制/////////////////////////////////
			/*if(start_end_flag==1)//如果发现起跑线标志，则停车
			{
			  	stop_car();
			}
			else*/
			//if(FRONT>1500&&Bmq<5)//防撞车 最后得去除
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
函数名称：clearDelPar
函数功能: 清零处理参数
参数：无
*******************************************************************************/
void clearDelPar()
{
 	startline_F=0;//发现起始行标志清0
 	//endline_F=0;//发现结束行标志清0
	//PickCenter_flag=0;//清零图像开始寻黑线标志
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
* 函数名称：SendPicture 
* 功能说明：发送图像到上位机 ，不同的上位机注意修改对应的数据接收协议
* 参数说明： 
* 函数返回：void 
* 修改时间：2018年3月27日 
* 备 注： 
***************************************************************/ 
void UARTSendPicture(uint8_t tmImage[H][V]) 
{ 
  int i = 0, j = 0; 
  uint8_t threshhold = 40;
  uart_putchar(UART_0,0xAB); //发送帧头标志 DEMOK上位机  
  uart_putchar(UART_0,0xAA); //发送帧头标志 DEMOK上位机  
  uart_putchar(UART_0,0xAB); //发送帧头标志 DEMOK上位机  
  uart_putchar(UART_0,0xAA); //发送帧头标志 DEMOK上位机  
  uart_putchar(UART_0,0xAB); //发送帧头标志 DEMOK上位机  
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
*                    北京龙邱智能科技 
*
*  函数名称：void SendPicture()
*  功能说明：摄像头数据发送
*  参数说明：无
*  函数返回：无
*  修改时间：
*  备    注：
*************************************************************************/
/*void SendPicture(void)
{
  int i = 0, j = 0;
  uart_putchar(UART_0,0xff);//发送帧头标志
  for(i=0;i<IMAGEH;i++)      //输出
  {
    for(j=0;j<IMAGEW;j++)    //输出从第0列到列，用户可以选择性的输出合适的列数
    {
      if(Image_Data[i][j]==0xff)
      {
        Image_Data[i][j]=0xfe;//防止发送标志位
      }
      uart_putchar(UART_0,Image_Data[i][j]);
    }
  }
}*/
void Test_Print_Road(void)
{
  for(int tmp_line = 0; tmp_line < H; tmp_line++){
                                  uart_printf(test_port, "黑线左%d行:%d\n",tmp_line+1,Bline_left[tmp_line]);
                                  uart_printf(test_port, "黑线右%d行:%d\n",tmp_line+1,Bline_right[tmp_line]);
                                  uart_printf(test_port, "中心线%d行：%d\n",tmp_line+1,Pick_table[tmp_line]);
                                  //uart_printf(test_port, "第%d行:", tmp_line);
                                  //for(int tmp_row = 0; tmp_row < V; tmp_row++){
                                     //uart_printf(test_port, "%d ",(int)Image_Data[tmp_line][tmp_row]);
                                  //}
                                  //uart_printf(test_port,"\n");
    }
}
void Test_Print_Vline(void)
{
  uart_printf(test_port, "最大有效行：%d\n",valid_line);
}
void Test_Print_JudgeRoad(void)
{
  uart_printf(test_port, "最大有效行：%d\n",valid_line);
  uart_printf(test_port, "小S标志：%d\n",ls_flag);
  uart_printf(test_port, "方向变量：%d\n",pick_way);
  uart_printf(test_port, "直道判断标志：%d\n",zhidao_count_flag);
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

  //uart_putchar(UART_0,0xff);//发送帧头标志
  for(i=0;i<IMAGEH;i++)      //输出
  {
    for(j=0;j<IMAGEW;j++)    //输出从第0列到列，用户可以选择性的输出合适的列数
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
  for(i=0;i<IMAGEH;i++)      //输出
  {
    for(j=0;j<IMAGEW;j++)    //输出从第0列到列，用户可以选择性的输出合适的列数
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


void imagineProcess(void)
{
 

  //setbinary();
    
  while(1)
  {  
       if(Field_Over_Flag)    //完成一场图像采集后显示并发送数据到上位机
      {
          //exti_disable(PTD14); //行中断关闭
          exti_disable(PTD15); //场中断关闭 
          
          clearDelPar();//清零处理参数
          
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
          
          if(judgeRight(31,41,180) == 1 && judgeRight(16,26,170) == 0)
          {
            int p1 = 0;
            int p2 = 0;
            int p3 = 0;
            for(int j = H-11;j>H-14;j--)
            {
              if (Bline_left[j]>15)
              {
                 p1 = Bline_left[j];
                 break;
              }
              
            } 
            for(int j = H-19;j>H-22;j--)
            {
              if (Bline_left[j]>20)
              {
                 p2 = Bline_left[j];
                 break;
              }
              
            } 
            for(int j = H-26;j>H-30;j--)
            {
              if (Bline_left[j]>25)
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
            static int cnt_1 = 0;
            cnt_1++;
            for(int i = H-11;i>H-41;i--)
            {
              Pick_table[i] = Bline_left[i]+60;
            }
            if(judgeRight(35,45,165) == 0)
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
            if(judgeRight(25,35,180)==1)
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
              if(cnt_3 > 10)
              {
                cnt_3 = 0;
                rstatus = 4;
                
              }
          }
          
          if(rstatus == 4)
          {
            static int cnt_4 = 0;
                cnt_4++;
                if(cnt_4>50)
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
            if(cnt_6 > 20 && judgeLeft(25,35,15) == 1)
            {
              cnt_6 = 0;
              rstatus = 7;
            }
            
          }
                    
          /*if(rstatus == 6)
          {
            static int cnt_6 = 0;
            cnt_6++;
            if(cnt_6>10)
            {
              if(judgeRight(25,35,180) == 1)
              {
                cnt_6 = 0;
                rstatus = 7;
              }
            }
            
          }*/
              
          if(rstatus == 7)
          {
            static int cnt_7 = 0;
            cnt_7 ++;
            for(int i = H-11;i>H-41;i--)
            {
              Pick_table[i] = Bline_left[i]+50;
            }
            if(cnt_7 > 30)
            {
              cnt_7 = 0;
              rstatus = 0;
            }
          }
              
              
              if(judgeLeft(31,41,8) == 0 && judgeLeft(16,26,18) == 1)
              {
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
                  if (Bline_right[j]<163)
                  {
                     p2 = Bline_right[j];
                     break;
                  }
                  
                } 
                for(int j = H-26;j>H-30;j--)
                {
                  if (Bline_right[j]<158)
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
                static int cnt_1 = 0;
                cnt_1++;
                if(cnt_1 > 20)
                {
                  cnt_1 = 0;
                  lstatus = 0;
                }
                if(judgeLeft(35,45,30) == 1)
                {
                  cnt_1 = 0;
                  lstatus = 2;
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
                  Pick_table[i] = Bline_left[i]+50;
                }
                if(cnt_3 > 20)
                {
                    cnt_3 = 0;
                    lstatus = 4;
                }
              }
              
              if(lstatus==4)
              {
                static int cnt_4 = 0;
                cnt_4++;
                if(cnt_4>50)
                {

                  if(judgeRight(15,25,180) == 1)
                  {
                    cnt_4 = 0;
                    PWMSetSteer(1150);
                    delayms(400);
                    lstatus = 5;
                  }   
                }
              }
              if(lstatus == 5)
              {
                if(judgeRight(35,45,150)==0)
                {
                  lstatus = 6;
                }
              }
              if(lstatus == 6)
              {
                static int cnt_6 = 0;
                cnt_6++;
                if(cnt_6>10)
                {
                  if(judgeLeft(25,35,15) == 0)
                  {
                    cnt_6 = 0;
                    lstatus = 7;
                  }
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
                if(cnt_7 > 30)
                {
                  cnt_7 = 0;
                  lstatus = 0;
                }
              }
            
            
           if(judgeLeft(15,35,5)==0 && judgeRight(15,35,183)==1 && !lstatus && !rstatus)
           {
             PWMSetSteer(990);
             delayms(500);
             
           }
            
      
          //uart_printf(UART_0,"right = %d\n",readyright);
  
          ele_direction_control();
          
          exti_enable(PTD15,IRQ_FALLING|PULLUP);    //场中断 
          Field_Over_Flag= 0; 
   
     }
  

  }
}



//测试主函数
void Test_LQV034(void)
{  
  
  //LCD_Show_Frame94();      //画图像 LCDW*LCDH 外框
  //uart_printf(test_port, "最大有效行\n");
  LQMT9V034_Init();        //摄像头初始化
  //uart_printf(test_port, "最大有效行2\n");
  int line = 0;
  /*while(1){
    if(Field_Over_Flag){
       //串口发送数据非常慢，注释掉OLED刷新很快
      exti_disable(PTD14); //行中断关闭
      exti_disable(PTD15); //场中断关闭  
      UARTSendPicture(Image_Data);     //发送数据到上位机，注意协议格式，不同的上位机看原函数对应修改，//不使用时请关闭
      exti_enable(PTD14,IRQ_RISING|PULLDOWN);   //行中断
      exti_enable(PTD15,IRQ_FALLING|PULLUP);    //场中断 
      
      Get_Use_Image();     //采集图像数据存放数组
      Get_01_Value();      //二值化图像数据
                  
      //Threshold = GetOSTU(Image_Data);   //OSTU大津法 获取全局阈值
      //BinaryImage(Image_Data,Threshold); //二值化图像数据
      //Pixle_Filter();
      //TFTSPI_Show_Pic3(18, 8, 94, 60, Pixle); //龙邱TFT1.8显示二值化图像
      //Draw_Road();                         //龙邱OLED模块显示动态图像
      Field_Over_Flag= 0;
    }
  }*/
  while(1)
  { 
    //LED_Ctrl(LED1, RVS);   //LED指示程序运行状态
    if(Field_Over_Flag)    //完成一场图像采集后显示并发送数据到上位机
    {
      //串口发送数据非常慢，注释掉OLED刷新很快
      exti_disable(PTD14); //行中断关闭
      exti_disable(PTD15); //场中断关闭  
      //uart_printf(test_port, "111最大有效行：%d\n",valid_line);
      //SendPicture();
      //UARTSendPicture(Image_Data);     //发送数据到上位机，注意协议格式，不同的上位机看原函数对应修改，//不使用时请关闭
      //LED_Ctrl(LED1, RVS);       //LED指示程序运行状态
      
    	  ///////////////////////////////图像采集部分///////////////////////////////
                        //uart_printf(test_port, "最大有效行：%d\n",valid_line);
      
                        clearDelPar();//清零处理参数
                          //Binary_line(line);//二值化该采集行
                        
                        find_edge();
                        valid_line=GetValidLine();
                         
                         
			
                        
                        
                        
                        
			for(line=H-1;line>=0;line--) //提取各行中心点并处理意外
			{
				row_F[line]=0;//清除采集完成标志位
				
				if(line==H-4)
				{
					//PickCenter_near(); 
				  	Near_lost=PickCenter_near_advance();
                                        //uart_printf(test_port, "最大有效行：%d\n",valid_line);
				  	if(Near_lost==1)//如果只发现丢了一行
					{
					  Far_find_flag=FAR_FIND_MIDDLE;
					  Shi_zi_line=4;
					}
				}
                              
				if(line<=H-5)//最下面四行往上
				{
                                          PickCenter_diff_advance(line);
                                          if(lost_w_count>2)
                                          {
                                                
                                          }
				  
				}
////////////////////////////////十字判别式///////////////////////////////////////
				if((Pick_flag[line]&ALL_LOST_W))//两边都找不到线且寻到为全白行数统计
				{
                                        lost_w_count++;
                                        Shi_zi_flag=1;
                                      //buzzer_on();
				}
				if(Shi_zi_flag==0&&lost_already==0)//在有效行丢失之前判断丢白线数判断十字道
				{
                                        if(lost_w_count>5)
                                        { 
                                           Shi_zi_flag=1;
					  //buzzer_on();
                                         }
				}
    
                                //uart_printf(test_port, "最大有效行：%d\n",valid_line);

///////////////////////////有效行判断///////////////////////////////////////这里面B结尾的标志位在哪里改变的？？？
                                
				if((Pick_flag[line]&LEFT_LOST_B)||(Pick_flag[line]&RIGHT_LOST_B)||((Bline_left[line]-Bline_right[line]<5)&&(Bline_left[line]-Bline_right[line]>-5)))
				{
				  if((Pick_flag[line-1]&LEFT_LOST_B)||(Pick_flag[line-1]&RIGHT_LOST_B)||((Bline_left[line]-Bline_right[line]<5)&&(Bline_left[line]-Bline_right[line]>-5)))
					lost_b_count++;
				}
                                //Bline_diff是左右线之间的差距
				if(line>H-25)
				{
				  if(maxBline_diff<Bline_diff)
				  	maxBline_diff=Bline_diff;
				}
                                //uart_printf(test_port, "最大有效行：%d\n",valid_line);
				if(lost_already==0)
				{
					if(lost_b_count>3||(Bline_left[line]<5)||(Bline_right[line]>(V-5))||((Bline_left[line]-Bline_right[line]<5)&&(Bline_left[line]-Bline_right[line]>-5)))
					{
                                                //uart_printf(test_port, "丢线%d行：%d\n",line,Bline_right[line]);
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
	  //////////////////////////////图像处理及控制部分//////////////////////////
			 //uart_printf(test_port, "最大有效行：%d\n",valid_line);
                         valid_line=IMAGEH-Lost_Line_count;
          
                         stable_control();
    
                         Lost_Line_count = 0;
      //uart_printf(test_port, "111最大有效行：%d\n",valid_line);
      exti_enable(PTD14,IRQ_RISING|PULLDOWN);   //行中断
      exti_enable(PTD15,IRQ_FALLING|PULLUP);    //场中断 
      
      //Get_Use_Image();     //采集图像数据存放数组
      //Get_01_Value();      //二值化图像数据     
      //Threshold = GetOSTU(Image_Data);   //OSTU大津法 获取全局阈值
      //BinaryImage(Image_Data,Threshold); //二值化图像数据
      //Pixle_Filter();
      //TFTSPI_Show_Pic3(18, 8, 94, 60, Pixle); //龙邱TFT1.8显示二值化图像
      //Draw_Road();                         //龙邱OLED模块显示动态图像
      Field_Over_Flag= 0;       
    }    
  }
}

// MT9V034 Port Init
void LQMT9V034_Init(void)
{     
  uint16_t data = 0;  
  
  //摄像头寄存器设置
  //TFTSPI_CLS(u16BLUE);
  SCCB_Init();                     //两个地址口都拉高，为MT9V034_I2C_ADDR  
  if(SCCB_RegRead(MT9V034_I2C_ADDR>>1,MT9V034_CHIP_VERSION,&data) == 0)//读取摄像头版本寄存器 
  {     
    if(data != MT9V034_CHIP_ID)                                  //芯片ID不正确，说明没有正确读取导数据，等待      
    { 
      //TFTSPI_P8X8Str(0,0,(u8*)"034 dpi!!", u16RED, u16BLUE);                       //摄像头识别失败，停止运行
      //uart_printf(UART_0, "error,id = %d!\n",data); 
      while(1)
      {
        uart_printf(test_port, "一\n");
      } 
    } 
    else                                                   //芯片ID正确
    {
      //TFTSPI_P8X8Str(0,0,(u8*)"V034 OK", u16RED, u16BLUE);
      //uart_printf(UART_0, "correct!\n"); 
    }
  } 
  else 
  { 
    while(1){
      uart_printf(test_port, "二\n"); 
      } ; //摄像头识别失败，停止运行
  }  

  MT9V034_SetFrameResolution(IMAGEH, IMAGEW);//设置摄像头图像4*4分频输出PCLK, 27/4 = 6.75M ,BIT4,5镜像设置:上下左右均镜像   
  MT9V034_SetAutoExposure(1);  
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0xAC, 0x0001);
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0xAD, 0x01E0);  
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x2C, 0x0004);
  
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x7F, 0x3000);           // test pattern

  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ANALOG_CTRL, MT9V034_ANTI_ECLIPSE_ENABLE);
  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x0F, 0x0000);  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_HDR_ENABLE_REG, 0x0101); // 0x0F bit8:1HDR,0linear; bit1:1彩色,0灰度;bit0:1HDR,0linear
  //MT9V034_WriteReg16(MT9V034_HDR_ENABLE_REG, 0x0103);     // 0x0F bit8:1HDR,0linear; bit1:1彩色,0灰度;bit0:1HDR,0linear
  //0x07 Chip Control bit2-0:0逐行扫描,1无效，2场，3单场；bit5:1立体视觉使能,bit7:1并口使能；bit8:1同步模式;bit9:1正常操作；bit15:0A/1B
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_CHIP_CONTROL, 0x0188);          //Context A  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ROW_NOISE_CORR_CTRL_REG, 0);   //0x70  0x0000 
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE_REG, 0x0303);   //0xAF  AEC/AGC A~bit0:1AE;bit1:1AG/B~bit2:1AE;bit3:1AG
  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MIN_EXPOSURE_REG, 0x0380);     //0xAC  最小粗快门宽度   0x0001
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_EXPOSURE_REG, 0x0480);     //0xAD  最大醋快门宽度   0x01E0-480
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_GAIN_REG, 60);             //0xAB  最大模拟增益     64
  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AGC_AEC_PIXEL_COUNT_REG, 188*120);//0xB0  用于AEC/AGC直方图像素数目,最大44000   4096=320*240  
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ADC_RES_CTRL_REG, 0x0303);     //0x1C  here is the way to regulate darkness :)    
  
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x13,0x2D2E);//We also recommended using R0x13 = 0x2D2E with this setting for better column FPN.  
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x20,0x03C7);//0x01C7对比度差，发白；0x03C7对比度提高 Recommended by design to improve performance in HDR mode and when frame rate is low.
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x24,0x0010);//Corrects pixel negative dark offset when global reset in R0x20[9] is enabled.
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x2B,0x0003);//Improves column FPN.
  SCCB_RegWrite(MT9V034_I2C_ADDR,0x2F,0x0003);//Improves FPN at near-saturation.  
  
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x08, 0x03D4);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x09, 0x03E7);
  //100DB //设置曝光时间
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_SHUTTER_WIDTH1,0x01BB);        //0x08 Coarse Shutter IMAGEW 1
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_SHUTTER_WIDTH2,0x01D9);        //0x09 Coarse Shutter IMAGEW 2
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_SHUTTER_WIDTH_CONTROL,0x0064); //0x0A Coarse Shutter IMAGEW Control 
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_V2_CTRL_REG_A, 0x001A);        //0x32   0x001A  
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_TOTAL_SHUTTER_WIDTH,0x0100);   //0x0B Coarse Shutter IMAGEW Total
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_HDR_ENABLE_REG,0x0103);        //0x0F High Dynamic Range enable,bit is set (R0x0F[1]=1), the sensor uses black level correction values from one green plane, which are applied to all colors.
  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AGC_AEC_DESIRED_BIN_REG, 64); //0xA5  图像亮度  50  1-64
  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_ANALOG_GAIN,0x8010);           //0x35
  
  //80dB HDR
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x08, 0x03CA);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x09, 0x03DE);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x0A, 0x0064);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x0B, 0x03E8);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x0F, 0x0103);
  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x35, 0x8010);   

  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_RESET, 0x03);          //0x0c  复位 
  
  //GPIO口初始化
  exti_enable(PTD14,IRQ_RISING|PULLDOWN);   //行中断
  exti_enable(PTD15,IRQ_FALLING|PULLUP);    //场中断 
  gpio_init(PTD0,GPI,0);          //八位数据输入口      
  gpio_init(PTD1,GPI,0);  
  gpio_init(PTD2,GPI,0);
  gpio_init(PTD3,GPI,0);
  gpio_init(PTD4,GPI,0);
  gpio_init(PTD5,GPI,0);
  gpio_init(PTD6,GPI,0);
  gpio_init(PTD7,GPI,0);
  //初始化DMA采集  
  dma_portx2buff_init(DMA_CH4, (void *)&PTD_BYTE0_IN,(void*)Image_Data, PTD13, DMA_BYTE1, IMAGEW*IMAGEH, DMA_RISING);  
}     

void setbinary()
{
    
    if(Field_Over_Flag)    //完成一场图像采集后显示并发送数据到上位机
      {
          exti_disable(PTD14); //行中断关闭
          exti_disable(PTD15); //场中断关闭  
          
          uint8_t binary=GetOSTU(Image_Data);
          Cmp=binary;
    
          exti_enable(PTD14,IRQ_RISING|PULLDOWN);   //行中断
          exti_enable(PTD15,IRQ_FALLING|PULLUP);    //场中断 
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
  
  //设置图像输出大小为120*188，不损失视野，同时满足智能车需要
  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x0D,0x030A);   //设置摄像头图像4*4分频输出PCLK, 27/4 = 6.75M ,BIT4,5镜像设置 
  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x0D,0x033A);   //设置摄像头图像4*4分频输出PCLK, 27/4 = 6.75M ,BIT4,5镜像设置:上下左右均镜像 
  
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


// 获取需要的图像数据
__ramfunc void Get_Use_Image(void)
{
  int i = 0,j = 0,row = 0,line = 0;
  
  for(i = 0; i  < IMAGEH; i+=2)  //120行，每2行采集一行，
  {
    for(j = 0;j < IMAGEW; j+=2)  //188，
    {        
      Image_Use[row][line] = Image_Data[i][j];         
      line++;        
    }      
    line = 0;
    row++;      
  }  
}

//按照均值的比例进行二值化
void Get_01_Value(void)
{
  int i = 0,j = 0;
  u8 GaveValue;
  u32 tv=0;
  char txt[16];
  
  //累加
  for(i = 0; i <LCDH; i++)
  {    
    for(j = 0; j <LCDW; j++)
    {                            
      tv+=Image_Use[i][j];   //累加  
    } 
  }
  GaveValue=tv/LCDH/LCDW;     //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100 
  sprintf(txt,"%03d:%03d",Threshold,GaveValue);//前者为大津法求得的阈值，后者为平均值  
  //TFTSPI_P8X8Str(50,0,(u8*)txt, u16RED, u16BLUE);
  //按照均值的比例进行二值化
  GaveValue=GaveValue*7/10+10;        //此处阈值设置，根据环境的光线来设定 
  for(i = 0; i < LCDH; i++)
  {
    for(j = 0; j < LCDW; j++)
    {                                
      //if(Image_Use[i][j] >GaveValue)//平均值阈值
      if(Image_Use[i][j] >Threshold) //大津法阈值   数值越大，显示的内容越多，较浅的图像也能显示出来    
        Pixle[i][j] =1;        
      else                                        
        Pixle[i][j] =0;
    }    
  }
}
//三面以上反数围绕清除噪点
void Pixle_Filter(void)
{  
  int nr; //行
  int nc; //列
  
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
*  函数名称：int Seek_Road(void)                                           *
*  功能说明：寻找白色区域偏差值                                            *
*  参数说明：无                                                            *
*  函数返回：值的大小                                                      *
*  修改时间：2017-07-16                                                    *
*  备    注：以中间为0，左侧减一，右侧加一，数值代表1的面积                *
*            计算区域从第一行开始到倒数第二行结束。                        *
*            如果面积为负数，数值越大说明越偏左边；                        *
*            如果面积为正数，数值越大说明越偏右边。                        *
***************************************************************************/ 
void Seek_Road(void)
{  
  int nr; //行
  int nc; //列
  int temp=0;//临时数值
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
  int nr; //行
  int nc; //列     
  
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
        zb[nr]=nc;//左边沿，越来越大
      }
      if((Pixle[nr+8][nc-1]==1)&&(Pixle[nr+8][nc]==1)&&(Pixle[nr+8][nc+1]==0)&&(Pixle[nr+8][nc+2]==0))
      {
        yb[nr]=nc;//右边沿，越来越小
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
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Init(void)
*  功能说明：配置SCCB所用引脚为GPIO功能，暂时不配置数据方向
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Init(void)
{
  gpio_init(PTB2,GPO,0);//配置为GPIO功能
  gpio_init(PTB3,GPO,0);//配置为GPIO功能  
}

/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Wait(void)
*  功能说明：SCCB等待演示
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
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
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Star(void)
*  功能说明：启动函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
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
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Stop(void)
*  功能说明：停止函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
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
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：uint8 SCCB_SendByte(uint8 Data)
*  功能说明：SCCB发送字节函数
*  参数说明：要发送的字节
*  函数返回：应答信号
*  修改时间：2017年12月5日
*  备    注：
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
* 函数名称：uint8 SCCB_ReadByte(void) 
* 功能说明：SCCB读取字节函数 
* 参数说明： 
* 函数返回：读取字节 
* 修改时间：2017年12月5日 
* 备 注： 
***************************************************************/ 
uint8 SCCB_ReadByte(void) 
{ 
  uint8 i; 
  uint8_t byte; 
  SCL_OUT(); 
  SDA_IN(); //使能输入
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
* 函数名称：static void SCCB_Ack(void) 
* 功能说明：IIC有回复信号 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
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
* 函数名称：static void SCCB_NAck(void) 
* 功能说明：IIC无回复信号 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
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
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_RegWrite(uint8 Device,uint8 Address,uint16 Data)
*  功能说明：向设备写数据
*  参数说明：要发送的字节
*  函数返回：应答信号
*  修改时间：2017年12月5日
*  备    注：
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
* 函数名称：uint8_t SCB_RegRead(uint8_t Device,uint8_t Address,uint16_t *Data) 
* 功能说明：读取数据 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
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
* 函数名称：int SCCB_Probe(uint8_t chipAddr) 
* 功能说明：查询该地址的设备是否存在 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
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
* 函数名称：SendPicture 
* 功能说明：发送图像到上位机 ，不同的上位机注意修改对应的数据接收协议
* 参数说明： 
* 函数返回：void 
* 修改时间：2018年3月27日 
* 备 注： 
***************************************************************/ 
void UARTSendPicture2(uint8_t tmImage[IMAGEH][IMAGEW]) 
{ 
  int i = 0, j = 0; 
  uart_putchar(UART_0,0xFE); //发送帧头标志 WindowsFormsApplication1.exe
  uart_putchar(UART_0,0xEF); //发送帧头标志 WindowsFormsApplication1.exe
  for(i=0;i < IMAGEH; i++) 
  { 
    for(j=0;j <IMAGEW;j++) 
    { 
      if(tmImage[i][j]==0xff) 
      { 
        tmImage[i][j]=0xfe; //防止发送标志位 
      } 
      uart_putchar(UART_0,tmImage[i][j]); 
    } 
  }
  uart_putchar(UART_0,0xEF); //发送帧尾标志 
  uart_putchar(UART_0,0xFE); //发送帧尾标志 
} 

/*void UARTSendPicture(uint8_t tmImage[IMAGEH][IMAGEW]) 
{ 
  int i = 0, j = 0; 
  uart_putchar(UART_0,0xFF); //发送帧头标志 DEMOK上位机  
  for(i=0;i < IMAGEH; i++) 
  { 
    for(j=0;j <IMAGEW;j++) 
    { 
      if(tmImage[i][j]==0xFF) 
      { 
        tmImage[i][j]=0xFE; //防止发送标志位 
      } 
      uart_putchar(UART_0,tmImage[i][j]); 
    } 
  }
} */
/*
void SendPicture(void)
{
  int i = 0, j = 0;
  UART_Put_Char(UART_4,0xff);//发送帧头标志
  for(i=0;i<Frame_Height;i++)      //输出
  {
    for(j=0;j<Frame_Width;j++)    
    {
      if(Image_Data[i][j]==0xff)
      {
        Image_Data[i][j]=0xfe;//防止发送标志位
      }
      UART_Put_Char(UART_4,Image_Data[i][j]);
    }
  }
}
*/

/*************************************************************** 
* 
* 函数名称：uint8_t GetOSTU(uint8_t tmImage[IMAGEH][IMAGEW]) 
* 功能说明：求阈值大小 
* 参数说明： 
* 函数返回：阈值大小 
* 修改时间：2018年3月27日 
* 备 注： 
参考：https://blog.csdn.net/zyzhangyue/article/details/45841255
      https://www.cnblogs.com/moon1992/p/5092726.html
      https://www.cnblogs.com/zhonghuasong/p/7250540.html     
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例w0，并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背景像素) 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7）将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
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
  double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差; 
  int16_t MinValue, MaxValue; 
  uint8_t Threshold = 0;
  uint8_t HistoGram[256];              //  

  for (j = 0; j < 256; j++)  HistoGram[j] = 0; //初始化灰度直方图，每个灰度计数0
  
  for (j = 0; j < IMAGEH; j++) 
  { 
    for (i = 0; i < IMAGEW; i++) 
    { 
      HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
    } 
  } 
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值
      
  if (MaxValue == MinValue)     return MaxValue;         // 图像中只有一个灰度   
  if (MinValue + 1 == MaxValue)  return MinValue;        // 图像中只有二个灰度
    
  for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];   //最小灰度到最大灰度像素总数
    
  PixelIntegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    PixelIntegral += HistoGram[j] * j;//灰度值*像素个数
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)//尝试每一个灰度值
  {
    PixelBack = PixelBack + HistoGram[j];   //前景像素点数
    PixelFore = Amount - PixelBack;         //背景像素点数
    OmegaBack = (double)PixelBack / Amount;//前景像素百分比
    OmegaFore = (double)PixelFore / Amount;//背景像素百分比
    PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
    PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
    MicroBack = (double)PixelIntegralBack / PixelBack;   //前景灰度百分比
    MicroFore = (double)PixelIntegralFore / PixelFore;   //背景灰度百分比
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
    if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //返回最佳阈值;
} 
/*************************************************************** 
* 
* 函数名称：void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW]) 
* 功能说明：图像数据二值化 
* 参数说明： 
* 函数返回：void 
* 修改时间：2018年3月27日 
* 备 注： 
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
* 函数名称：void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW]) 
* 功能说明：图像数据二值化（利用中位数进行处理 ）
* 参数说明： 
* 函数返回：void 
* 修改时间：2018年3月27日 
* 备 注： 
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








