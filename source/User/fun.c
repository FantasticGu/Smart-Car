#include "include.h"
#include "fun.h"
//摄像头参数
///////////////////////////////////////////////摄像头///////////////////////////////////////////////////////////
////////////////////////////////全局变量定义////////////////////////////////////
#define test_port UART_0
unsigned int row=0;	//摄像头行计数，最大240
uint8 video[H][V];	//存放数据数组
uint8 video_deal[H][V];	//存放数据数组
uint16 Bline_left[H];	 //左边线存放数组
uint16 Bline_right[H];	 //右边线存放数组
uint16 Pick_table[H];	 //中心线存放数组
uint16 PrBline_left[H];  //原始的左边线存放数组
uint16 PrBline_right[H]; //原始的右边线存放数组
uint8  Pick_flag[H];//该行是否找到黑线标志数组
uint8  Deal_flag[H];//处理数据是否有效标志数组
uint16 lost_already=0;
uint8 Pick_line=0;
uint8 PickCenter_flag=0;	//提取中线标志
uint8 Lost_Line_count=0;
uint8 Lost_left_count=0;
uint8 Lost_right_count=0;
uint8 Near_lost=0;//近处丢失标志
uint8 Shi_zi_line=0;
uint8 shizi_left = 0;
uint8 shizi_right = 0;
uint8 Shi_zi_num=0;
uint8 Shi_zi_flag=0;
uint8 positive_num=0;
uint8 decrease_num=0;
uint16 last_position=V/2;
uint8 stop_num=0;
int Position_diff=0;
float biLi=1.0;
unsigned int const data_table[H]={
  5, 10, 15, 20, 25, 30, 35, 40, 45, 50,
 55, 60, 65, 70, 75, 80, 85, 90, 95,100,
103,106,109,112,115,118,121,124,127,130,
133,136,139,142,145,148,151,154,157,160,
163,166,169,172,175,178,181,184,187,190,
193,196,199,202,205,208,211,214,217,220
};//需采集数据的行
uint8 Cmp=160;	//黑线阈值
uint8 lastCmp = 160;
uint8 haveinited = 0;
uint8 mid_before = 94; //上一时刻中线
uint8 row_F[H];	//该行采集完成标志
char startline_F;	//发现起始行
char endline_F;	//发现结束行 
unsigned int imagerow=0;	//采集行计数，最大H
uint8 Out_flag=0;		//出界标志，无效图像标志
uint8 start_write=160;   //起跑线白色阈值
uint8 liangdu=100;	//摄像头亮度

uint8 last_vline;
uint8 valid_line=60;//最大有效行
uint8 judge_vl;//用于判断的有效行
uint8 last_lost=55;	//上一场丢失行
uint8 baoguang=0x50;	//曝光时间值
uint8 Enter_shi_zi=0;//进入十字标志
uint16 Bline_diff=0;//两黑线距离
uint16 maxBline_diff=0;
uint16 LR_diff=0;
uint32 whole_area=0;
uint32 tx_area=0;

int far_diff=0;	//远端相对偏移量 作方向和直道长度判断
uint8 zhidao_speed=55;	//直道速度
uint8 CD_speed=40;	//全局速度
uint8 all_speed_flag=5;   //全局速度标志

uint8 slow_down_num=0;//减速标志
int judge_xielv[H-5];	//斜率判断数组
uint8 buzzer_num=0;	//蜂鸣器次数
uint8 buzzer_flag=0;	//蜂鸣器响应标志
uint8 zhidao_count_flag=0;	//直道判断标志
uint8 last_zhidao_flag=0;
uint8 start_end_flag=0;	//起跑线标志
uint8 roadFlag=0;//赛道标志
uint8 last_roadFlag=0;
uint8 temp_shizi=0;//零时用
uint32 run_time=0;//程序运行时间（次数）
uint8 ls_flag=0;//小S标志
uint8 S_road_flag=0;//S弯标志
uint8 pick_way=1;	//方向变量 左为0 右为2 中间为1
uint8 last_pick_way=1;//
uint8 lost_w_count=0;//白色丢失行变量
uint8 lost_b_count=0;//黑色丢失行变量
int near_xielv_left=0;
int near_xielv_right=0;

int even_diff=0;	//中心线平均偏差
int even_diff_near=0;
int even_diff1=0;   //
float dj_Kp=1;	//舵机Kp值
int D_slope_near=0;		//舵机斜率控制值
int D_slope=0;		//舵机斜率控制值
int Curve=0;		//Curve(曲线，判断)值
int Curve_near=0;		//Curve(曲线，判断)值
int Curve_middle=0;		//Curve(曲线，判断)值
int Curve_far=0;		//Curve(曲线，判断)值

int jiaodu_num=dj_center;	//角度值
int last_turn=dj_center;	//上一次转向值
int dj_pid_num=dj_center;	//舵机角度值

uint8 set_flag=1;//设置标志位
uint32 switch_key=0;	//拨码开关，IO端口值
uint8 par_num=4;	//设置参数编号，4为全局速度
uint8 last_par_num=0;	//上一次设置参数，作清屏用  
uint8 ctl_num=0;//控制标志 0为新控制，1为老控制
uint8 xianshi=2;//显示参数
uint8 init_7620_flag=1;	//OV7620设置标志位
uint8 flash_flag=0;//Flash标志
uint8 DisImg_flag=0;//显示图像标志
uint8 Far_find_flag=0;//
uint16 save_data_num=0;
uint8 save_file_num=0;

ParValue myPar_num;

//////////////////////luoyang new
uint8 lastmiddleplace=V/2;


uint8  BlackEndMR   = 0;//清零
uint8  BlackEndML   = 0;
uint8  BlackEndLL   = 0;
uint8  BlackEndRR   = 0;
uint8  BlackEndL    = 0;
uint8  BlackEndM    = 0;
uint8  BlackEndR    = 0;

uint8 LEndFlag  = 0;
uint8 MEndFlag  = 0;
uint8 REndFlag  = 0;	
uint8 MREndFlag = 0;
uint8 MLEndFlag = 0;
uint8 LLEndFlag = 0;
uint8 RREndFlag = 0;

uint16 bigbendflag = 0;


int leftpointflag=0;
int rightpointflag=0;
int leftwhiteline=0;
int rightwhiteline=0;

uint8 left,right=0;


uint8 state=0;

uint8 readyleftflag=0;
uint8 readyrightflag=0;

uint8 readyleftnum[10];
uint8 readyrightnum[10];

uint8 leftcircleflag=0;
uint8 rightcircleflag=0;

uint8 leftflagnum[10];
uint8 rightflagnum[10];

uint8 intoleftcircleflag=0;
uint8 intorightcircleflag=0;

uint8 outleftcircleflag=0;
uint8 outrightcircleflag=0;

uint8 gostraightflag=0;


uint8 leftcrossflag = 0;
uint8 rightcrossflag = 0;
uint8 middlecrossflag =0;

uint8 stopcarnum=0;
uint8 stopcarflag=0;

uint8 stopcarflagnum[10];

uint8 whiteblackflag=0;
uint8 middleline=90;

uint8 destinationflag=0;


uint8 camera_flag=1;

///////////////////////////////temp_variable////////////////////////////////
float area;
float last_area;
///////////////////////////////////////////////摄像头///////////////////////////////////////////////////////////

extern struct Bline Bline_lefts[PIC_H]; 
extern struct Bline Bline_rights[PIC_H];
extern uint8 PickCenter_flag;
//extern unsigned char const adjust_table[];//梯形失真调整参数数组
extern uint8 big_right;
extern uint8 Cmp;
extern uint8 buzzer_num;
extern uint8 buzzer_flag;
extern uint8 init_7620_flag;
extern uint8 roadFlag;
extern uint8 temp_shizi;
extern uint8 Shi_zi_num;
extern uint16  const duoji_table[];
extern uint8 liangdu;
extern uint8 baoguang;
//extern uint8 par_num;
extern uint8 row_F[H];//该行采集完成标志
extern uint8 ctl_num;//控制标志 0为新控制，1为老控制
extern char startline_F;//发现起始行
extern char endline_F;//发现结束行  
extern uint8 set_flag;//设置标志位
extern uint32 switch_key;//拨码开关，IO端口值
//extern ParValue myPar_num;
uint16 last_left_line=(V/2);
uint16 last_right_line=(V/2);
uint8 buttonsPressed=0;
/*static const char * const mainMenu[] = {
    " CDU--Spower",
    "1. CD_speed   ",
    "2. dis_Set    ",
    "3. ZD_speed   ",
    "4. BiLi       ",
    "5. Reset      ",
    "6. set_dj_Kp  ",
	"7. set_pid_P  ",
	"8. set_pid_D  ",
	"9. Threshold  ",
	"10.djSet      ",
	"11.sendFlashD ",
	"12.readSDData "
};*/
//IO初始化函数
/*void IO_Init()
{
	/* 打开各个端口的时钟源 */
/*	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | 
	SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
	//C0~C7设为GPIO输入模式，连接ov7260的8位灰度输入
	PORTC_PCR0=PORT_PCR_MUX(1);//B0引脚设置为GPIO模式
	PORTC_PCR1=PORT_PCR_MUX(1);//B1引脚设置为GPIO模式
	PORTC_PCR2=PORT_PCR_MUX(1);//B2引脚设置为GPIO模式
	PORTC_PCR3=PORT_PCR_MUX(1);//B3引脚设置为GPIO模式
	PORTC_PCR4=PORT_PCR_MUX(1);//B4引脚设置为GPIO模式
	PORTC_PCR5=PORT_PCR_MUX(1);//B5引脚设置为GPIO模式
	PORTC_PCR6=PORT_PCR_MUX(1);//B6引脚设置为GPIO模式
	PORTC_PCR7=PORT_PCR_MUX(1);//B7引脚设置为GPIO模式
	GPIOC_PDDR&=0XFFFFFF00;//B0~B7设置为输入，数字摄像头8位灰度输入
	
	PORTA_PCR14=PORT_PCR_MUX(1)|PORT_PCR_IRQC(10);//A14引脚设置为GPIO模式，下降沿中断,场中断
	
	PORTB_PCR22=PORT_PCR_MUX(1)|PORT_PCR_IRQC(9);//B22引脚设置为GPIO模式，上升沿中断,行中断
	
	/////////////////////////////核心板LED///////////////////////////////////////////////
	PORTA_PCR15=PORT_PCR_MUX(1);
	
	//////////////////////////////SD卡SPI模拟口初始化///////////////////////////////////
	//设置PORTA pin16,pin17,pin18,pin19为GPIO口
	PORTE_PCR0=(0|PORT_PCR_MUX(1));
	PORTE_PCR1=(0|PORT_PCR_MUX(1)); 
	PORTE_PCR2=(0|PORT_PCR_MUX(1));
	PORTE_PCR3=(0|PORT_PCR_MUX(1));
	PORTE_PCR4=(0|PORT_PCR_MUX(1));
	//设置PORTA pin16,pin17为输出方向
	GPIOE_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(0)|GPIO_PIN(1)|GPIO_PIN(2)|GPIO_PIN(3)|GPIO_PIN(4));
	GPIOE_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(0)|GPIO_PIN(1)|GPIO_PIN(2)|GPIO_PIN(3)|GPIO_PIN(4)); //先读取，然后才能输出
	//PORTB_PCR23=PORT_PCR_MUX(1)|PORT_PCR_IRQC(1);//B11引脚设置为GPIO模式，上升沿触发DMA请求
    ////////////////////// 扩展I/0口初始化 ////////////////////////////   
	 	PORTC_PCR16=PORT_PCR_MUX(1);
		PORTC_PCR15=PORT_PCR_MUX(1);
	////////////////////// 拨码开关I/0口初始化 ////////////////////////////
		PORTD_PCR0=PORT_PCR_MUX(1);
		PORTD_PCR1=PORT_PCR_MUX(1);
		PORTD_PCR2=PORT_PCR_MUX(1);
		PORTD_PCR3=PORT_PCR_MUX(1);
	////////////////////// 指示灯I/O口初始化 ///////////////////////////////
		PORTC_PCR10=PORT_PCR_MUX(1);		//LED1	右转向
		PORTD_PCR2 =PORT_PCR_MUX(1);		//LED2	左转向
		PORTB_PCR16=PORT_PCR_MUX(1);		//LED3
    	PORTB_PCR17=PORT_PCR_MUX(1);		//LED4
    	PORTB_PCR20=PORT_PCR_MUX(1);		//LED5
    ////////////////////// IIC模拟I/O初始化 ////////////////////////////////
		PORTC_PCR8=PORT_PCR_MUX(1);
    	PORTC_PCR9=PORT_PCR_MUX(1);
	////////////////////// 按键I/O初始化 ////////////////////////////////
		PORTB_PCR10=PORT_PCR_MUX(1);	//KEY1
		PORTB_PCR9=PORT_PCR_MUX(1);		//KEY2
		PORTB_PCR3=PORT_PCR_MUX(1);		//KEY3
    	PORTB_PCR2=PORT_PCR_MUX(1);		//KEY4
		PORTB_PCR1=PORT_PCR_MUX(1);		//KEY5
		PORTB_PCR10|=0x03;//设置为内部上拉模式
		PORTB_PCR9 |=0x03;
		PORTB_PCR3 |=0x03;//设置为内部上拉模式
		PORTB_PCR2 |=0x03;
		PORTB_PCR1 |=0x03;//设置为内部上拉模式

		GPIOA_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(15));
		GPIOC_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(10));
		GPIOC_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(15)|GPIO_PIN(16));
    	GPIOB_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(20)|GPIO_PIN(16)|GPIO_PIN(17));//B口指示灯
		GPIOD_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(2));//D口指示灯
		//GPIOD_PDDR&=~GPIO_PDDR_PDD(GPIO_PIN(0)|GPIO_PIN(1)|GPIO_PIN(2)|GPIO_PIN(3));//拨码开关
		GPIOB_PDDR&=~GPIO_PDDR_PDD(GPIO_PIN(10)|GPIO_PIN(9)|GPIO_PIN(3)|GPIO_PIN(2)|GPIO_PIN(1));//按键
		GPIOC_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(10));
		GPIOA_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(15)); 
		
		GPIOC_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(15)|GPIO_PIN(16));
		GPIOB_PDOR &= ~GPIO_PDOR_PDO(~GPIO_PIN(20)|~GPIO_PIN(16)|~GPIO_PIN(17)); //先读取，然后才能输出
		GPIOD_PDOR &= ~GPIO_PDOR_PDO(~GPIO_PIN(2)); //先读取，然后才能输出
    	GPIOC_PDOR &= ~GPIO_PDOR_PDO(~GPIO_PIN(14)); //先读取，然后才能输出
		
    	GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(20)|GPIO_PIN(16)|GPIO_PIN(17));	//IO口输出高电平，灭
		GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(2));	//IO口输出高电平，灭
		GPIOC_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(10));
	
}*/

/***************************************************************************//**
 * @brief   Set up a menu
 * @param   menuText  The text for the menu title and options.
 * @param   numItems  The number of menu items ( numItems <= 7 )
 * @return  Position of selection
 ******************************************************************************/

uint8 Menu_active(char **menuText, uint8 numItems)
{
    uint8 i;
    static uint8 position = 0;
	uint8 start_position=1;
    uint8 lastPosition = 15;
	uint8 listItems=3;
    /*if (numItems > 8){                                              // Screen is 8 lines tall = 1
                                                                    // title line + 7 items max
        numItems = 8;
    }*/
    //LCD_Print(0, 0, (uint8 *)menuText[0]); // Print the title
    buttonsPressed = 0;
    while (!buttonsPressed)                                         // Menu active until selection                                                               // is made
    {
		buttonsPressed=scan_key();
        if(KEY1==buttonsPressed)
		{
			if(position>0)
				position--;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			position++;
			buttonsPressed=0;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
			buttonsPressed=0;
			break;
		}
		else if(KEY3==buttonsPressed)
		{
			break;
		}
		else if(KEY4==buttonsPressed)
			buttonsPressed=0;
        if (position > numItems){
            position = numItems;
        }
        else if (position == 0){
            position = 1;
        }
				if(position>listItems)
				{
					start_position=position-listItems+1;
				}
				else
				{
					start_position=1;
				}
        if (position != lastPosition)                               // Update position if it is
                                                                    // changed 
        {
            for (i = start_position; i < listItems+start_position; i++)                      // Display menu items
            {
                if (i != position){
                    //LCD_Print(0, (i-start_position+1)*2, (uint8 *)menuText[i]);
                }
                else {
                    // Highlight item at current position
                    //LCD_P8x16StrInvert(0, (i-start_position+1)*2, (uint8 *)menuText[i]);
                }
            }
            lastPosition = position;
        }
    }
		if(buttonsPressed!=0)
    	return position;
		else
			return 0xFF;
}
/*******************************************************************************
函数名称：mode
函数功能: 模式选择函数
参数：无
*******************************************************************************/
/*void mode()
{
  switch (Menu_active((char **)mainMenu, 12))
  {
  	case 1: setArgUchar(" cdSpeed:        ",&CD_speed,1); break;//setCDSpeed()
    case 2: displaySet(); break;
    case 3: setArgUchar(" zdSpeed:        ",&zhidao_speed,1); break;//setZDSpeed() 
    case 4: setArgFloat(" BiLi:        ",&biLi,0.1);   break;	//BiLi()
    case 5: Reset();    break;
	case 6: setArgFloat(" dj_Kp:        ",&dj_Kp,0.1);    break;//set_dj_Kp()
	case 7: setArgInt(" Kp:        ",&Kp,1);    break;	//set_pid_P()
	case 8: setArgInt(" Kd:        ",&Kd,1);    break;	//set_pid_D()
  	case 9: setArgUchar(" Cmp:        ",&Cmp,1);	break;	//setThreshold()
  	case 10: djSet();	break;
	case 11: sendFlashData();	break;
    case 12: readSDData();	break;
    default: break;
   }
}*/
/*******************************************************************************
函数名称：sendFlashData
函数功能: 发送flash数据
参数：无
*******************************************************************************/
/*void sendFlashData()
{
    uint8 tmp=0;
	uint16 i=0;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp==1)
		{
			LCD_Print(0,2,"send....");
			flash_read(FLASH_ADDR,0,sizeof(test_table),test_table);
			for(i=0;i<TEST_SECTOR_SIZE;i++)
			  uart_send1(UART0,test_table[i]);
			tmp=0;
		}
		LCD_Print(0,2," wait......");
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
		    tmp=1;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：readSDData
函数功能: 读取SD数据
参数：无
*******************************************************************************/
/*void readSDData()
{
    uint8 tmp=0;
	uint8 pixel;
	uint32 sector=0;
	int offset=0;
	//char data[20]={0};
	uint8 ok_flag=0;
	int num=0;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	LCD_Print(0,2,"waiting....");
	while (!buttonsPressed) 
	{
		if(ok_flag==1)
		{
			LCD_Print(0,2,"read....  ");
			//LPLD_Disk_Read(0,test_table,sector,1);
			LCD_Print(0,2,"finish....");
			delay(100000);
			num=-128;
			while(num<510)
			{
				num++;
				Image_clear();//
				for(pixel=0;pixel<128;pixel++)
				{
				    offset=pixel+num;
		            if(offset<512&&offset>=0)
					Image_set(127-pixel,test_table[offset]);
					Image_set(127-pixel,LCD_HEIGHT/2);
				}
				Draw_MyImage();
				delay(1000);
			}
			LCD_CLS();
			ok_flag=0;
		}
		if(tmp!=0)
		{
			if(tmp==1)
			{
				sector++;
			}
			else if(tmp==2)
			{
				if(sector>0)
					sector--;
			}
			tmp=0;
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
		  	tmp=1;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
		  	tmp=2;
			buttonsPressed=0;
		}
		else if(KEY3==buttonsPressed)
		{
			ok_flag=1;
			buttonsPressed=0;
		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：choice_kz
函数功能: 选择控制参数
参数：无
*******************************************************************************/
/*void Reset()
{
  uint8 tmp;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=myPar_num.SET_FLAG)
		{
			LCD_Print(0,2," Reset:       ");
			if(myPar_num.SET_FLAG==1)
	  		LCD_Print(102,2,"yes");
			else
				LCD_Print(102,2,"no");
			tmp=myPar_num.SET_FLAG;
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			myPar_num.SET_FLAG=1;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			myPar_num.SET_FLAG=0;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：choice_kz
函数功能: 选择控制参数
参数：无
*******************************************************************************/
/*uint8 scan_key()
{
	uint32 key_code=0;
	uint16 key_num=0;
	uint8 key_value=0;
	//key_code=(GPIO_PDIR_REG(PORTB)&0x0000FFFF);
	//key_num=(uint16)key_code;
	//TEST_KEY(key_code,key_num);
	if(key_num!=0x060E)
	{
		delay(1);
		//TEST_KEY(key_code,key_num);
		if(key_num!=0x060E)
		{
			switch(key_num)
			{
				case 0x20E:key_value=1;break;
				case 0x40E:key_value=2;break;
				case 0x606:key_value=3;break;
				case 0x60A:key_value=4;break;
				case 0x60C:key_value=5;break;
				default:break;
			}
			while(key_num!=0x060E)
				//TEST_KEY(key_code,key_num);
		}
	}
	return key_value;
}
void Testled()
{
	while (!buttonsPressed) 
	{
		//GPIOB_PDOR&=~GPIO_PDDR_PDD(GPIO_PIN(20)|GPIO_PIN(16)|GPIO_PIN(17));//B口指示灯
		//GPIOD_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(2));	//IO口输出高电平，灭
		//GPIOC_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(10));
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：setThreshold
函数功能: 选择控制参数
参数：无
*******************************************************************************/
/*void setThreshold()
{
	char data[5];
	uint8 tmp=0;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=Cmp)
		{
			LCD_Print(0,2," Cmp:        ");
			sprintf(data,"%d",Cmp);
	  		LCD_Print(80,2,(uint8*)data);
			tmp=Cmp;
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			Cmp+=1;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			if(Cmp>1)
				Cmp-=1;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：choice_kz
函数功能: 选择控制参数
参数：无
*******************************************************************************/
/*void setCDSpeed()
{
	char data[5];
	uint8 speed_tmp;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(speed_tmp!=CD_speed)
		{
			LCD_Print(0,2," cdSpeed:        ");
			sprintf(data,"%d",CD_speed);
	  	LCD_Print(80,2,(uint8*)data);
			speed_tmp=CD_speed;
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			CD_speed+=5;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			if(CD_speed>5)
				CD_speed-=5;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：setArgInt
函数功能: 设置整型参数
参数：无
*******************************************************************************/
/*void setArgInt(char *str,int * Arg,const int step)
{
	char data[5];
	int tmp=0xff;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=((*Arg)))
		{
			LCD_Print(0,2,(byte *)str);
			sprintf(data,"%d",(*Arg));
	  		LCD_Print(80,2,(uint8*)data);
			tmp=(*Arg);
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			(*Arg)+=step;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			if((*Arg)>step)
				(*Arg)-=step;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：setArgUchar
函数功能: 设置无符号字符型参数
参数：无
*******************************************************************************/
/*void setArgUchar(char *str,unsigned char * Arg,const unsigned char step)
{
	char data[5];
	unsigned char tmp=0xff;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=((*Arg)))
		{
			LCD_Print(0,2,(byte *)str);
			sprintf(data,"%d",(*Arg));
	  		LCD_Print(80,2,(uint8*)data);
			tmp=(*Arg);
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			(*Arg)+=step;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			if((*Arg)>step)
				(*Arg)-=step;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：setArgFloat
函数功能: 设置浮点型参数
参数：无
*******************************************************************************/
/*void setArgFloat(char *str,float * Arg,const float step)
{
	char data[5];
	float tmp=100.0;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=((*Arg)))
		{
			LCD_Print(0,2,(byte *)str);
			sprintf(data,"%.2f",(*Arg));
	  		LCD_Print(80,2,(uint8*)data);
			tmp=(*Arg);
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			(*Arg)+=step;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			if((*Arg)>step)
				(*Arg)-=step;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：BiLi
函数功能: 选择控制参数
参数：无
*******************************************************************************/
/*void BiLi()
{
	char data[5];
	float tmp;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=biLi)
		{
			LCD_Print(0,2," BiLi:        ");
			sprintf(data,"%.2f",biLi);
	  	LCD_Print(80,2,(uint8*)data);
			tmp=biLi;
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			biLi+=0.1;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			if(biLi>0)
				biLi-=0.1;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：BiLi
函数功能: 选择控制参数
参数：无
*******************************************************************************/
/*void set_dj_Kp()
{
	char data[5];
	float tmp=0;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=dj_Kp)
		{
			LCD_Print(0,2," dj_Kp:        ");
			sprintf(data,"%.2f",dj_Kp);
	  		LCD_Print(80,2,(uint8*)data);
			tmp=dj_Kp;
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			dj_Kp+=0.1;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			if(dj_Kp>0)
				dj_Kp-=0.1;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：setZDSpeed
函数功能: 选择控制参数
参数：无
*******************************************************************************/
/*void setZDSpeed()
{
	char data[5];
	uint8 speed_tmp;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(speed_tmp!=zhidao_speed)
		{
			LCD_Print(0,2," ZDSpeed:        ");
			sprintf(data,"%d",zhidao_speed);
	  		LCD_Print(80,2,(uint8*)data);
			speed_tmp=zhidao_speed;
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			zhidao_speed+=5;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			if(zhidao_speed>5)
				zhidao_speed-=5;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：set_pid_P
函数功能: 设置pid的 D 值
参数：无
*******************************************************************************/
/*void set_pid_D()
{
	char data[5];
	int tmp;
	buttonsPressed=0;
	//LCD_CLS();//清屏
	//LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=Kd)
		{
			//LCD_Print(0,2," Kd:        ");
			//sprintf(data,"%d",Kd);
	  		//LCD_Print(80,2,(uint8*)data);
			tmp=Kd;
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			Kd+=1;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			if(Kd>1)
				Kd-=1;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：choice_kz
函数功能: 选择控制参数
参数：无
*******************************************************************************/
/*void set_pid_P()
{
	char data[5];
	int tmp;
	buttonsPressed=0;
	LCD_CLS();//清屏
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=Kp)
		{
			LCD_Print(0,2," Kp:        ");
			sprintf(data,"%d",Kp);
	  		LCD_Print(80,2,(uint8*)data);
			tmp=Kp;
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			Kp+=1;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
			if(Kp>1)
				Kp-=1;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：djSet
函数功能: 舵机设置
参数：无
*******************************************************************************/
void djSet()
{
	uint16 tmp=0xff;
	uint16 dj_num=dj_center;
	char data[5]={0};
	buttonsPressed=0;
	//LCD_CLS();//清屏
	//LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=dj_num)
		{
			//LCD_Print(0,2,"dj_num:     ");
			sprintf(data,"%d",dj_num);
			//LCD_Print(60,2,(uint8 *)data);
			tmp=dj_num;
			FTM1_C0V=dj_num;
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
			dj_num++;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
		    if(dj_num>0)
				dj_num--;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：displaySet
函数功能: 显示图像控制参数
参数：无
*******************************************************************************/
void displaySet()
{
	uint8 tmp=0xff;
	buttonsPressed=0;
	//LCD_CLS();//清屏
	//LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=DisImg_flag)
		{
			/*LCD_Print(0,2," Display:     ");
			if(DisImg_flag==0)
	  			LCD_Print(80,2,"Par  ");
			else if(DisImg_flag==1)
				LCD_Print(80,2,"Bline");
			else if(DisImg_flag==2)
				LCD_Print(80,2,"Th_ez");
			else if(DisImg_flag==3)
				LCD_Print(80,2,"SW   ");
			else if(DisImg_flag==4)
			  LCD_Print(80,2,"Par_m  ");
			else
			    LCD_Print(80,2,"None ");
			tmp=DisImg_flag;*/
		}
		buttonsPressed=scan_key();
		if(KEY1==buttonsPressed)
		{
		  	if(DisImg_flag<5)
				DisImg_flag++;
			buttonsPressed=0;
		}
		else if(KEY2==buttonsPressed)
		{
		    if(DisImg_flag>0)
				DisImg_flag--;
			buttonsPressed=0;

		}
		else if(KEY4==buttonsPressed)
		{
			break;
		}
		else if(KEY5==buttonsPressed)
		{
			set_flag=0;
		}
	}
	buttonsPressed=0;
}
/*******************************************************************************
函数名称：choice_kz
函数功能: 选择控制参数
参数：无
*******************************************************************************/
void choice_kz()
{
  uint8 temp_key=0,temp_flag=0;
  uint8 put[3]={0};
  //LCD_CLS();
  //LCD_Print(8,0,"wait_choicekz...");
  //temp_key=gpio_get(PORTD,4);
  if(temp_key==0)
  {
	delay1(5);
	//temp_key=gpio_get(PORTD,4);
	if(temp_key==0)
	{
		ctl_num=1;
	    temp_flag=1;
	}
	//while(gpio_get(PORTD,4)==0);
  }
  //temp_key=gpio_get(PORTD,5);
  if(temp_key==0)
  {
	delay1(5);
	//temp_key=gpio_get(PORTD,5);
	if(temp_key==0)
	{
	  ctl_num=0;
	  temp_flag=1;
	}
	//while(gpio_get(PORTD,5)==0);
  }
  if(temp_flag==1)
  {
	LCD_Print(8,2,"ctl_num:        ");
	char_change_1(ctl_num,put);
	LCD_Print(80,2,put);
	if(ctl_num==0)
	{
	  LCD_Print(8,4,"deal           ");
	}
	else if(ctl_num==1)
	{
	  LCD_Print(8,4,"stable_deal    ");
	}
  }
  
}
/*******************************************************************************
函数名称：threshold
函数功能: 阈值获得函数
参数：无
*******************************************************************************/
void threshold()
{
  
}
/*******************************************************************************
函数名称：test_threshold
函数功能: 阈值获得函数
参数：无
*******************************************************************************/
void test_threshold()
{
  uint32 threshold_temp=0;
  uint8 i,line,pixel,th,put[3]={0};
  enable_irq (87);//使能A口中断 ，A14场中断	   
  startline_F=0;//发现起始行标志清0
  endline_F=0;//发现结束行标志清0
  for(i=0;i<H&&endline_F==0;i++) //提取各行中心点并处理意外
  {
		while(row_F[i]==0);//等待该行采样完成
		row_F[i]=0;//清除采集完成标志位
  }
  for(line=3;line<6;line++)
  {
	for(pixel=100;pixel<180;pixel++)
	{
	  threshold_temp+=video[line][pixel];
	}
  }
  th=threshold_temp/240;
  //LCD_Print(8,0,"sd_liangdu...");
  //LCD_Print(8,2,"th:");
  char_change_1(th,put);
  //LCD_Print(52,2,put);
}
/*******************************************************************************
函数名称：choice_parameter
函数功能: 选择参数函数
参数：无
*******************************************************************************/
void choice_par()
{
  uint8 temp_key=0,temp_flag=0;
  uint8 put[3]={0};
  //LCD_CLS();
  //LCD_Print(8,0,"wait_choice.... ");
  //temp_key=gpio_get(PORTD,4);
  if(temp_key==0)
  {
	delay1(5);
	//temp_key=gpio_get(PORTD,4);
	if(temp_key==0)
	{
	  if(par_num>5)
		par_num=5;
	  else
	  	par_num=par_num+1;
	  temp_flag=1;
	}
	//while(gpio_get(PORTD,4)==0);
  }
  //temp_key=gpio_get(PORTD,5);
  if(temp_key==0)
  {
	delay1(5);
	//temp_key=gpio_get(PORTD,5);
	if(temp_key==0)
	{
	  if(par_num<1)
		par_num=0;
	  else
	  	par_num=par_num-1;
	  temp_flag=1;
	}
	//while(gpio_get(PORTD,5)==0);
  }
  if(temp_flag==1)
  {
	/*//LCD_Print(8,2,"par_num:         ");
 	char_change_1(par_num,put);
	delay2();
	//LCD_Print(72,2,put);
	if(par_num==0)
	  //LCD_Print(8,4,"Kp          ");
	else if(par_num==1)
	  LCD_Print(8,4,"Ki          ");
	else if(par_num==2)
	  LCD_Print(8,4,"Kd          ");
	else if(par_num==3)
	  LCD_Print(8,4,"zhidao_speed");
	else if(par_num==4)
	  LCD_Print(8,4,"cd_speed    ");
	else if(par_num==5)
	  LCD_Print(8,4,"yuzhi       ");
	else
	  LCD_Print(8,4,"            ");*/
  }
  temp_flag=0;
}
/*******************************************************************************
函数名称：set_parameter
函数功能: 设置参数函数
参数：无
*******************************************************************************/
void set_par()
{
  uint8 temp_key=0,temp_flag=0;
  uint8 put[3]={0};
  //LCD_CLS();
  //LCD_Print(8,0,"wait_set.... ");
  /*if(par_num==0)//Kp参数设置
  {
		  //temp_key=gpio_get(PORTD,4);
		  if(temp_key==0)
		  {
			delay1(5);
			//temp_key=gpio_get(PORTD,4);
			/*if(temp_key==0)
			{
			  if(Kp>60)
				Kp=60;
			  else
				Kp=Kp+1;
			  temp_flag=1;
			}*/
			//while(gpio_get(PORTD,4)==0);
		  //}
		  //temp_key=gpio_get(PORTD,5);
		  //if(temp_key==0)
		  //{
			//delay1(5);
			//temp_key=gpio_get(PORTD,5);
			/*if(temp_key==0)
			{
			  if(Kp<1)
				Kp=0;
			  else
				Kp=Kp-1;
			  temp_flag=1;
			}
			while(gpio_get(PORTD,5)==0);*/
		  //}
		  //if(temp_flag==1)
		  //{
			  //LCD_Print(8,2,"speed_Kp:       ");
			  //char_change_1(Kp,put);
			  //delay2();
			  //LCD_Print(80,2,put);
			  //LCD_Print(8,4,"            ");
		 // }
		  //temp_flag=0;
	  /*else if(par_num==1)//Ki参数设置
	  {
		temp_key=gpio_get(PORTD,4);
		  if(temp_key==0)
		  {
			delay1(5);
			temp_key=gpio_get(PORTD,4);
			if(temp_key==0)
			{
			  if(Ki>60)
				Ki=60;
			  else
				Ki=Ki+1;
			  temp_flag=1;
			}
			while(gpio_get(PORTD,4)==0);
		  }
		  temp_key=gpio_get(PORTD,5);
		  if(temp_key==0)
		  {
			delay1(5);
			temp_key=gpio_get(PORTD,5);
			if(temp_key==0)
			{
			  if(Ki<1)
				Ki=0;
			  else
				Ki=Ki-1;
			  temp_flag=1;
			}
			while(gpio_get(PORTD,5)==0);
		  }
		  if(temp_flag==1)
		  {
			  LCD_Print(8,2,"speed_Ki:         ");
			  char_change_1(Ki,put);
			  LCD_Print(80,2,put);
			  LCD_Print(8,4,"            ");
		  }
		  temp_flag=0;
	  }
	  else if(par_num==2)//Kd参数设置
	  {
		temp_key=gpio_get(PORTD,4);
		  if(temp_key==0)
		  {
			delay1(5);
			temp_key=gpio_get(PORTD,4);
			if(temp_key==0)
			{
			  if(Kd>60)
				Kd=60;
			  else
				Kd=Kd+1;
			  temp_flag=1;
			}
			while(gpio_get(PORTD,4)==0);
		  }
		  temp_key=gpio_get(PORTD,5);
		  if(temp_key==0)
		  {
			delay1(5);
			temp_key=gpio_get(PORTD,5);
			if(temp_key==0)
			{
			  if(Kd<1)
				Kd=0;
			  else
				Kd=Kd-1;
			  temp_flag=1;
			}
			while(gpio_get(PORTD,5)==0);
		  }
		  if(temp_flag==1)
		  {
			  LCD_Print(8,2,"speed_Kd:         ");
			  char_change_1(Kd,put);
			  delay2();
			  LCD_Print(80,2,put);
			  LCD_Print(8,4,"            ");
		  }
		  temp_flag=0;*/
  //}
  if(par_num==3)//直道速度
  {
		  //temp_key=gpio_get(PORTD,4);
		  if(temp_key==0)
		  {
			delay1(5);
			//temp_key=gpio_get(PORTD,4);
			if(temp_key==0)
			{
			  if(zhidao_speed>200)
				zhidao_speed=200;
			  else
				zhidao_speed=zhidao_speed+2;
			  temp_flag=1;
			}
			//while(gpio_get(PORTD,4)==0);
		  }
		  //temp_key=gpio_get(PORTD,5);
		  if(temp_key==0)
		  {
			delay1(5);
			//temp_key=gpio_get(PORTD,5);
			if(temp_key==0)
			{
			  if(zhidao_speed<1)
				zhidao_speed=0;
			  else
				zhidao_speed=zhidao_speed-2;
			  temp_flag=1;
			}
			//while(gpio_get(PORTD,5)==0);
		  }
		if(temp_flag==1)
 	  	{
		  //LCD_Print(8,2,"zd_speed:     ");
		  char_change_1(zhidao_speed,put);
		  delay2();
		  //LCD_Print(80,2,put);
		  //LCD_Print(8,4,"            ");
	  	}
	  	temp_flag=0;
	  }
	 else if(par_num==4)//全局速度
	  {
			  //temp_key=gpio_get(PORTD,4);
			  if(temp_key==0)
			  {
				delay1(5);
				//temp_key=gpio_get(PORTD,4);
				if(temp_key==0)
				{
				  if(CD_speed>200)
					CD_speed=200;
				  else
					CD_speed=CD_speed+2;
				  temp_flag=1;
				}
				//while(gpio_get(PORTD,4)==0);
			  }
			  //temp_key=gpio_get(PORTD,5);
			  if(temp_key==0)
			  {
				delay1(5);
				//temp_key=gpio_get(PORTD,5);
				if(temp_key==0)
				{
				  if(CD_speed<1)
					CD_speed=0;
				  else
					CD_speed=CD_speed-2;
				  temp_flag=1;
				}
				//while(gpio_get(PORTD,5)==0);
	         }
			 if(temp_flag==1)
 	  	    {
			  //LCD_Print(8,2,"cd_speed:     ");
			  char_change_1(CD_speed,put);
			  delay2();
			  //LCD_Print(80,2,put);
			  //LCD_Print(8,4,"            ");
	  		}
	  		temp_flag=0;
     }
  else if(par_num==5)
	  {
			  //temp_key=gpio_get(PORTD,4);
			  if(temp_key==0)
			  {
				delay1(5);
				//temp_key=gpio_get(PORTD,4);
				if(temp_key==0)
				{
				  if(Cmp>200)
					Cmp=200;
				  else
					Cmp=Cmp+1;
				  temp_flag=1;
				}
				//while(gpio_get(PORTD,4)==0);
			  }
			  //temp_key=gpio_get(PORTD,5);
			  if(temp_key==0)
			  {
				delay1(5);
				//temp_key=gpio_get(PORTD,5);
				if(temp_key==0)
				{
				  if(Cmp<1)
					Cmp=0;
				  else
					Cmp=Cmp-1;
				  temp_flag=1;
				}
				//while(gpio_get(PORTD,5)==0);
	         }
			 if(temp_flag==1)
 	  	    {
			 // LCD_Print(8,2,"cmp:        ");
			  char_change_1(Cmp,put);
			  delay2();
			  //LCD_Print(80,2,put);
			  //LCD_Print(8,4,"            ");
	  		}
	  		temp_flag=0;
     }
}
/*******************************************************************************
函数名称：choice_xs
函数功能: 选择显示参数
参数：无
*******************************************************************************/
/*void choice_xs()
{
  uint8 temp_key=0,temp_flag=0;
  uint8 put[3]={0};
  //LCD_CLS();
  LCD_Print(8,0,"wait_choice_xs...");
  temp_key=gpio_get(PORTD,4);
  if(temp_key==0)
  {
	delay1(5);
	temp_key=gpio_get(PORTD,4);
	if(temp_key==0)
	{
	  if(xianshi<5)
	  	xianshi=xianshi+1;
	  temp_flag=1;
	}
	while(gpio_get(PORTD,4)==0);
  }
  temp_key=gpio_get(PORTD,5);
  if(temp_key==0)
  {
	delay1(5);
	temp_key=gpio_get(PORTD,5);
	if(temp_key==0)
	{
	  if(xianshi>0)
	  	xianshi=xianshi-1;
	  temp_flag=1;
	}
	while(gpio_get(PORTD,5)==0);
  }
  if(temp_flag==1)
  {
	LCD_Print(8,2,"xianshi:         ");
 	char_change_1(xianshi,put);
	delay2();
	LCD_Print(72,2,put);
  }
  temp_flag=0;
}

/*******************************************************************************
函数名称：find_coordinate
函数功能: 找到定位参考线,第27行前瞻为80cm
参数：
*******************************************************************************/
/*uint8 find_coordinate()
{
	  uint16 left_flag=(V-1),right_flag=0,pixel,flag=1,line=26;
	  uint8 i;
	  for(pixel=V/2;pixel<V;pixel++)
	  {
		  if(video[line][pixel]<Cmp)
		  {
			  for(i=1;i<6;i++)
			  {
				if(video[line][pixel+i]<Cmp)
				  flag++;
			  }
			  if(flag>2)
			  {
				left_flag=pixel;
				break;
			  }
		  }
	  }
	  for(pixel=V/2-1;pixel>0;pixel--)
	  {
			if(video[line][pixel]<Cmp)
			{
			  for(i=1;i<6;i++)
			  {
				if(video[line][pixel-i]<Cmp)
				  flag++;
			  }
			  if(flag>2)
			  {
			  	right_flag=pixel;
				break;
			  }
			}
	  }
	  if(left_flag<200&&right_flag>80)
	  {
	  	//GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(10));//IO口输出亮低电平
		return 1;
	  }
	  else
	  {
		//GPIOB_PDOR |= 	GPIO_PDOR_PDO(GPIO_PIN(10));//IO口输出灭高电平
		return 0;
	  }
}
//发送图像功能函数
void put_image_data()
{
    uint16 line,pixel;
    uart_send1(UART0,0xAA);
    uart_send1(UART0,0xAA);
    uart_send1(UART0,0xAA);
    for(line=0;line<H;line++)
    {
       for(pixel=0;pixel<V;pixel++)
       {
          uart_send1(UART0,video[line][pixel]);
         }
       uart_send1(UART0,0xAA);
       uart_send1(UART0,0x55);
       uart_send1(UART0,0xAA);
     }
    uart_send1(UART0,0xFF);
    uart_send1(UART0,0xFF);
    uart_send1(UART0,0xFF);
}
//发送图像功能函数1
void put_image_1(void)
{
    uint16 line,pixel;
    disable_irq(87);
    disable_irq(88);
    disable_irq(0);
    uart_send1(UART0,0x00);
    uart_send1(UART0,0xFF);
    uart_send1(UART0,0x01);
    uart_send1(UART0,0x0);
    for(line=0;line<H;line++)
    {
       for(pixel=0;pixel<V;pixel++)
       {
          uart_send1(UART0,video[line][pixel]);
         }
      }
      enable_irq(87);
}
//发送图像功能函数2
void put_image_2(uint8 put_line)
{
    uint16 line,pixel;
    disable_irq(87);
    disable_irq(88);
    disable_irq(0);
    uart_send1(UART0,0x00);
    uart_send1(UART0,0xFF);
    uart_send1(UART0,0x01);
    uart_send1(UART0,0x0);
    for(line=0;line<put_line;line++)
    {
       for(pixel=0;pixel<V;pixel++)
       {
          uart_send1(UART0,video[line][pixel]);
         }
      }
      enable_irq(87);
}
/*******************************************************************************
函数名称：put_center_char
函数功能: 发送中心线，每5行发送1行，以字符发送
参数：
*******************************************************************************/
/*void put_center_char(uint16 *center,uint8 put_line)
{
    uint16 pixel;
    uint8  line,flag=0,i=0,char_1,char_2;
    for(line=0;line<put_line;line++)
    {
          char_1=line/10+48;
          char_2=line%10+48;
          uart_send1(UART0,char_1);
          uart_send1(UART0,char_2);
          uart_send1(UART0,'-');
          for(pixel=0;pixel<V;pixel++)
           {
             i++;
             if(pixel==*center)
               flag=1;
             if(i==5)
             {
                if(flag==1)
                  {
                    uart_send1(UART0,'1');
                    uart_send1(UART0,'1');
                    flag=0;
                  }
                else
                  { 
                    uart_send1(UART0,'0');
                  }
                i=0;
             }
            }
          uart_send1(UART0,'\n');
          center++;
    }
      enable_irq(87);
}
/*******************************************************************************
函数名称：put_center_hex
函数功能: 发送中心线，每5行发送1行，以十六进制发送
参数：
*******************************************************************************/
/*void put_center_hex(uint16 *center,uint8 put_line)
{
    uint16 line,pixel,flag=0,i=0;
    uart_send1(UART0,0x00);
    uart_send1(UART0,0xFF);
    uart_send1(UART0,0x01);
    uart_send1(UART0,0x0);
    for(line=0;line<put_line;line++)
    {
           for(pixel=0;pixel<V;pixel++)
           {
             i++;
             if(pixel==*center)
               flag=1;
             if(i==5)
             {
                if(flag==1)
                  {
                    uart_send1(UART0,0x01);
                    flag=0;
                  }
                else
                {uart_send1(UART0,0x00);}
                i=0;
             }
          }
          center++;
    }
      enable_irq(87);
}
/*******************************************************************************
函数名称：put_get_char_whole
函数功能: 发送中心线和两黑线，每5行发送1行，以字符发送
参数：
*******************************************************************************/
/*void put_get_char_whole(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line)
{
    uint16 line,pixel,flag=0;
	uint8 get_line1=0,get_line2=0;
    for(line=0;line<put_line;line++)
    {
	  get_line1=line/10+48;
	  get_line2=line%10+48;
	  uart_send1(UART0,get_line1);
	  uart_send1(UART0,get_line2);
	  uart_send1(UART0,'-');
           for(pixel=0;pixel<V;pixel++)
           {
             if(pixel==*center)
               flag=1;
             else if(pixel==*bline_left)
               flag=1;
             else if(pixel==*bline_right)
               flag=1;
             if(flag==1)
                  {
                    uart_send1(UART0,'1');
                    flag=0;
                  }
             else
                {uart_send1(UART0,'0');}
          }
		  uart_send1(UART0,'\n');
          center++;
          bline_left++;
          bline_right++;
    }
	uart_send1(UART0,'\n');
}
/*******************************************************************************
函数名称：put_get_hex_whole
函数功能: 发送中心线和两黑线
参数：
*******************************************************************************/
/*void put_get_hex_whole(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line)
{
    uint16 line,pixel,flag=0;
	disable_irq(87);
    uart_send1(UART0,0x00);
    uart_send1(UART0,0xFF);
    uart_send1(UART0,0x01);
    uart_send1(UART0,0x0);
    for(line=0;line<put_line;line++)
    {
           for(pixel=0;pixel<V;pixel++)
           {
             if(pixel==*center)
               flag=1;
             else if(pixel==*bline_left)
               flag=1;
             else if(pixel==*bline_right)
               flag=1;
             if(flag==1)
                  {
                    uart_send1(UART0,0xFF);
                    flag=0;
                  }
             else
                {uart_send1(UART0,0x00);}
          }
          center++;
          bline_left++;
          bline_right++;
    }
	enable_irq(87);
}
/*******************************************************************************
函数名称：put_get_hex
函数功能: 发送中心线和两黑线，每5行发送1行，以十六进制发送
参数：
*******************************************************************************/
/*void put_get_hex(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line)
{
    uint16 line,pixel,flag=0,i=0;
    uart_send1(UART0,0x00);
    uart_send1(UART0,0xFF);
    uart_send1(UART0,0x01);
    uart_send1(UART0,0x0);
    for(line=0;line<put_line;line++)
    {
           for(pixel=0;pixel<V;pixel++)
           {
             i++;
             if(pixel==*center)
               flag=1;
             else if(pixel==*bline_left)
               flag=1;
             else if(pixel==*bline_right)
               flag=1;
             if(i==5)
             {
                if(flag==1)
                  {
                    uart_send1(UART0,0x01);
                    flag=0;
                  }
                else
                {uart_send1(UART0,0x00);}
                i=0;
             }
          }
          center++;
          bline_left++;
          bline_right++;
    }
}
//提取中心线发送图像功能函数
void put_image_center(uint16 *center,uint8 put_line)
{
    uint16 line,pixel;
    disable_irq(87);
    disable_irq(88);
    disable_irq(0);
    uart_send1(UART0,0x00);
    uart_send1(UART0,0xFF);
    uart_send1(UART0,0x01);
    uart_send1(UART0,0x0);
    for(line=0;line<put_line;line++)
    {
       for(pixel=0;pixel<V;pixel++)
       {
          if(pixel==*center)
            uart_send1(UART0,0x01);
          else
            uart_send1(UART0,0x00);
         }
        center++;
      }
      enable_irq(87);
}
//发送图像功能函数3
void put_image_3(void)
{
    uint16 line,pixel;
    disable_irq(87);
    disable_irq(88);
    disable_irq(0);
    uart_send1(UART0,0x0);
    uart_send1(UART0,0x0);
    //uart_send1(UART0,0xAA);
    for(line=0;line<H;line++)
    {
       for(pixel=0;pixel<V;pixel++)
       {
          uart_send1(UART0,video[line][pixel]);

         }
          uart_send1(UART0,0xAA);
          uart_send1(UART0,0xAA);
          uart_send1(UART0,0xAA);
      }
    uart_send1(UART0,0x0);
    uart_send1(UART0,0x0);
      enable_irq(87);
}
/*******************************************************************************
函数名称：binaryzation
函数功能: 二值化图像数据
参数：
*******************************************************************************/
void Binaryzation()
{
  uint16 line,pixel;
  for(line=0;line<H;line++)
  {
    for(pixel=V/2;pixel<V;pixel++)
    {
      if(video[line][pixel]<Cmp)//Cmp为摄像头二值化阈值
      {
        video[line][pixel]=0;
      }
      else
        video[line][pixel]=1;
    }
      for(pixel=V/2-1;pixel>0;pixel--)
    {
      if(video[line][pixel]<Cmp)
      {
        video[line][pixel]=0;
      }
      else
        video[line][pixel]=1;
        
      }
    }
}
/*******************************************************************************
函数名称：binaryzation_line
函数功能: 二值化图像数据
参数：
*******************************************************************************/
void Binary_line(uint8 line)
{
  uint16 pixel;
  uint32 gray_value=0;
  uint32 count=0;
  uint8 Deal_CMP=120;
  //for(pixel=0;pixel<V;pixel++)
  //{
	//if((Image_Data[line][pixel]<235)&&(Image_Data[line][pixel]>30))
	//{
	  	//count++;
		//gray_value+=Image_Data[line][pixel];
	//}
  //}
  //Deal_CMP=gray_value/count;
  for(pixel=0;pixel<V;pixel++)
  {
     if(Image_Data[line][pixel]<Deal_CMP)//Cmp为摄像头二值化阈值
     {
        Image_Data[line][pixel]=1; //灰度小于阈值，黑点
     }
     else
        Image_Data[line][pixel]=0; //大于阈值，白点
  }
}
/*******************************************************************************
函数名称：getWholeArea
函数功能: 获得有效行内的整体面积
参数：
*******************************************************************************/
uint32 getWholeArea()
{
  	uint16 line;
  	uint32 area=0;
  	for(line=V-1;line>V-valid_line;line--)
  	{
	  //if((Deal_flag[line]&INVALID_LINE)==0)
	  {
		area+=(Bline_left[line]-Bline_right[line]);
	  }
    }
	return area;
}
/*******************************************************************************
函数名称：regression（回归）
函数功能: 最小二乘法求斜率
参数：
*******************************************************************************/
int regression(uint16 Pick_table[],int startline,int endline)//计算斜率函数
{
    int num=0,i;
    int sumX=0,sumY=0,avrX=0,avrY=0;
    int B_up1=0,B_up2=0;
	int B_up=0,B_down=0;
	int slope=0;
    for(i=startline;i<=endline;i++)
    {
         if(Pick_table[i]) 
         {
            num++;
            sumX+=i;
            sumY+=Pick_table[i];
         }
    }
    avrX=sumX/num;
    avrY=sumY/num;
    for(i=startline;i<=endline;i++)
    {
	         if(Pick_table[i]) 
	         { 
	            B_up1=(int)Pick_table[i]-(int)avrY;
	            B_up2=i-avrX;
	            B_up+=(int)B_up1*(int)B_up2;
	            //B_up=B_up/100*100;
	            B_down+=(int)(i-avrX)*(int)(i-avrX);
	         }
   }
   if(B_down==0) slope=0;
   else slope=B_up*10/B_down;
   return slope;
}
/*******************************************************************************
函数名称：getWholeArea
函数功能: 获得有效行内的整体面积
参数：
*******************************************************************************/
uint32 getTxArea()
{
  	uint32 area=0;
	uint32 area1=0;
	uint32 area2=0;
	area1=(uint32)get_area(Bline_left[0],Bline_right[0],Bline_left[valid_line],0,0,valid_line);
	area2=(uint32)get_area(Bline_right[0],Bline_left[valid_line],Bline_right[valid_line],0,valid_line,valid_line);
	area=area1+area2;
	return area;
}
/*******************************************************************************
函数名称：get_area
函数功能: 计算曲率
参数：
*******************************************************************************/
float get_area_dsp(uint16 x1,uint16 x2,uint16 x3,uint16 y1,uint16 y2,uint16 y3)//A(x1,y1),B(x2,y2),C(x3,y3)
{
  float temp1,temp2,temp3,temp4,temp5,temp6,temp7,temp8,div=0.5;
  temp1=x2-x1;
  temp2=y3-y1;
  temp4=x3-x1;
  temp5=y2-y1;
  arm_mult_f32(&temp1, &temp2, &temp3, 1);
  arm_mult_f32(&temp4, &temp5, &temp6, 1);
  temp7=temp3-temp6;  
  arm_mult_f32(&temp7, &div, &temp8, 1); 
  return temp8;
}
/*******************************************************************************
函数名称：get_curvature
函数功能: 计算曲率
参数：
*******************************************************************************/
float get_curvature(uint16 x1,uint16 x2,uint16 x3,uint16 y1,uint16 y2,uint16 y3)//A(x1,y1),B(x2,y2),C(x3,y3)
{
  float temp1,temp2,temp3,temp4,ab,bc,ac,area,num=0,qulv;
  area=get_area(x1,x2,x3,y1,y2,y3);
  temp1=x2-x1;
  temp2=y2-y1;
  arm_mult_f32(&temp1, &temp1, &temp3, 1);
  arm_mult_f32(&temp2, &temp2, &temp4, 1);
  temp4=temp4+temp3;
  arm_sqrt_f32(temp4,&ab);
  
  temp1=x3-x2;
  temp2=y3-y2;
  arm_mult_f32(&temp1, &temp1, &temp3, 1);
  arm_mult_f32(&temp2, &temp2, &temp4, 1);
  temp4=temp4+temp3;
  arm_sqrt_f32(temp4,&ac);
  temp1=x3-x1;
  temp2=y3-y1;
  
  arm_mult_f32(&temp1, &temp1, &temp3, 1);
  arm_mult_f32(&temp2, &temp2, &temp4, 1);
  temp4=temp4+temp3;
  arm_sqrt_f32(temp4,&bc);
  
  arm_mult_f32(&ab, &bc, &num, 1);
  arm_mult_f32(&ac, &num, &num, 1);
  qulv=area/num;
  return qulv;
}
/*******************************************************************************
函数名称：get_area
函数功能: 计算曲率 
参数：
*******************************************************************************/
float get_area(uint32 x1,uint32 x2,uint32 x3,uint32 y1,uint32 y2,uint32 y3)//A(x1,y1),B(x2,y2),C(x3,y3)
{
  int temp1,temp2,temp3,temp4,temp5,temp6,temp7,temp8;
  temp1=x2-x1;
  temp2=y3-y1;
  temp4=x3-x1;
  temp5=y2-y1;
  temp3=temp1*temp2;
  temp6=temp4*temp5;
  temp7=temp3-temp6;  
  temp8=temp7/2; 
  return temp8;
}


/*******************************************************************************
函数名称：PickCenter_near
函数功能: 寻找近处中心线函数
参数：
*******************************************************************************/
int32 checkShizi(uint8 line)
{
  int32 diff;
  diff=0;
  /*int32 pixel,flag=0;//,temp1,temp2;
  uint32 Black_count_left=0,Black_count_right=0;
  uint16 pick_line=0,temp_line;
  uint16 temp_lost_line=0;
  int32 start_pixel;
  //uint16 left_lost_flag=0,right_lost_flag=0;
  //const int Cmp_D=25;
  //const int  MAX=240;
  uint32 i;
  //for(pick_line=0;pick_line<5;pick_line++)
	  {
			for(pixel=start_pixel;pixel<V;pixel+=2)
			{
				if(video[pick_line][pixel]<Cmp)
				{
					  flag=0;
					  if(pixel<(V-4))
					  {
						  for(i=1;i<5;i++)
						  {
								if(video[pick_line][pixel+i]<Cmp)
									flag++;
						  }
						  if(flag>3)
						  {
								Bline_left[pick_line]=pixel;
								break;
						  }
					  }
					  else
					  {
							i=pixel;
							while(i<(V-1))
							{
								if(video[pick_line][i+1]<Cmp)
									flag++;
								i++;
							 }
							if(flag>0)
							{
							 	Bline_left[pick_line]=pixel;
								break;
							}
					  }
				 }
			}
			if(pixel>=V)
			{
			  Bline_left[pick_line]=V-1;
			  Pick_flag[pick_line] |= LEFT_LOST_W;
			}
			for(pixel=start_pixel;pixel>0;pixel-=2)
			{
				if(video[pick_line][pixel]<Cmp)
				{
					  flag=0;
					  if(pixel>3)
					  {
						  for(i=1;i<5;i++)
						  {
								if(video[pick_line][pixel-i]<Cmp)
									flag++;
						  }
						  if(flag>3)
						  {
								 Bline_right[pick_line]=pixel;
								 break;
						  }
					  }
					  else
					  {
							i=pixel;
							while(i>0)
							{
								if(video[pick_line][i-1]<Cmp)
									flag++;
								i--;
							 }
							if(flag>0)
							{
								 Bline_right[pick_line]=pixel;
								 break;
							}
					  }
				}
			}
			if(pixel<=0)
			{
			  Bline_right[pick_line]=0;
			  Pick_flag[pick_line] |= RIGHT_LOST_W;
			}
			{
			 	diff=abs_sub(Bline_left[pick_line],Bline_right[pick_line]);
			}
	  }
  */
	  return diff;
}
/*******************************************************************************
函数名称：PickCenter_near
函数功能: 寻找近处中心线函数
参数：
*******************************************************************************/
int32 PickCenter_near()
{
  int32 pixel,flag=0;//,temp1,temp2;
  uint32 Black_count_left=0,Black_count_right=0;
  uint16 pick_line=0,temp_line;
  uint16 temp_lost_line=0;
  int32 start_pixel;
  //uint16 left_lost_flag=0,right_lost_flag=0;
  //const int Cmp_D=25;
  //const int  MAX=240;
  uint32 i;
  for(temp_line=0;temp_line<5;temp_line++)//统计前5行中间列左右50列黑点数目 
  {																			  //后期可以尝试边沿法
		for(pixel=V/2;pixel<(V/2+45);pixel++)
		{
			if(Image_Data[temp_line][pixel] == 1)
			{
				Black_count_right++;
			}
		}
		for(pixel=V/2-45;pixel<(V/2);pixel++)
		{
			if(Image_Data[temp_line][pixel] == 1)
			{
				Black_count_left++;
			}
		}
  }
  if(Black_count_right>Black_count_left)//中间右边黑点数目大于左边 赛道偏左
  {
		start_pixel=V/2+45;
  }
  else if(Black_count_right<Black_count_left)//中间左边黑点数目大于右边 赛道偏右
  {
		start_pixel=V/2-45;
  }
  else
  {
		start_pixel=V/2;
  }
  for(pick_line=0;pick_line<5;pick_line++)
	  {
			for(pixel=start_pixel;pixel<V;pixel++)
			{
				if(Image_Data[pick_line][pixel] == 1)
				{
					  flag=0;
					  if(pixel<(V-7))
					  {
						  for(i=1;i<4;i++)
						  {
								if(Image_Data[pick_line][pixel+i] == 1)
									flag++;
						  }
						  if(flag>3)
						  {
								Bline_right[pick_line]=pixel;
								break;
						  }
					  }
					  else
					  {
							i=pixel;
							while(i<(V-1))
							{
								if(Image_Data[pick_line][i+1] == 1)
									flag++;
								i++;
							 }
							if(flag>0)
							{
							 	Bline_right[pick_line]=pixel;
								break;
							}
					  }
				 }
			}
			if(pixel>=V)
			{
			  Bline_right[pick_line]=V-1;
			  Pick_flag[pick_line]|=RIGHT_LOST_W;
			}
			for(pixel=start_pixel;pixel>0;pixel--)
			{
				if(Image_Data[pick_line][pixel] == 1)
				{
					  flag=0;
					  if(pixel>3)
					  {
						  for(i=1;i<4;i++)
						  {
								if(Image_Data[pick_line][pixel-i] == 1)
									flag++;
						  }
						  if(flag>3)
						  {
								 Bline_left[pick_line]=pixel;
								 break;
						  }
					  }
					  else
					  {
							i=pixel;
							while(i>0)
							{
								if(Image_Data[pick_line][i-1] == 1)
									flag++;
								i--;
							 }
							if(flag>0)
							{
								 Bline_left[pick_line]=pixel;
								 break;
							}
					  }
				}
			}
			if(pixel<=0)
			{
			  Bline_left[pick_line]=0;
			  Pick_flag[pick_line]|=LEFT_LOST_W;
			}
			if(Bline_left[pick_line]==(V-1)&&Bline_right[pick_line]==0)//左右边界都未找到
			{
				temp_lost_line++;
				Pick_flag[pick_line]|=ALL_LOST_W;
			}
			else if(Bline_left[pick_line]<(V-100)&&Bline_right[pick_line]>100)
			{
			    if(last_zhidao_flag==1)
					if((Bline_left[pick_line]-Bline_right[pick_line])<100)
					{
					  //start_end_flag=1;
					}
			}
			//else
			{
			  Bline_diff=abs_sub(Bline_left[pick_line],Bline_right[pick_line]);
			}
	  }
 
  //为跟线搜索算法参数赋初值
  last_left_line=Bline_left[4];
  last_right_line=Bline_right[4];
  if(temp_lost_line>0)
	return temp_lost_line;
  else
  	return 0;
}


/*******************************************************************************
函数名称：PickCenter_near_advance
函数功能: 寻找近处中心线函数
参数：
*******************************************************************************/
int32 PickCenter_near_advance()
{
  int32 pixel,flag=0;//,temp1,temp2;
  uint32 Black_count_left=0,Black_count_right=0;
  uint16 pick_line=0,temp_line;
  uint16 temp_lost_line=0;//左右都是白色，找不到赛道边缘数量
  int32 start_pixel = 94;//屏幕中央
  //uint16 left_lost_flag=0,right_lost_flag=0;
  //const int Cmp_D=25;
  //const int  MAX=240;
  uint32 i;
  /*for(temp_line=0;temp_line<5;temp_line++)//统计前5行中间列左右50列黑点数目 
  {																			  //后期可以尝试边沿法
		for(pixel=V/2;pixel<(V/2+45);pixel++)
		{
			if(Image_Data[temp_line][pixel] == 1)
			{
				Black_count_right++;
			}
		}
		for(pixel=V/2-45;pixel<(V/2);pixel++)
		{
			if(Image_Data[temp_line][pixel] == 1)
			{
				Black_count_left++;
			}
		}
  }
  if(Black_count_right>Black_count_left)//中间右边黑点数目大于左边 赛道偏左
  {
		start_pixel=V/2+45;
  }
  else if(Black_count_right<Black_count_left)//中间左边黑点数目大于右边 赛道偏右
  {
		start_pixel=V/2-45;
  }
  else
  {
		start_pixel=V/2;
  }*/
  
  //# define change_set 30 change_start 15
  
  start_pixel = V/2;
  if(mid_before < V/2 - change_start)
  {
    start_pixel = V/2 - change_set;
  }
  else if(mid_before > V/2 + change_start)
  {
    start_pixel = V/2 + change_set;
  }
  
  //start_pixel = (mid_before + V/2)/2;
  for(pick_line=H-1;pick_line>H-5;pick_line--)//从最下面开始看，只看最下的4行
	  {
			for(pixel=start_pixel;pixel<V;pixel++)//从开始点往最右侧找
			{
				if(Image_Data[pick_line][pixel] < Cmp)//Cmp很像阈值，大于的话说明是白赛道
				{
					  flag=0;
					  if(pixel<(V-7))//没到边缘的位置
					  {
						  //uart_printf(test_port, "测试\n");
                                                    for(i=0;i<4;i++)//从这个点再往旁边找4个，记录白色点的个数
						  {
								if(Image_Data[pick_line][pixel+i] < Cmp)
									flag++;
                                                                //uart_printf(test_port, "测试：%d\n",flag);
						  }
						  if(flag>3)//如果这四个点都是黑色
						  {
								
                                                                Bline_right[pick_line]=pixel;//那么先记录此处为右侧边线
								break;
						  }
					  }
					  else//到了最边缘的7个像素
					  {
							i=pixel;
							while(i<(V-1))//直接找到最边缘，看有多少个白色点
							{
								if(Image_Data[pick_line][i+1] < Cmp)
									flag++;
								i++;
							 }
							if(flag>0)//只要有一个黑色点的话，那么这个点的位置就是变现
							{
							 	Bline_right[pick_line]=pixel;
								break;
							}
					  }
				 }
			}
                        
			if(Bline_right[pick_line]>=V || pixel>=V)//如果一直找到边缘，
			{
			  Bline_right[pick_line]=V-1;//那么右侧的线就设置在中央
			  Pick_flag[pick_line]|=RIGHT_LOST_W;//0x4u
			}
                        
			for(pixel=start_pixel;pixel>0;pixel--)//左侧同上
			{
				if(Image_Data[pick_line][pixel] < Cmp)
				{
					  flag=0;
					  if(pixel>3)
					  {
						  for(i=0;i<4;i++)
						  {
								if(Image_Data[pick_line][pixel-i] < Cmp)
									flag++;
                                                                //uart_printf(test_port, "测试：%d\n",flag);
                                                  }
						  if(flag>3)
						  {
								 Bline_left[pick_line]=pixel;
								 break;
						  }
					  }
					  else
					  {
							i=pixel;
							while(i>0)
							{
								if(Image_Data[pick_line][i-1] < Cmp)
									flag++;
								i--;
							 }
                                                        
							if(flag>0)
			1				;{
								 Bline_left[pick_line]=pixel;
								 break;
							}
					  }
				}
			}
			if(Bline_left[pick_line]<=0 || pixel<=0)
			{
			  Bline_left[pick_line]=0;
			  Pick_flag[pick_line]|=LEFT_LOST_W;//0x1u
			}
                        
                        
                        
                        
                        
                        
			if(Bline_left[pick_line]==0&&Bline_right[pick_line]==V-1)//左右边界都未找到
			{
				temp_lost_line++;
				Pick_flag[pick_line]|=ALL_LOST_W;//0x10u
			}
			else if(Bline_left[pick_line]<(V-100)&&Bline_right[pick_line]>100)//
			{
			    if(last_zhidao_flag==1)
					if((Bline_left[pick_line]-Bline_right[pick_line])<100)
					{
					  //start_end_flag=1;
					}
			}
                        
			//else
			{
			  Bline_diff=abs_sub(Bline_left[pick_line],Bline_right[pick_line]);
			}
	  }
  
  //为跟线搜索算法参数赋初值
  
  last_left_line=Bline_left[H-4];
  last_right_line=Bline_right[H-4];
  if(temp_lost_line>0)
	return temp_lost_line;
  else
  	return 0;
}

/*******************************************************************************
函数名称：PickCenter_near
函数功能: 从中心寻线法
参数：
*******************************************************************************/
int32 PickCenter_m(int32 start_pixel,uint16 pick_line)
{
	int32 pixel,flag=0;//,temp1,temp2;
  	//uint32 Black_count_left=0,Black_count_right=0;
  	//uint16 pick_line=0,temp_line;
  	uint16 temp_lost_line=0;
  	//int32 start_pixel;
	//uint16 left_lost_flag=0,right_lost_flag=0;
	//const int Cmp_D=25;
	//const int  MAX=240;
  	uint32 i;
  	//pick_line=line;
	///////////////////////////////////////////////////////////////////////////
  	{
	  ///////////////////////////////左边线///////////////////////////////////
		for(pixel=start_pixel;pixel<V;pixel-=2)
		{
			if(Image_Data[pick_line][pixel] == 1)
			{
				flag=0;
				if(pixel>4)
				{
					for(i=1;i<4;i--)
					{
						if(Image_Data[pick_line][pixel-i] == 1)
							flag++;
					 }
					if(flag>3)
					{
						Bline_left[pick_line]=pixel;
						break;
					}
				}
				else
				{
					i=pixel;
					while(i>0)
					{
						if(Image_Data[pick_line][i+1] == 1)
							flag++;
						i--;
					}
					if(flag>1)
					{
						Bline_left[pick_line]=pixel;
						break;
					}
				}
			}
		}
		if(pixel<=0)
		{
		  Bline_left[pick_line]=1;
		}
		else//找到黑线 后期要改为判断左右黑线关系后再做赋值
		{
		  last_left_line = Bline_left[pick_line];
		}
		/////////////////////右边线///////////////////
		for(pixel=start_pixel;pixel>0;pixel+=2)
		{
			if(Image_Data[pick_line][pixel] == 1)
			{
				flag=0;
				if(pixel<V-5)
				{
					for(i=1;i<4;i++)
					{
						if(Image_Data[pick_line][pixel+i] == 1)
							flag++;
					}
					if(flag>3)
					{
						Bline_right[pick_line]=pixel;
						break;
					}
				}
				else
				{
					i=pixel;
					while(i<V)
					{
						if(Image_Data[pick_line][i-1] == 1)
							flag++;
						i++;
					}
					if(flag>1)
					{
					    Bline_right[pick_line]=pixel;
						break;
					}
				 }
			}
		}
		if(pixel<=0)
		{
		  Bline_right[pick_line]=0;
		}
		else
		{
		  last_left_line=Bline_right[pick_line];
		}
		if(Bline_left[pick_line]==0&&Bline_right[pick_line]==V-1)//左右边界都未找到
		{
			temp_lost_line++;
		}
	}
 
	  //为跟线搜索算法参数赋初值
	if(temp_lost_line==0)
	{
		return 0;
	}
	else
		return 1;
}
/*******************************************************************************
函数名称：PickCenter_diff_advance
函数功能: 跟踪寻线
参数：line 寻找行
*******************************************************************************/
void PickCenter_diff_advance(uint16 line)
{
    uint16 pixel;
    uint16 left_start;
    uint16 right_start;
    //uint16 left_end;//左边线寻线结束边界
    //uint16 right_end;//右边线寻线结束边界
    uint16 i;
    uint8 flag=0;//黑线判断标志
    volatile uint16 left_lost_flag=0,right_lost_flag=0;
    //const int Cmp_D=25;
    //const int  MAX=240;
  //////////////////////////////寻线算法/////////////////////////////////////

    if(last_left_line <= 3)//如果上次太靠近边缘，就当做是这一侧没了
    {
          left_start = 0;
          last_left_line = 0;
          Bline_left[line] = 0;
          left_lost_flag = 1;
          Lost_left_count++;
          Pick_flag[line] |= LEFT_LOST_W;
    }
    
    else
    {
          left_start = (last_left_line+last_right_line)/2;//从下面一行左右边界的一半处找
          if(left_start >= V)
          {
            left_start = V-10;
          }
          
          for(pixel=left_start;pixel >= 0;pixel--)//从上次的赛道中央找到最左侧
          {
                      if(Image_Data[line][pixel] < Cmp)//如果小于阈值的话
                      {
                              flag=0;
                              if(pixel>4)//如果离边缘还有4个以上像素，往左边看4个，如果4个都小于阈值，定为左边线
                              {
                                      for(i=0;i<4;i++)
                                      {
                                              if(Image_Data[line][pixel-i] < Cmp)
                                                      flag++;
                                      }
                                      if(flag>3)
                                      {
                                              Bline_left[line]=pixel;
                                              break;
                                      }
                               }
                              else//如果已经找到了旁边
                              {
                                      i=pixel;
                                      while(i>1)//一直看到最旁边的位置
                                      {
                                              if(Image_Data[line][i-1] < Cmp)
                                                      flag++;
                                              i--;
                                      }
                                      if(flag>1)//发现了1个以上就定为左边缘
                                      {
                                              Bline_left[line]=pixel;
                                              break;
                                      }
                              }
                      }
            }
          
          
          
            if(Bline_left[line]<=0)//如果找到头还没有，那就定为0
            {
                  Bline_left[line]=0;
                  last_left_line = Bline_left[line];
            }
            else//找到黑线 后期要改为判断左右黑线关系后再做赋值
            {
                  last_left_line = Bline_left[line];
            }
          
          
    }
    
    
    if(last_right_line >= V-3)
    {
          right_start = V-1;
          last_right_line = V-1;
          Bline_right[line] = V-1;
          right_lost_flag = 1;
          Lost_right_count++;
          Pick_flag[line] |= RIGHT_LOST_W;
    }
    else
    { 
          right_start = (last_right_line+last_left_line)/2;
          for(pixel=right_start;pixel<V;pixel++)
          {
                 if(Image_Data[line][pixel] < Cmp)
                 {
                         flag=0;
                         if(pixel< V-4)
                          {
                               for(i=0;i<4;i++)
                               {
                                  if(Image_Data[line][pixel+i] < Cmp)
                                        flag++;
                               }
                               if(flag>3)
                               {
                                  Bline_right[line]=pixel;
                                  break;
                                }
                         }
                          else
                          {
                                i=pixel;
                                while(i < V)
                                {
                                        if(Image_Data[line][i+1] < Cmp)
                                            flag++;
                                        i++;
                                } 
                                if(flag > 1)
                                {
                                      Bline_right[line]=pixel;
                                      break;
                                }
                          }
                   }
            }
          
        
          if(Bline_right[line] >= V)
          {
              Bline_right[line]=V-1;
              last_right_line = Bline_right[line];
          }
          else//找到黑线 后期要改为判断左右黑线关系后再做赋值
          {
              last_right_line = Bline_right[line];
           }
          
     }
    
  

  //以上算法的结果正常如果不丢线应该没问题。
  //如果第一次丢线，在这一做法中，把要丢的线预先置位了

  if(Bline_left[line]==0&&Bline_right[line]==V-1)//左右边界都未找到
  {
        //temp_lost_line++;
        Pick_flag[line] |= ALL_LOST;
        Lost_Line_count++;
  }
  /*if(Bline_left[line]==0&&Bline_right[line]>150){
        Bline_right[line] = Bline_right[line]/2;
  }*/
  if(Bline_left[line] - Bline_right[line] <= 20 && Bline_left[line] - Bline_right[line] >= -20)//左右之间的赛道间隔过于近，也是无效行
  {
        Pick_flag[line] |= ALL_LOST;
        Lost_Line_count++;
  }
  Bline_diff=abs_sub(Bline_left[line],Bline_right[line]);//记录这一行赛道宽度



}
/*******************************************************************************
函数名称：PickCenter_diff
函数功能: 跟踪寻线
参数：line 寻找行
*******************************************************************************/
void PickCenter_diff(uint16 line)
{
  uint16 pixel;
  uint16 left_start;//
  uint16 right_start;//
  uint16 left_end;//左边线寻线结束边界
  uint16 right_end;//右边线寻线结束边界
  uint16 i;
  uint8 flag=0;//黑线判断标志
  volatile uint16 left_lost_flag=0,right_lost_flag=0;
  //const int Cmp_D=25;
  //const int  MAX=240;
  //////////////////////////////寻线算法/////////////////////////////////////
  {
	    if(last_left_line>IMG_DIFF)
	    	left_start=last_left_line+IMG_DIFF;
		else
		    left_start=V/2;
            //uart_printf(test_port, "左边起始：%d\n",last_left_line);
            //uart_printf(test_port, "设置值：%d\n",IMG_DIFF);
                if(Bline_left[line-1]<(V-IMG_DIFF)){
	    	left_end=last_left_line-IMG_DIFF;//取上一行黑线偏右IMG_DIFF列坐标 //之后修改
                left_end = 0;}
	    else
		left_end= 1;
		if((Image_Data[line][left_start] == 1)&&(Image_Data[line][left_start-1] == 1))
		{
		  	pixel=left_start;
		}
		else
		{
		  	for(pixel=left_start;pixel<left_end;pixel--)
			{
				if(Image_Data[line][pixel-1] == 0&&Image_Data[line][pixel] == 1) //找到边界
				{
				   flag=0;
				   for(i=0;i<3;i++)
				   {
					 if(Image_Data[line][pixel+i] == 1)
					   flag++;
				   }
					if(flag>1)
					{
						Bline_left[line]=pixel;
						last_left_line=pixel;
						break;
					}
				}
			}
		}
		if(pixel==left_start)
		{
		  	Pick_flag[line] |= LEFT_LOST_B;
		  	//Bline_left[line]=last_left_line;
			//if(abs_sub(Bline_left[line-1],Bline_left[line-2])>5)
				Bline_left[line]=left_start;
			//else
			    //Bline_left[line]=Bline_left[line-1];
		 	last_left_line=Bline_left[line];
		  	left_lost_flag=1;
		  	Lost_left_count++;
		}  
		else if(pixel==left_end)//未找到黑线
		{
		  	Pick_flag[line] |= LEFT_LOST_W;
			Bline_left[line]=last_left_line;
			left_lost_flag=1;
			Lost_left_count++;
		}
/////////////////////////////////////////////////////////////////////////////////
		if(last_right_line+IMG_DIFF < V)
			right_end=last_right_line+IMG_DIFF;
		else
		    right_end=V;
		right_start=last_right_line-IMG_DIFF;
		if(right_start>(V-1))
		{
			 right_start=V-1;
		}//注意是否可去掉
		if((Image_Data[line][right_start] == 1)&&(Image_Data[line][right_start-1] == 0))
		{
			pixel=right_start;
		} 
		else
		{
		  	for(pixel=right_start;pixel>right_end;pixel++)
	  		{
				if(Image_Data[line][pixel] == 1)
				{
				   flag=0;
				   for(i=0;i<3;i++)
				   {
					 if(Image_Data[line][pixel-i] == 1)
					   flag++;
				   }
					if(flag>1)
					{
						Bline_right[line]=pixel;
						last_right_line=pixel;
						break;
					}
				}
	  		}
		}
		if(pixel==right_start)
		{
		  	Pick_flag[line] |= RIGHT_LOST_B;
			//Bline_right[line]=last_right_line;
			//if(abs_sub(Bline_right[line-1],Bline_right[line-2])>5)
				Bline_right[line]=right_start;
			//else
			   // Bline_right[line]=Bline_right[line-1];
			last_right_line=Bline_right[line];
			right_lost_flag=1;
			Lost_right_count++;
		}
		else if(pixel==right_end)//未找到右边黑线
		{
		  	Pick_flag[line] |= RIGHT_LOST_W;
			Bline_right[line]=last_right_line;
			right_lost_flag=1;
			Lost_right_count++;
		}
  }
  if(line<35)
  {
	 if(last_zhidao_flag==1)
	 if(run_time>25)
	 {
		test_start_line(line);
	 }
  }
  if((right_lost_flag==1)&&(left_lost_flag==1))
  {
	Lost_Line_count++;
	if((Pick_flag[line]&LEFT_LOST_W)&&(Pick_flag[line]&RIGHT_LOST_W))
		Pick_flag[line] |= ALL_LOST_W;
  }
  //else
  {
	Bline_diff=abs_sub(Bline_left[line],Bline_right[line]);
  }
}
/*******************************************************************************
函数名称：PickCenter_up
函数功能: 寻找中心线函数升级
参数：line 寻找行
*******************************************************************************/
void PickCenter_up(uint16 line)
{
  uint16 pixel,flag=0,temp1,temp2;
  uint8 i;
  uint8 right_lost_flag=0,left_lost_flag=0;
  if(PickCenter_flag<5)
  {
		for(pixel=V/2;pixel<V;pixel++)
		{
			if(video[line][pixel]<Cmp)
			{
			      flag=0;
				  for(i=1;i<5;i++)
				  {
					if(video[line][pixel+i]<Cmp)
					  flag++;
				  }
				  if(flag>3)
				  {
					Bline_left[line]=pixel;
					break;
				  }
			 }
		}
		if(pixel==V)
			Bline_left[line]=V-1;
		for(pixel=V/2;pixel>0;pixel--)
	  	{
			if(video[line][pixel]<Cmp)
			{
			      flag=0;
			  	  if(pixel>4)
				  {
					for(i=1;i<5;i++)
				  	{
						if(video[line][pixel-i]<Cmp)
					  		flag++;
				  	}
				  }
				  if(flag>3)
				  {
					 Bline_right[line]=pixel;
					 break;
				  }
			}
	  	}
	  	if(pixel==0)
		 Bline_right[line]=0;
  }
  else if(PickCenter_flag<50)//此处有修改看是否会使提取过程出错
  {
	    if(Bline_left[line-1]<270)
	    	temp1=Bline_left[line-1]+5;//取上一行黑线偏右IMG_DIFF列坐标
		else
		  	temp1=(V-1);
		for(pixel=Bline_left[line-1]-5;pixel<temp1;pixel++)
		{
			if(video[line][pixel]<Cmp)
			{
			      flag=0;
				  for(i=1;i<5;i++)
				  {
					if(video[line][pixel+i]<Cmp)
					  flag++;
				  }
				  if(flag>2)
				  {
					Bline_left[line]=pixel;
					break;
				  }
			 }
		}
		if(pixel==temp1)
		{
			Bline_left[line]=Bline_left[line-1];
			left_lost_flag=1;
		}
		if(Bline_right[line-1]>IMG_DIFF)
			temp2=Bline_right[line-1]-5;
		else
		    temp2=0;
		for(pixel=Bline_right[line-1]+5;pixel>temp2;pixel--)
	  	{
		  	if(pixel>278)
			{
			  pixel=(V-1);
			}//注意是否可去掉
			if(video[line][pixel]<Cmp)
			{
			      flag=0;
			   	  if(pixel>4)
				  {
					  for(i=1;i<5;i++)
					  {
						if(video[line][pixel-i]<Cmp)
						  flag++;
					  }
				  }
				  if(flag>2)
				  {
					Bline_right[line]=pixel;
					break;
				  }
			}
			
	  	}
	  if(pixel==temp2)
	  {
		Bline_right[line]=Bline_right[line-1];
		right_lost_flag=1;
	  }
  }
  else //此处有修改看是否会使提取过程出错
  {
	    if(Bline_left[line-1]<270)
	    	temp1=Bline_left[line-1]+5;//取上一行黑线偏右IMG_DIFF列坐标
		else
		  	temp1=(V-1);
		for(pixel=Bline_left[line-1]-5;pixel<temp1;pixel++)
		{
			if(video[line][pixel]<Cmp)
			{
			      flag=0;
				  for(i=1;i<5;i++)
				  {
					if(video[line][pixel+i]<Cmp)
					  flag++;
				  }
				  if(flag>2)
				  {
					Bline_left[line]=pixel;
					break;
				  }
			 }
		}
		if(pixel==temp1)
		{
		  Bline_left[line]=Bline_left[line-1];
		  left_lost_flag=1;
		}
		if(Bline_right[line-1]>IMG_DIFF)
			temp2=Bline_right[line-1]-5;
		else
		    temp2=0;
		for(pixel=Bline_right[line-1]+5;pixel>temp2;pixel--)
	  	{
		  	if(pixel>278)
			{
			  pixel=(V-1);
			}
			if(video[line][pixel]<Cmp)
			{
			      flag=0;
			       if(pixel>4)
				  {
					  for(i=1;i<5;i++)
					  {
						if(video[line][pixel-i]<Cmp)
						  flag++;
					  }
				  }
				  if(flag>2)
				  {
					Bline_right[line]=pixel;
					break;
				  }
			}
			
	  	}
	  if(pixel==temp2)
	  {
		Bline_right[line]=Bline_right[line-1];
		right_lost_flag=1;
	  }
  }
  if(((right_lost_flag==1)&&(left_lost_flag==1))||(Bline_left[line] - Bline_right[line] <= 10 && Bline_left[line] - Bline_right[line] >= -10))
  {
	Lost_Line_count++;
  }
  PickCenter_flag++;
}
/*******************************************************************************
函数名称：PickCenter_up
函数功能: 寻找中心线函数升级
参数：line 寻找行
*******************************************************************************/
void PickCenter_new()
{
  uint16 pixel,flag=0,temp1,temp2;
  uint16 line=0;
  PickCenter_flag=0;
  uint8 i;
  for(line=0;line<H;line++)
  {
		if(line<5)
	  {
			for(pixel=V/2;pixel<V;pixel++)
			{
				if(video[line][pixel]<Cmp)
				{
					  flag=0;
					  for(i=1;i<5;i++)
					  {
						if(video[line][pixel+i]<Cmp)
						  flag++;
					  }
					  if(flag>3)
					  {
						Bline_lefts[line].coord=pixel;
						Bline_lefts[line].line=line;
						break;
					  }
				 }
			}
			if(pixel==V)
			{
			  Bline_lefts[line].coord=V-1;
			  Bline_lefts[line].line=line;
			}
			for(pixel=V/2;pixel>0;pixel--)
			{
				if(video[line][pixel]<Cmp)
				{
					  flag=0;
					  if(pixel>4)
					  {
						for(i=1;i<5;i++)
						{
							if(video[line][pixel-i]<Cmp)
								flag++;
						}
					  }
					  if(flag>3)
					  {
						 Bline_rights[line].coord=pixel;
						 Bline_rights[line].line=line;
						 break;
					  }
				}
			}
			if(pixel==0)
			{
			  Bline_rights[line].coord=0;
			  Bline_rights[line].line=line;
			}
	  }
	  else if(PickCenter_flag<30)//此处有修改看是否会使提取过程出错
	  {
			if(Bline_left[line-1]<(V+IMG_DIFF))
				temp1=Bline_left[line-1]+IMG_DIFF;//取上一行黑线偏右IMG_DIFF列坐标
			else
				temp1=V;
			for(pixel=Bline_left[line-1]-IMG_DIFF;pixel<temp1;pixel++)
			{
				if(video[line][pixel]<Cmp)
				{
					  flag=0;
					  for(i=1;i<5;i++)
					  {
						if(video[line][pixel+i]<Cmp)
						  flag++;
					  }
					  if(flag>2)
					  {
						Bline_left[line]=pixel;
						break;
					  }
				 }
			}
			if(pixel==temp1)
			{
				Bline_left[line]=Bline_left[line-1];
			}
			if(Bline_right[line-1]>IMG_DIFF)
				temp2=Bline_right[line-1]-IMG_DIFF;
			else
				temp2=0;
			for(pixel=Bline_right[line-1]+IMG_DIFF;pixel>temp2;pixel--)
			{
				if(pixel>(V-1))
				{
				  pixel=V;
				}//注意是否可去掉
				if(video[line][pixel]<Cmp)
				{
					  flag=0;
					  if(pixel>4)
					  {
						  for(i=1;i<5;i++)
						  {
							if(video[line][pixel-i]<Cmp)
							  flag++;
						  }
					  }
					  if(flag>2)
					  {
						Bline_right[line]=pixel;
						break;
					  }
				}
				
			}
		  if(pixel==temp2)
			 Bline_right[line]=Bline_right[line-1];
	  }
	  else //此处有修改看是否会使提取过程出错
	  {
			if(Bline_left[line-1]<270)
				temp1=Bline_left[line-1]+IMG_DIFF;//取上一行黑线偏右IMG_DIFF列坐标
			else
				temp1=(V-1);
			for(pixel=Bline_left[line-1]-IMG_DIFF;pixel<temp1;pixel++)
			{
				if(video[line][pixel]<Cmp)
				{
					  flag=0;
					  for(i=1;i<5;i++)
					  {
						if(video[line][pixel+i]<Cmp)
						  flag++;
					  }
					  if(flag>2)
					  {
						Bline_left[line]=pixel;
						break;
					  }
				 }
			}
			if(pixel==temp1)
			Bline_left[line]=Bline_left[line-1];
			if(Bline_right[line-1]>IMG_DIFF)
				temp2=Bline_right[line-1]-IMG_DIFF;
			else
				temp2=0;
			for(pixel=Bline_right[line-1]+IMG_DIFF;pixel>temp2;pixel--)
			{
				if(pixel>278)
				{
				  pixel=(V-1);
				}
				if(video[line][pixel]<Cmp)
				{
					  flag=0;
					   if(pixel>4)
					  {
						  for(i=1;i<5;i++)
						  {
							if(video[line][pixel-i]<Cmp)
							  flag++;
						  }
					  }
					  if(flag>2)
					  {
						Bline_right[line]=pixel;
						break;
					  }
				}
				
			}
		  if(pixel==temp2)
			 Bline_right[line]=Bline_right[line-1];
	  }
	  PickCenter_flag++;
  }
}
/*******************************************************************************
函数名称：test_center
函数功能: 找到中心线矫正摄像头用
参数：
*******************************************************************************/
void test_center()
{
  uint8 line,center=0,put[3]={0};
  for(line=0;line<H;line++)
  {
	if(abs_sub(Pick_table[line],(V/2))<2)
	     center++;
  }
	//LCD_Print(8,6,"center:");
    if(run_time%5==0)
 		char_change_1(center,put);
	delay2();
	//LCD_Print(72,6,put);
}

/*******************************************************************************
函数名称：find_shizi()
函数功能: 寻找十字道
参数：line 出现十字道特征行 
*******************************************************************************/
int32 find_shizi(uint8 line)
{
  uint16 temp_line,left_line=0,right_line=0;
  //int32 guai_dian=0;
  //int temp_left_flag=0,temp_right_flag=0;
  //int diff_left=0,diff_right=0;
  int diff_max=0;
  int left_min=(V-1),right_max=0;
  for(temp_line=1;temp_line<line;temp_line++)
  {
	if(Bline_left[temp_line]<left_min)
	{
	  left_min=Bline_left[temp_line];
	  left_line=temp_line;
	}
	if(Bline_right[temp_line]>right_max)
	{
	  right_max=Bline_right[temp_line];
	  right_line=temp_line;
	}
  }
  if(right_max>((V-1)-left_min))
  {
	if(Bline_right[right_line]>=Bline_right[right_line-1]&&((Bline_right[right_line]-Bline_right[right_line+2])>10))
	  diff_max=right_line;
  }
  else
  {
	if(Bline_left[left_line]<=Bline_left[left_line-1]&&((Bline_left[left_line]-Bline_left[left_line+2])<-10))
	  diff_max=left_line;
  }
  return diff_max;
  
}

/*******************************************************************************
函数名称：ti_jiaozheng_new
函数功能: 矫正图像的梯形失真新方法
参数：
*******************************************************************************/
void ti_jiaozheng_new()
{
  uint16 line;
  int temp_left,temp_right;
  
  for(line=0;line<H;line++)
  {
	if(Bline_right[line]<((V-1)-Bline_left[line]))
	  {
		temp_left=(5000*Bline_left[line]-730439)/(10000-45*(2*line+8))+130; //near
    	if(temp_left<0)
    	  Bline_left[line]=0;
    	else if(temp_left>(V-1))
    	  Bline_left[line]=(V-1);
		else
		  Bline_left[line]=temp_left;
		
		Pick_table[line]=Bline_left[line]-70;
		if(Bline_left[line]>V/2)
		{
		  Bline_right[line]=Bline_left[line]-V/2;
		}
		else 
		  Bline_right[line]=0;
	  }
	  else
	  {
		temp_right=(5000*Bline_right[line]-730439)/(10000-45*(2*line+8))+130; //near
    	if(temp_right<0)
    	  Bline_right[line]=0;
    	else if(temp_right>(V-1))
    	  Bline_right[line]=V-1;
		else
		  Bline_right[line]=temp_right;
		Pick_table[line]=Bline_right[line]+70;
		Bline_left[line]=(Bline_right[line]+V/2)%280;
	  }
	if(Bline_left[line]<10||Bline_right[line]>250)
	{
	  //valid_line=line;
	  break;
	}
  }
	  
}
/*******************************************************************************
函数名称：ti_jiaozheng
函数功能: 矫正图像的梯形失真
参数：
*******************************************************************************/
void ti_jiaozheng()
{
  uint16 line;
  int temp_left,temp_right;
  
  for(line=H-41;line<H-1;line++)//保存原始的左右边线提取结果
  {
	PrBline_left[line] = Bline_left[line];
	PrBline_right[line] = Bline_right[line];
  }
  
  for(line=H-41;line<H-1;line++)
  {
	   //////////////////////////////////左边线矫正////////////////////////////////
	    if(Bline_left[line]==(V-1))
		  Deal_flag[line] |= DEAL_LEFT_LOST;
    	temp_left=(5000*Bline_left[line]-520439)/(10000-60*(2*line+1))+100; //near
    	if(temp_left<0)
    	  Bline_left[line]=0;
    	else if(temp_left>(V-1))
    	  Bline_left[line]=(V-1);
		else
		  Bline_left[line]=temp_left;
		//////////////////////////////////右边线矫正////////////////////////////////
		if(Bline_right[line]==0)
		  Deal_flag[line] |= DEAL_RIGHT_LOST;
    	temp_right=(5000*Bline_right[line]-520439)/(10000-60*(2*line+1))+100; //near
    	if(temp_right<0)
    	  Bline_right[line]=0;
    	else if(temp_right>(V-1))
    	  Bline_right[line]=(V-1);
		else
		  Bline_right[line]=temp_right;
  }
  
}
/*******************************************************************************
函数名称：duoji_control
函数功能: 舵机控制
参数：
*******************************************************************************/
void duoji_control(uint16 jiaodu,uint8 way)
{
  if(way==left_way)//0
  {
    //这个地方改为舵机向左偏jiaodu
    ftm_pwm_duty(FTM3, FTM_CH1,jiaodu);
    //FTM1_C0V=jiaodu;//dj_center-(jiaodu-dj_center);
    //GPIOC_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(10));	//IO口输出亮低电平,左方向灯亮
    //GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(2));//IO口输出亮高电平
	//GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));//IO口输出亮高电平
  }
  else if(way==right_way)//2
  {
    ftm_pwm_duty(FTM3, FTM_CH1,jiaodu);
    //这个地方改为舵机向右偏jiaodu
        //FTM1_C0V=jiaodu;//dj_center+dj_center-jiaodu;
	//GPIOD_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(2));	//IO口输出亮低电平
	//GPIOC_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(10));	//IO口输出亮高电平
	//GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));//IO口输出亮高电平
  }
  else//way==center_way  1
  {  
    ftm_pwm_duty(FTM3, FTM_CH1,822);//正位置的pwm  ？885or 870
    //这个地方改为舵机正中
    //FTM1_C0V=dj_center;//此处注意舵机数组修改 
    //GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(2));//IO口输出亮高电平
    //GPIOC_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(10));//IO口输出亮高电平
	 //center_led();
  }
  
}
/*******************************************************************************
函数名称：duoji_control
函数功能: 舵机控制备份
参数：
*******************************************************************************/
void duoji_control_backup(uint16 jiaodu,uint8 way)
{
  if(way==left_way)
  {
     FTM1_C0V=jiaodu;
     //GPIOB_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(21));	//IO口输出亮低电平,左方向灯亮
     //GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(4));//IO口输出亮高电平
	 //GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));//IO口输出亮高电平
  }
  else if(way==right_way)
  {
    FTM1_C0V=jiaodu;
    //GPIOD_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(4));	//IO口输出亮低电平
    //GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(21));	//IO口输出亮高电平
	//GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));//IO口输出亮高电平
  }
  else
  {  
     FTM1_C0V=dj_center;//此处注意舵机数组修改
     //GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(4));//IO口输出亮高电平
     //GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(21));//IO口输出亮高电平
	 //center_led();
  }
  
}
/*******************************************************************************
函数名称：center_led
函数功能: 中心线显示
参数：
*******************************************************************************/
void center_led()
{
  if(Pick_table[20]==Pick_table[12])
  {
	//if(Pick_table[12]==Pick_table[5])
	  //GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(20));//IO口输出亮低电平 LED5
  }
  else
  {
  	  //GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(20)); //IO口输出亮高电平 LED5
  }
}
/*******************************************************************************
函数名称：abs_sub
函数功能: 两个无符号数相减的绝对值
参数：
*******************************************************************************/
uint32 abs_sub(uint32 diff1,uint32 diff2)
{
  uint16 temp;
  if(diff1>diff2)
  {
	temp=diff1-diff2;
  }
  else
  {
	temp=diff2-diff1;
  }
  return temp;
}
/*******************************************************************************
函数名称：mult_f
函数功能: DSP单元相乘结果为浮点数
参数：
*******************************************************************************/
float mult_f(float mult1,float mult2)
{
  float temp;
  arm_mult_f32(&mult1, &mult2, &temp, 1);
  return temp;
}
/*******************************************************************************
函数名称：send_data_1
函数功能: 以字符形式发送数据
参数：
*******************************************************************************/
/*void send_data_1(short int data)
{
  uint8 temp1,temp2,temp3;
  uart_send1(UART0,'{');
  if(data<0)
  {
	data=-data;
	uart_send1(UART0,'-');
  }
  temp3=data/100+48;
  temp2=(data%100)/10+48;
  temp1=data%10+48;
  if(temp3!='0')
  	uart_send1(UART0,temp3);
  else
	uart_send1(UART0,' ');
  if(temp2!='0')
  	uart_send1(UART0,temp2);
  else
  	uart_send1(UART0,' ');
  uart_send1(UART0,temp1);
  uart_send1(UART0,'}');
  uart_send1(UART0,'\t');
}
/*******************************************************************************
函数名称：lvbo
函数功能: 滤波
参数：
*******************************************************************************/
void lvbo(uint8 num)
{
  uint8 line;
  for(line=4;line<50;line++)
  {
	if((Bline_left[line]>Bline_left[line-1])&&(Bline_left[line]>Bline_left[line+1]))
	{
	  if(((Bline_left[line]-Bline_left[line-1])>num)
		 &&((Bline_left[line]-Bline_left[line+1])>num))
	  
	  {
	  	Bline_left[line]=(Bline_left[line-1]+Bline_left[line+1])/2;
	  }
	}
	else if((Bline_left[line]<Bline_left[line-1])&&(Bline_left[line]<Bline_left[line+1]))
    {
	 if(((Bline_left[line-1]-Bline_left[line])>num)
			 &&((Bline_left[line+1]-Bline_left[line])>num))
	  {
			Bline_left[line]=(Bline_left[line-1]+Bline_left[line+1])/2;
	   }
	}
	
	if((Bline_right[line]>Bline_right[line-1])&&(Bline_right[line]>Bline_right[line+1]))
	{
	  if(((Bline_right[line]-Bline_right[line-1])>num)
		 &&((Bline_right[line]-Bline_right[line+1])>num))
	  
	  {
		Bline_right[line]=(Bline_right[line-1]+Bline_right[line+1])/2;
	  }
	}
	else if((Bline_right[line]<Bline_right[line-1])&&(Bline_right[line]<Bline_right[line+1]))
    {
	 if(((Bline_right[line-1]-Bline_right[line])>num)
			 &&((Bline_right[line+1]-Bline_right[line])>num))
	  {
			Bline_right[line]=(Bline_right[line-1]+Bline_right[line+1])/2;
	   }
	}
  }
}
/*******************************************************************************
函数名称：buzzer
函数功能: 蜂鸣器
参数：
*******************************************************************************/
void buzzer_on()
{
  buzzer_flag=1;
  buzzer_num=0;
}
/*******************************************************************************
函数名称：buzzer
函数功能: 蜂鸣器
参数：
*******************************************************************************/
void buzzer_ctl()
{
  if((buzzer_flag==1)&&(buzzer_num<30))
  {
	buzzer_num++;
	//GPIOC_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));	//IO口输出高电平
  }
  else
  {
	buzzer_flag=0;
	buzzer_num=0;
	//GPIOC_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(16));	//IO口输出低电平
  }
}
/*******************************************************************************
函数名称：center_buxian
函数功能: 滤波
参数：
*******************************************************************************/
void center_buxian()
{
  uint8 line;
  for(line=3;line<50;line++)
  {
	if((Pick_table[line]>Pick_table[line-1])&&(Pick_table[line]>Pick_table[line+1]))
	{
		Pick_table[line]=(Pick_table[line-1]+Pick_table[line+1])/2;
	}
	else if((Pick_table[line]<Pick_table[line-1])&&(Pick_table[line]<Pick_table[line+1]))
	{
		Pick_table[line]=(Pick_table[line-1]+Pick_table[line+1])/2; 
	}
  }
}
/*******************************************************************************
函数名称：bu_xian
函数功能: 滤波
参数：
*******************************************************************************/
void bu_xian()
{
  uint8 line;
  uint16 temp1,temp2;
  for(line=H-5;line>H-41;line--)
  {
	if((Bline_left[line]>Bline_left[line-1])&&(Bline_left[line]>Bline_left[line+1]))
	{
		temp1=(Bline_left[line-1]+Bline_left[line-2])/2;
	  	temp2=(Bline_left[line+1]+Bline_left[line+2])/2;
	  	Bline_left[line]=(temp1+temp2)/2;
	}
	else if((Bline_left[line]<Bline_left[line-1])&&(Bline_left[line]<Bline_left[line+1]))
	{
			temp1=(Bline_left[line-1]+Bline_left[line-2])/2;
	  		temp2=(Bline_left[line+1]+Bline_left[line+2])/2;
	  		Bline_left[line]=(temp1+temp2)/2;
	}
	
	if((Bline_right[line]>Bline_right[line-1])&&(Bline_right[line]>Bline_right[line+1]))
	{
		temp1=(Bline_right[line-1]+Bline_right[line-2])/2;
	  	temp2=(Bline_right[line+1]+Bline_right[line+2])/2;
	  	Bline_right[line]=(temp1+temp2)/2;
	}
	else if((Bline_right[line]<Bline_right[line-1])&&(Bline_right[line]<Bline_right[line+1]))
    {
			temp1=(Bline_right[line-1]+Bline_right[line-2])/2;
	  		temp2=(Bline_right[line+1]+Bline_right[line+2])/2;
	  		Bline_right[line]=(temp1+temp2)/2;
	}
  }
}
/*******************************************************************************
函数名称：lvbo_cu
函数功能: 滤波
参数：
*******************************************************************************/
void lvbo_cu(uint8 num)
{
  uint8 line;
  uint16 temp1,temp2;
  for(line=4;line<58;line++)
  {
	if(abs_sub(Bline_left[line],Bline_left[line-1])>num)
	{
	  temp1=(Bline_left[line-1]+Bline_left[line-2])/2;
	  temp2=(Bline_left[line+1]+Bline_left[line+2])/2;
	  Bline_left[line]=(temp1+temp2)/2;
	}
	if(abs_sub(Bline_right[line],Bline_right[line-1])>num)
	{
	  temp1=(Bline_right[line-1]+Bline_right[line-2])/2;
	  temp2=(Bline_right[line+1]+Bline_right[line+2])/2;
	  Bline_right[line]=(temp1+temp2)/2;
	}
  }
}
/*******************************************************************************
函数名称：send_some
函数功能: 发送些测试
参数：无
*******************************************************************************/
/*void send_some()
{
  uint8 line;
  for(line=0;line<H;line++)
	{
	  send_data(Bline_left[line]);

	  send_data(Bline_right[line]);
	  uart_send1(UART0,' ');
	}
	uart_send1(UART0,'\n');
}
/*******************************************************************************
函数名称：show_miss
函数功能: 关闭采集图像中断
参数：
*******************************************************************************/
void show_miss(uint8 state)
{
  	if(state==1)
	{
	  	//GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(9));//IO口输出亮低电平
		//GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(10));//IO口输出亮低电平
		//GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(11));//IO口输出亮低电平
	}
	else
	{
	  	//GPIOB_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(9));//IO口输出亮低电平
		//GPIOB_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(10));//IO口输出亮低电平
		//GPIOB_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(11));//IO口输出亮低电平
	}
}
/*******************************************************************************
函数名称：char_change
函数功能: 将显示字符转换为12864显示字符
参数：无
*******************************************************************************/
void char_change_1(uint8 num,uint8 *table)
{
  if(num<100)
  {
	*table=num/10+48;
  	*(table+1)=num%10+48;
	*(table+2)=' ';
  }
  else
  {
	*table=num/100+48;
 	*(table+1)=(num%100)/10+48;
  	*(table+2)=(num%100)%10+48;
  }
}
/*******************************************************************************
函数名称：lcd_int
函数功能: lcd显示整数
参数：无
*******************************************************************************/
/*void lcd_int(uint8 position,uint8 *string,int num)
{
  	uint8 put[5]={0};
  	if(num<0)
  	{
		put[0]='-';
		num=-num;
 	 }
  	else
  	{
		put[0]=' ';
  	}
	put[1]=(num%1000)/100+48;
	put[2]=(num%100)/10+48;
  	put[3]=(num%100)%10+48;
  	//LCD_Print(position,6,string);
  	//LCD_Print(position+24,6,put);
}
/*******************************************************************************
函数名称：char_change
函数功能: 将显示字符转换为12864显示字符
参数：无
*******************************************************************************/
void char_change_2(float f_num,uint8 *table)
{
  uint8 num;
  num=(uint8)f_num*10;
  if(num<100)
  {
	*table=num/10+48;
	*(table+1)='.';
  	*(table+2)=num%10+48;
  }
  else
  {
	*table=num/100+48;
 	*(table+1)=(num%100)/10+48;
  	*(table+2)=(num%100)%10+48;
  }
}
/*******************************************************************************
函数名称：char_change
函数功能: 将显示字符转换为12864显示字符
参数：无
*******************************************************************************/
void char_change(uint8 num,uint8 *table)
{

	*table=num/100+48;
 	*(table+1)=(num%100)/10+48;
  	*(table+2)=(num%100)%10+48;
}
/*******************************************************************************
函数名称：set_ov7620_ld
函数功能: 设置ov7620亮度
参数：无
*******************************************************************************/
/*void set_ov7620_ld()
{
  uint8 temp_key=0,temp_flag=0;
  uint8 put[3]={0};
  temp_key=gpio_get(PORTD,4);
  if(temp_key==0)
  {
	delay1(5);
	temp_key=gpio_get(PORTD,4);
	if(temp_key==0)
	{
	  if(liangdu>250)
		liangdu=255;
	  else
	  	liangdu=liangdu+5;
	  temp_flag=1;
	}
	while(gpio_get(PORTD,4)==0);
  }
  temp_key=gpio_get(PORTD,5);
  if(temp_key==0)
  {
	delay1(5);
	temp_key=gpio_get(PORTD,5);
	if(temp_key==0)
	{
	  if(liangdu<5)
		liangdu=0;
	  else
	  	liangdu=liangdu-5;
	  temp_flag=1;
	}
	while(gpio_get(PORTD,5)==0);
  }
  LCD_Print(64,0,"l:");
  char_change_1(liangdu,put);
  LCD_Print(80,0,put);
  if(temp_flag==1)
 	 set_ov7620(0x42,0x06,liangdu);//设置摄像头亮度 
}
/*******************************************************************************
函数名称：Init_7620
函数功能: 初始化ov7620
参数：无
*******************************************************************************/
/*void Init_7620()
{
  if(init_7620_flag==1)
  {
	set_ov7620(0x42,0x13,0x00);//取消自动曝光模式
	delay2();
	set_ov7620(0x42,0x06,liangdu);//设置摄像头亮度 
	delay2();
	set_ov7620(0x42,0x10,0x50);//设置摄像头曝光时间
	set_ov7620(0x42,0x14,0x20);//设置摄像头分辨率 bit5=1 QVGA 320*240 Bit5=0 VGA 640*480  //20//2c//3c
	LCD_Print(8,0,"ov7620_ok!");//显示初始化成功
	init_7620_flag=0;//清初始化标志
  }
}
/*******************************************************************************
函数名称：set_ov7620_bg
函数功能: 设置ov7620曝光时间
参数：无
*******************************************************************************/
/*void set_ov7620_bg()
{
  uint8 temp_key=0,temp_flag=0;
  uint8 put[3]={0};
  //temp_key=gpio_get(PORTD,4);
  if(temp_key==0)
  {
	delay1(5);
	//temp_key=gpio_get(PORTD,4);
	if(temp_key==0)
	{
	  if(baoguang>250)
		baoguang=255;
	  else
	  	baoguang=baoguang+2;
	  temp_flag=1;
	}
	//while(gpio_get(PORTD,4)==0);
  }
  //temp_key=gpio_get(PORTD,5);
  if(temp_key==0)
  {
	delay1(5);
	//temp_key=gpio_get(PORTD,5);
	if(temp_key==0)
	{
	  if(baoguang<3)
		baoguang=0;
	  else
	  	baoguang=baoguang-2;
	  temp_flag=1;
	}
	//while(gpio_get(PORTD,5)==0);
  }
  //LCD_Print(8,0,"l:");
  char_change_1(baoguang,put);
  //LCD_Print(24,0,put);
  if(temp_flag==1)
  {
 	// set_ov7620(0x42,0x10,baoguang);//设置摄像头曝光时间
  }
}
/*******************************************************************************
函数名称：display1
函数功能: 模式选择函数
参数：无
*******************************************************************************/
/*void display1()
{
  uint8 put_c[3];
  LCD_Print(8,0," CDU--Spower");  //汉字字符串显示
  LCD_Print(8,2,"s:");  //汉字字符串显示
  char_change(ideal_Bmq,put_c);
  LCD_Print(24,2,put_c);  //汉字字符串显示
  char_change(Bmq,put_c);
  LCD_Print(56,2,put_c); 
  char_change(CD_speed,put_c);//将字符每一位分别取到显示数组
  LCD_Print(88,2,put_c); 
  LCD_Print(8,4,"v_l:");  //汉字字符串显示
  char_change_1(valid_line,put_c);
  LCD_Print(48,4,put_c); 
  /*if(my_num>0)
  	char_change_1((uint8)my_num,put_c);
  else
	char_change_1((uint8)(-my_num),put_c);
  LCD_Print(80,4,put_c);
  if(roadFlag==0)
  	LCD_Print(72,4,"zhidao!");  //汉字字符串显示
  else if(roadFlag==1)
	LCD_Print(72,4,"wandao!");  //汉字字符串显示
  else
	LCD_Print(72,4,"shizidao!");  //汉字字符串显示
  if(xianshi==0)
  	lcd_int(8,"diff:",even_diff);
  else if(xianshi==1)
	;//lcd_int("xl:",D_slope);
  /*if(my_zhidao==1)
  	LCD_Print(88,6,"1");  //汉字字符串显示
  else
	LCD_Print(88,6,"0");  //汉字字符串显示*/
//}
/*******************************************************************************
函数名称：displayPar
函数功能: 参数显示函数
参数：无
*******************************************************************************/
/*void displayPar()
{
  char data[7]={0};
  LCD_P6x8Str(8,0," CDU--Spower");  //汉字字符串显示
  LCD_P6x8Str(8,1,"s:");  //汉字字符串显示
  sprintf(data,"%3d",ideal_Bmq);
  LCD_P6x8Str(24,1,(byte *)data);  //汉字字符串显示
  sprintf(data,"%3d",Bmq);
  LCD_P6x8Str(56,1,(byte *)data); 
  sprintf(data,"%3d",CD_speed);
  LCD_P6x8Str(88,1,(byte *)data); 
  LCD_P6x8Str(8,2,"v_l:");  //汉字字符串显示
  sprintf(data,"%3d",valid_line);
  LCD_P6x8Str(24,2,(byte *)data); 
  /*if(my_num>0)
  	char_change_1((uint8)my_num,put_c);
  else
	char_change_1((uint8)(-my_num),put_c);
  LCD_Print(80,4,put_c); 
  if(roadFlag==0)
  	LCD_P6x8Str(72,2,"zhidao!");  //汉字字符串显示
  else if(roadFlag==1)
	LCD_P6x8Str(72,2,"wandao!");  //汉字字符串显示
  else
	LCD_P6x8Str(72,2,"shizidao!");  //汉字字符串显示
  sprintf(data,"diff:%3d",even_diff);
  LCD_P6x8Str(8,3,(byte *)data);
  sprintf(data,"xl:%3d",D_slope);
  LCD_P6x8Str(68,3,(byte *)data);
  sprintf(data,"jd:%5d",jiaodu_num);
  LCD_P6x8Str(8,4,(byte *)data);
  /*if(my_zhidao==1)
  	LCD_Print(88,6,"1");  //汉字字符串显示
  else
	LCD_Print(88,6,"0");  //汉字字符串显示
}*/
/*******************************************************************************
函数名称：displayPar
函数功能: 参数显示函数
参数：无
*******************************************************************************/
/*void LCD_P6x8Int(byte x,byte y,byte ch[],int num)
{
  char data[7]={0};
  LCD_P6x8Str(x,y,ch);  //汉字字符串显示
  sprintf(data,"%3d",num);
  LCD_P6x8Str(x+24,y,(byte *)data);  //汉字字符串显示
}
/*******************************************************************************
函数名称：displayBlineSimple
函数功能: 显示黑线功能函数（简版）
参数：无
*******************************************************************************/
/*void displayBlineSimple()
{
	uint8 line;
	uint16 even_diff_temp;
	Image_clear();//
	even_diff_temp=V/2+even_diff;
	for(line=0;line<H;line++)
	{
		Image_set(0,line);
		if((Deal_flag[line]&INVALID_LINE)==0)
		{
		  	Image_set(Bline_left[line]/2+3,line);
			Image_set(Bline_right[line]/2+3,line);
			Image_set(Pick_table[line]/2+3,line);
			Image_set(even_diff_temp/2+3,line);
		}
		Image_set(V/2+6,line);
	}
	for(int i=48;i<64;i++)
		Image_set(i,(byte)(valid_line));
	if(temp_shizi==ROAD_SHIZI)
	{
	  for(int i=96;i<128;i++)
		Image_set(i,(byte)(valid_line));
	}
	  	//Image_set(i,(byte)(Shi_zi_num));
	Draw_MyImage();
}
/*******************************************************************************
函数名称：displayBinary
函数功能: 显示二值化赛道信息
参数：无
*******************************************************************************/
/*void displayBinary()
{
	uint8 line;
	uint16 pixel;
	Image_clear();//
	for(line=0;line<H;line++)
	{
	  	for(pixel=0;pixel<V;pixel+=2)
	  	{
		  if(video[line][pixel]>Cmp)
			Image_set(pixel/2,line);
		}
	}
	for(int i=96;i<128;i++)
		Image_set(i,(byte)(valid_line));
	for(int i=102;i<128;i++)
		Image_set(i,(byte)(H/2));
	Draw_MyImage();
}
/*******************************************************************************
函数名称：displayBline
函数功能: 显示黑线功能函数（繁版）
参数：无
*******************************************************************************/
/*void displayBline()
{
	uint8 line;
	uint16 even_diff_temp;
	Image_clear();//
	even_diff_temp=V/2+even_diff;
	for(line=0;line<H;line++)
	{
		Image_set(0,line);
		if((Deal_flag[line]&INVALID_LINE)==0)
		{
		  	Image_set(Bline_left[line]/2+3,line);
			Image_set(Bline_right[line]/2+3,line);
			Image_set(Pick_table[line]/2+3,line);
			Image_set(even_diff_temp/2+3,line);
		}
		Image_set(V/2+6,line);
	}
	for(line=60;line<64;line++)
		Image_set(V/4+3,line);
	for(line=0;line<5;line++)
		Image_set(V/4+3,line);
	for(int i=48;i<64;i++)
		Image_set(i,(byte)(valid_line));
	if(temp_shizi==ROAD_SHIZI)
	{
	  for(int i=96;i<128;i++)
		Image_set(i,(byte)(valid_line));
	}
	  	//Image_set(i,(byte)(Shi_zi_num));
	Draw_MyImage();
}
/*******************************************************************************
函数名称：test_lcd
函数功能: 测试液晶屏
参数：无
*******************************************************************************/
/*void test_lcd()
{
    LCD_Init();
	/*LCD_Fill(0xff);//黑屏 
	delay2();
	LCD_Fill(0x00);//亮屏
	delay2();
	put_array[0]=(uint8)Kp/10+48;
	put_array[1]=(uint8)Kp%10+48;
	LCD_Print(8,0," CDU--Spower");  //汉字字符串显示
	LCD_Print(8,2,put_array);  //汉字字符串显示
}
/*******************************************************************************
函数名称：dis_img_irq
函数功能: 关闭采集图像中断
参数：
*******************************************************************************/
/*void dis_img_irq()
{
  	disable_irq(87);
	disable_irq(88);
	disable_irq(0);
}
//give some delay~~  
void delay(uint32 t)
{
  uint32 i,j;
  for(i=0;i<t;i++)
    for(j=0;j<251;j++);
}
void delay1(uint8 t)
{
   uint8 i,j;
   for(i=0;i<t;i++)
	 for(j=0;j<10;j++)
	   asm("nop");   
}
void delay2()
{
   int i =0;	
   int j=0;
   for(i=0;i<100;i++)
	  for(j=0;j<1000;j++)
	      asm("nop");   
}*/
/////////////////////////////////////////////////////








/*******************************************************************************
函数名称：find_edge
函数功能: 先寻找Cmp，再寻找边缘，然后寻找中心线
参数：
*******************************************************************************/

void find_edge()
{
  
            if(!haveinited)
            {
              lastCmp = GetOSTU(Image_Data);
              haveinited = 1;
            }

            uint8 tmp = GetOSTU(Image_Data);
            if(tmp > lastCmp)
            {
              lastCmp++;
            }
            else if(tmp < lastCmp)
            {
              lastCmp--;
            }
            Cmp = lastCmp;
           // uart_printf(test_port,"Cmp = %d \n",Cmp);
  
                        for(int line=0;line<H;line++)
			{
                              Bline_left[line]=0;
                              Bline_right[line]=V-1;
                              Pick_table[line]=V/2;
                              Pick_flag[line]=0;//赛道标志清零
                              Deal_flag[line]=0;//处理标志清零
                              row_F[line]=0;//清除采集完成标志位
			}
                        
                        //uart_printf(test_port, "最大有效行：%d\n",valid_line);
			//TEST_IO_H;
    
			//uart_printf(test_port, "最大有效行：%d\n",valid_line);
                        
                        int j;
                        Lost_left_count        = 0;
                        Lost_right_count       = 0;
                        Lost_Line_count         = 0;
                        //获取最下面10行的中线
                        
                        
                        for(int i=H-1;i>H-41;i--)
                        {
                            //找左边线
                            if(i == H-1)
                            {
                                j=lastmiddleplace;
                            }
                            else
                            {
                                j=Pick_table[i+1];
                            }
                            if(j<=2)
                                j=2;
                            while(j>=2)
                            {
                                if(Image_Data[i][j]>Cmp && Image_Data[i][j-1]<Cmp && Image_Data[i][j-2]<Cmp)
                                {
                                    Bline_left[i]=j;
                                    break;
                                }
                                j--;
                            }
                            //找右边线
                            if(i == H-1)
                            {
                                j=lastmiddleplace;
                            }
                            else
                            {
                                j=Pick_table[i+1];
                            }
                            if(j>=V-3)
                                j=V-1;
                            while(j<=V-3)
                            {
                                if(Image_Data[i][j]>Cmp && Image_Data[i][j+1]<Cmp && Image_Data[i][j+2]<Cmp)
                                {
                                    Bline_right[i]=j;
                                    break;
                                }
                                j++;
                            }
                            //ti_jiaozheng();
                            //bu_xian();
                            
                            //记录丢线情况
                            if(Bline_left[i]!=0 && Bline_right[i]!=V-1)//没有丢线
                            {
                                Pick_table[i]=(Bline_left[i]+Bline_right[i])/2;
                            }
                            else if(Bline_left[i]==0 && Bline_right[i]!=V-1)//左边线丢了
                            {
                                Lost_left_count++; 
                                Pick_table[i]=Bline_right[i]/2;
                             }
                            
                            else if(Bline_left[i]!=0 && Bline_right[i]==V-1)//丢了右线,没有丢左线
                            {
                                Lost_right_count++;//记录只有右线丢的数量
                                Pick_table[i]=(Bline_left[i]+V-1)/2;
                            }
                            
                            else if(Bline_left[i]==0 && Bline_right[i]==V-1)//两边都丢了的话  
                            {
                                Lost_Line_count++;
                                if(i ==H-1)//如果是首行就以图像中心作为中点
                                {
                                     Pick_table[i] = V/2;
                                }       
                                else 
                                {
                                     Pick_table[i] = Pick_table[i+1];//如果不是首行就用上一行的中线作为本行中点
                                 }             
                            }
                            
                          //  uart_printf(test_port,"left:%d   middle:%d   right:%d \n ",Bline_left[i],Pick_table[i],Bline_right[i]);
                                 
                         
                        }
                        
                       // uart_printf(test_port,"\n\n\n");
                        
                        lastmiddleplace=Pick_table[H-1];
                        
                        
                        
                        /*int left_i=0;
                        int right_i=0;
                        int lost_max=0; 
                        int fix_middle=0;
                        while(Bline_left[left_i]<=2)
                          left_i++;
                        while(Bline_right[right_i]>=V-3)
                          right_i++;
                        if(left_i>right_i)
                        {
                            lost_max=left_i;
                        }
                        else
                        {
                            lost_max=right_i;
                        }
                        for(int i=H-1;left_i>0||right_i>0;i--)
                        {
                          
                            fix_middle=((Bline_left[i]-left_i)+(Bline_right[i]+right_i))/2;
                            if(fix_middle>V-1)
                              Pick_table[i]=V-1;
                            else if(fix_middle<0)
                              Pick_table[i]=0;
                            else
                              Pick_table[i]=fix_middle;
                            
                            if(left_i>0)
                              left_i--;
                            if(right_i>0)
                              right_i--;
                        }*/
                        
                        
                        
//                        //获取左边线和右边线
//                        int jj=0;//卡点寻找界限的左右边界
//                        for(int i=109;i>50;i--)
//                        {
//                                if(Bline_left[i+1]!=0 && Bline_right[i+1]!=V-1)//如果之前一行左右都有,左右卡定界限寻找，减少计算量
//                                {
//                                      j = ((Bline_left[i+1]+10) >= V-1)? V-1:(Bline_left[i+1]+10);
//                                      jj = ((Bline_left[i+1]-10) <= 0)? 0:(Bline_left[i+1]-10);     
//                
//                                      while(j >= jj)       
//                                      {       
//                                          if(Image_Data[i][j]>=Cmp && Image_Data[i][j-1]<Cmp && Image_Data[i][j-2]<Cmp)  
//                                          {
//                                              Bline_left[i] = j;
//                                              break;
//                                           }   
//                                           j--;    
//                                       }
//                                      
//                                       j = ((Bline_right[i+1]-10) <= 0)? 0:(Bline_right[i+1]-10); //在找右边界   
//                                       jj = ((Bline_right[i+1]+10) >= V-1)? V-1:(Bline_right[i+1]+10);    
//                                       while(j <= jj)             
//                                       {
//                                            if(Image_Data[i][j]>=Cmp && Image_Data[i][j+1]<Cmp && Image_Data[i][j+2] < Cmp) 
//                                            {
//                                                 Bline_right[i] = j;
//                                                 break;    
//                                            }
//                                           j++;
//                                       }
//                                }
//                                
//                                else if(Bline_left[i+1]!=0 && Bline_right[i+1]==V-1) //右侧丢线
//                                {
//                                  
//                                       j  = ((Bline_left[i+1]+10) >=V-1)? V-1:(Bline_left[i+1]+10);//左边界用边沿扫描   
//                                       jj = ((Bline_left[i+1]-10) <= 0)? 0:(Bline_left[i+1]-10);     
//                                       
//                                        while(j >= jj)   
//                                        {     
//                                            if(Image_Data[i][j]>=Cmp && Image_Data[i][j-1]<Cmp && Image_Data[i][j-2]<Cmp)
//                                            {
//                                                 Bline_left[i] = j;
//                                                 break;
//                                            }
//                                           j--;   
//                                        }     
//                                        
//                                        j = Pick_table[i+1];//上一行丢了右边界用全行扫描 
//                                        while(j <= V-1)      
//                                        {    
//                                            if(Image_Data[i][j]>=Cmp && Image_Data[i][j+1]<Cmp && Image_Data[i][j+2] < Cmp)
//                                            {
//                                                 Bline_right[i] = j;
//                                                 break;
//                                            }
//                                           j++;
//                                        }                    
//                                 }
//                                
//                                else if(Bline_left[i+1]==0 && Bline_right[i+1]!=V-1) //左侧丢线
//                                {
//                                  
//                                       j = ((Bline_right[i+1]-10) <= 1)? 0:(Bline_right[i+1]-10);//边缘追踪找右边界 
//                                       jj = ((Bline_right[i+1]+10) >= V-1)? V-1:(Bline_right[i+1]+10);   
//                                       
//                                        while(j <= jj)   
//                                        {     
//                                            if(Image_Data[i][j]>=Cmp && Image_Data[i][j-1]<Cmp && Image_Data[i][j-2]<Cmp)
//                                            {
//                                                 Bline_right[i] = j;
//                                                 break;
//                                            }
//                                            j++;   
//                                        }     
//                                        
//                                        j = Pick_table[i+1];//上一行丢了左边界用全行扫描 
//                                        while(j >=0)      
//                                        {    
//                                            if(Image_Data[i][j]>=Cmp && Image_Data[i][j+1]<Cmp && Image_Data[i][j+2] < Cmp)
//                                            {
//                                                 Bline_left[i] = j;
//                                                 break;
//                                            }
//                                            j--;
//                                        }                    
//                                 }
//                                else//如果上一行左右线都丢了
//                                {
//                                      j = Pick_table[i+1];   //找全行找左边界
//                                      while(j >= 0)  
//                                      {
//                                           if(Image_Data[i][j]>=Cmp && Image_Data[i][j-1]<Cmp && Image_Data[i][j-2] < Cmp)     
//                                           {
//                                                Bline_left[i] = j;
//                                                break;
//                                           }
//                                           j--;
//                                      }
//                                      j = Pick_table[i+1];   //全行找右边界   
//                                     while(j <= V-1)       
//                                     {   
//                                          if(Image_Data[i][j]>=Cmp && Image_Data[i][j+1]<Cmp && Image_Data[i][j+2] < Cmp)
//                                          {
//                                                Bline_right[i] = j;  
//                                                break;
//                                          }
//                                         j++;      
//                                     }   
//                                }
//                            
//                        }
                        
                        
}



int max(int a,int b)
{
  if(a>b) return a;
  else return b;
}


//选取关键列，从图像底部往上扫描，看停在哪里
int GetValidLine()//获取黑线截止行
{
  //标志位,如果已经从下找到最上面的黑白交界处 置1
  LEndFlag  = 0;
  MEndFlag  = 0;
  REndFlag  = 0;	
  MREndFlag = 0;
  MLEndFlag = 0;
  LLEndFlag = 0;
  RREndFlag = 0;

  //记录每一列找到最高的位置的纵坐标
  BlackEndMR   = 0;//清零
  BlackEndML   = 0;
  BlackEndLL   = 0;
  BlackEndRR   = 0;
  BlackEndL    = 0;
  BlackEndM    = 0;
  BlackEndR    = 0;
  
  for (int i = H-1; i >= 3 ; i--)
  {
        //MM
    	if(Image_Data[i][V/2] > Cmp && !MEndFlag )//!MEndFlag=1 //40
        {
		BlackEndM++;//中黑线截至行
        }
	else if((i > 1 && Image_Data[i-1][V/2] <=Cmp && Image_Data[i-2][V/2] <=Cmp)||(Image_Data[H-1][V/2]<=Cmp&&Image_Data[H-2][V/2]<=Cmp))//连续两行是黑色        
        {
		MEndFlag = 1;
        }
        //L
	if(Image_Data[i][V/4] > Cmp && !LEndFlag )//20
        {
		BlackEndL++;//左黑线截至行
        }
	else if((i > 1 && Image_Data[i-1][V/4]<=Cmp && Image_Data[i-2][V/4] <=Cmp)||(Image_Data[H-1][V/4]<=Cmp&&Image_Data[H-2][V/4]<=Cmp))
        {
		LEndFlag = 1;
        }
        //R
	if(Image_Data[i][V*3/4] > Cmp && !REndFlag )//60
        {
		BlackEndR++;//右黑线截至行
	}
	else if((i > 1 && Image_Data[i-1][V*3/4] <=Cmp && Image_Data[i-2][V*3/4] <=Cmp)||(Image_Data[H-1][V*3/4]<=Cmp&&Image_Data[H-2][V*3/4]<=Cmp))
        {
		REndFlag = 1;
        }
        //ML
        if(Image_Data[i][V*3/8]>Cmp && !MLEndFlag )
        {
		BlackEndML++;
        }
	else if((i > 1 && Image_Data[i-1][V*3/8] <=Cmp && Image_Data[i-2][V*3/8] <=Cmp)||(Image_Data[H-1][V*3/4]<=Cmp&&Image_Data[H-2][V*3/4]<=Cmp))
        {
		MLEndFlag = 1;
        }
        //MR
	if(Image_Data[i][V*5/8]>Cmp && !MREndFlag )
        {
		BlackEndMR++;
        }
	else if((i > 1 && Image_Data[i-1][V*5/8] <=Cmp && Image_Data[i-2][V*5/8] <=Cmp)||(Image_Data[H-1][V*5/8]<=Cmp&&Image_Data[H-2][V*5/8]<=Cmp))
        {
		MREndFlag = 1;
	}
        //LL
        if(Image_Data[i][V/8] >Cmp && !LLEndFlag )
        {
		BlackEndLL++;
	}
	else if((i > 1 && Image_Data[i-1][V/8] <=Cmp && Image_Data[i-2][V/8] <=Cmp)||(Image_Data[H-1][V/8]<=Cmp&&Image_Data[H-2][V/8]<=Cmp))
        {
		LLEndFlag = 1;
	}
        //RR
        if(Image_Data[i][V*7/8] >Cmp && !RREndFlag )
        {
		BlackEndRR++;
	}
	else if((i > 1 && Image_Data[i-1][V*7/8] <=Cmp && Image_Data[i-2][V*7/8] <=Cmp)||(Image_Data[H-1][V*7/8]<=Cmp&&Image_Data[H-2][V*7/8]<=Cmp))
        {
		RREndFlag = 1;
	}
   }
  
   
   int valid_line=0;
        
    
    valid_line =max(BlackEndL,BlackEndM);//取大值
    valid_line =max(valid_line,BlackEndR);
    valid_line =max(valid_line,BlackEndMR);
    valid_line =max(valid_line,BlackEndML);
    valid_line =max(valid_line,BlackEndLL);
    valid_line =max(valid_line,BlackEndRR);
        

        
    return valid_line;
    
}



void specialroad()
{
    //uart_printf(test_port,"%d  %d  %d  %d  %d  %d  %d \n",LLEndFlag,LEndFlag,MLEndFlag,MEndFlag,MREndFlag,REndFlag,RREndFlag);
    //uart_printf(test_port,"%d  %d  %d  %d  %d  %d  %d \n",BlackEndLL,BlackEndL,BlackEndML,BlackEndM,BlackEndMR,BlackEndR,BlackEndRR);
    //uart_printf(test_port,"-----------------------------------\n");
    
    bigbendflag=0;
    if(!BlackEndLL||!BlackEndRR)
    {
        bigbendflag=1;
        if((!BlackEndLL&&!BlackEndL)||(!BlackEndRR&&!BlackEndR))
        {
            bigbendflag=2;
            if((!BlackEndLL&&!BlackEndL&&!BlackEndML)||(!BlackEndRR&&!BlackEndR&&!BlackEndMR))
            {
                bigbendflag=3;
            }
        }
    }
    
}


/*******************************************************************************
函数名称：int CrossRecognition()  返回值0代表没有十字，1代表车头偏左入十字 2代表车头偏右入十字
函数功能: 以下为十字判断函数
参数：无
*****************************************************************************/

//判断画面左侧和右侧是否存在拐点


void findleftpoint()
{
  int i;
  leftpointflag=0;
  left=0;
  right=0;
   //左拐点，，判断5行之间的明显拐点，用于只剩下一点点三角的情况
  
//   for(i= H-1;i>H/4;i--) 
//   {
//     if(Bline_left[i]!=0&&Bline_left[i-1]!=0&&Bline_left[i-2]!=0&&Bline_left[i-3]!=0&&Bline_left[i-4]!=0  //连续5行不丢线，并且这行左边线都贴不到邮编
//        &&Bline_left[i]<V-1&&Bline_left[i-1]<V-1&&Bline_left[i-2]<V-1&&Bline_left[i-3]<V-1&&Bline_left[i-3]<V-1)  
//     {
//     
//      if((Bline_left[i]-Bline_left[i-1]<=-1)&&(Bline_left[i-1]-Bline_left[i-2]<=-1)&&(Bline_left[i-3]-Bline_left[i-2]<=-1)&&(Bline_left[i-4]-Bline_left[i-3]<=-1))//找到左拐点
//       {
//           leftpointflag=1;
//           break;
//        }
//     }
//   }
   
   //看左拐点的大趋势
   for(i = H-1;i>H/2;i--)
   {
      if(Bline_left[i] < Bline_left[i-1])//左侧边线 右拐
      {
          right++;
      }           
      else if(Bline_left[i] > Bline_left[i-1])//左侧边线 左拐
      {
          left++;
      }
                   
      //if(Bline_right[i] <= V-3)//发现右侧出现赛道时，结束
          //break;
   }
   
   
   if(left>5 && right>5)
   {
      leftpointflag=1;
   }
   
   //uart_printf(test_port,"***left：%d  right:%d    flag： %d ***\n\n\n",left,right,leftpointflag);
        
}



void findrightpoint()
{
  int i;
  rightpointflag=0;
  left=0;
  right=0;
   //左拐点
//   for(i= H-1;i>H/2;i--) 
//   {
//     if(Bline_right[i]!=0&&Bline_right[i-1]!=0&&Bline_right[i-2]!=0&&Bline_right[i-3]!=0&&Bline_right[i-4]!=0  //连续5行不丢线，并且这行左边线都贴不到邮编
//        &&Bline_right[i]<V-1&&Bline_right[i-1]<V-1&&Bline_right[i-2]<V-1&&Bline_right[i-3]<V-1&&Bline_right[i-3]<V-1)  
//     {
//     
//        if((Bline_right[i]-Bline_right[i-1]>=1)&&(Bline_right[i-1]-Bline_right[i-2]>=1)&&(Bline_right[i-3]-Bline_right[i-2]>=1)&&(Bline_right[i-4]-Bline_right[i-3]>=1))//找到左拐点
//        {
//           rightpointflag=1;
//        }
//     }
//   }
   
   //看右拐点的大趋势
   for(i = H-1;i>H/2;i--)
   {
      if(Bline_right[i] > Bline_right[i-1])//左侧边线 右拐
           left++;
                   
      else if(Bline_right[i] < Bline_right[i-1])//左侧边线 左拐
           right++;
                   
      //if(Bline_left[i] >=2)//发现右侧出现赛道时，结束
          //break;
   }
   if(left>5 && right>5)
      rightpointflag=1;
   
  
}

//判断画面下方左侧或者右侧是否为全白
void findleftwhite()
{
  leftwhiteline=0;
  if(Bline_left[H-1] <=2)
  {
      leftwhiteline = 1;
      for(int row = H-2; row > 80; row--)  //80  100
      {
          if(Bline_left[row] >2 )
          {
              leftwhiteline = 0;
          }
      }
   }  
}

void findrightwhite()
{
  rightwhiteline=0;
  if(Bline_right[H-1] >=V-4)
  {
      rightwhiteline = 1;
      for(int row = H-2; row > 80; row--)  //80 100
      {
          if(Bline_right[row] <V-4 )
          {
              rightwhiteline = 0;
          }
      }
   }  
//    for(int i=H-1;i>H-20;i--)
//    {
//        uart_printf(test_port,"row:%d\n",Bline_right[i]);
//    }
//  
//     uart_printf(test_port,"++++++++\n");
  
}



//判断是否有十字以及车体面对十字的状态，
int CrossRecognition()
{ 
  findleftpoint();
  findrightpoint();
  findleftwhite();
  findrightwhite();
  
  
  //uart_printf(test_port,"***leftpoint:%d  rightpoint:%d ***\n",leftpointflag,rightpointflag);
  
  //uart_printf(test_port,"***leftline:%d rightline:%d***\n\n\n",leftwhiteline,rightwhiteline);
  
  
  int flag=0;//1代表 直入十字 2代表车头偏右 3代表车头偏右
  //uart_printf(test_port,"Lost_left_count: %d Lost_right_count:%d \n",Lost_left_count,Lost_right_count);
  
  if(Lost_left_count>=10 && Lost_right_count>=10)//看到左右两侧均有明显丢行
  {
      flag=1;
      
  }
  else
  {
    if(leftpointflag&&rightwhiteline)//车头偏左
    {
        flag=2;
    }
    else if(rightpointflag&&leftwhiteline)//车头偏右
    {
        flag=3;
    }
  }
  
  return flag;
  
}


void CrossConduct(int flag)
{
  int i;
  int k;
  int m;
  int k1;
  int m1;
  int xielv;
  //直入十字可以分为两种情况 ：1.底部的线还存在，2.底部丢线
  
  if(flag==1)//如果是直入十字的情况
  {
  
     if((Bline_left[H-1]!=0&&Bline_left[H-2]!=0&&Bline_left[H-3]!=0&&Bline_left[H-4]!=0))//底部4行左面边不丢线
     {
        
         for(int line=H-5; line>10; line--)
         {
           if(Bline_left[line]>2)
             k=line;
           m=Bline_left[line];
            break;
         }
         xielv=k-(H-5)/m-Bline_left[H-5];
         for(int line1=H-5; line1<=k; line1++)
         {
           Bline_left[line1]=line1-k/xielv+m;
           Bline_right[line1]= Bline_left[line1]+V-20; 
         }
            
     }
         
     if(Bline_right[H-1]!=V-1&&Bline_right[H-2]!=V-1&&Bline_right[H-3]!=V-1&&Bline_right[H-4]!=V-1)//右面不丢下线
     {
         for(int line=H-5; line>10; line--){
           if(Bline_right[line]>2)
             k1=line;
           m1=Bline_right[line];
            break;
         }
         xielv=k1-(H-5)/m1-Bline_left[H-5];
         for(int line1=H-5; line1<=k1; line1++){
           Bline_right[line1]=line1-k1/xielv+m1;
            Bline_left[line1]=Bline_right[line1]-V+20;
         }
     }
     
     if(Bline_left[H-1]==0&&Bline_left[H-2]==0&&Bline_left[H-3]==0&&Bline_left[H-4]==0&&Bline_right[H-1]==V-1&&Bline_right[H-2]==V-1&&Bline_right[H-3]==V-1&&Bline_right[H-4]==V-1)
     //离十字很近，左右下面的线都丢了
     {
          //补线处理代码
         for(int line=H-5; line>10; line--){
           if(Bline_left[line]>2)
             k=line;
           m=Bline_left[line];
            break;
         }
         for(int line=H-5; line>10; line--){
           if(Bline_right[line]>2)
             k1=line;
           m1=Bline_right[line];
            break;
         }
         for(int line1=H-1; line1<=k; line1++){
           Bline_left[line1]=Bline_left[line1-1]+(m1-m)/2;
         }
         for(int line1=H-1; line1<=k1; line1++){
           Bline_right[line1]=Bline_right[line1-1]+(m1-m)/2;
         }
     }
 }
 
  else if(flag==1)//左斜入十字
  {
        //直接给舵机偏移？
  }
  
   else if(flag==2)//右斜入十字
  {
        
  
   }
}



//////////////////////////////////////////////////////////////环岛处理
/*int abs(int a,int b)
{
    if(a>b)
      return a-b;
    else
      return b-a;
}*/



void CircleRecognition()//识别环岛设置标志位
{
  
  
      //uart_printf(test_port,"state1:%d\n",state); 
     
      uint8 change[V];
      int min=V/2;
      int down=0;
      int i = 0;
      int up=0;
      uint8 sum=0;

  switch(state)
  {
      case 0:
        
            for(i = H-1;i>H-40;i--)
            {
              if(Pick_table[i+1]-Pick_table[i]>10)
              {
                readyrightflag = 1;
                state = 1;
              }
            }
            
            for(i = H-1;i>H-40;i--)
            {
              if(Pick_table[i+1]-Pick_table[i]<-10)
              {
                readyleftflag = 1;
                state = 1;
              }
            }
           
           break;
           
            
      case 1:
        
            if(readyrightflag)
            {
              pit_irq_dis(PIT0);
              PWMSetMotor2(0,0);
              delayms(3000);
              pit_irq_en(PIT0);
            }
                
                
                
      case 2:
        
            if(intoleftcircleflag)//如果进入左侧环岛
           
        break;
      
      //再次经过环岛处，看到尖角，此时判断条件减弱
      case 3:        
            
            //获取右侧画面最小值
            min=V/2;
            for(int i=V/2;i<V;i++)//提取画面右半部分黑白交界处的 纵坐标存储到change数组
            {
                change[i]=0;
                int j=H-1;//从第j行开始想上找
                while(j>=2)
                {
                    if(Image_Data[j][i]>Cmp&&Image_Data[j-1][i]<=Cmp&&Image_Data[j-2][i]<=Cmp)
                    {
                        change[i]=j;
                        break;
                    }
                    j--;
                }
                
                if(change[i]>change[i-1])
                {
                    min=i;
                }
            }
            
            
            //取得最低点左右两侧上升和下降的程度
            down=0;
            up=0;
            for(int i=V/2;i<min;i++)
            {
                if(change[i]<change[i+1])
                  down++;  
            }
            for(int i=min;i<V-1;i++)
            {
                if(change[i]>change[i+1])
                  up++;  
            }
                   
            
            //标志位前移一个
            for(int i =0;i<9;i++)
            {
               rightflagnum[i]=rightflagnum[i+1];
               //uart_printf(test_port,"***%d\n",rightflagnum[8]);
            }
            
            if((min-V/2)>30&&(V-min)>13&&down>7&&up>3)//判断其特征是否符合下三角,同时尚未进入环岛
            {
                rightflagnum[9]=1;
            }
            else
            {
                rightflagnum[9]=0;
            }
            
            sum=0;
            for(int i=0;i<10;i++)
            {
                if(rightflagnum[i]==1)
                   sum++;
            }
            
            //uart_printf(test_port,"sum:%d\n",sum);
            
            if(sum>4)
            {
                gostraightflag=10;//环在车的右侧
            }  
                     
          
    
            //画面左边
            min=0;  
            for(int i=0;i<V/2;i++)//提取画面右半部分黑白交界处的 纵坐标存储到change数组
            {
                change[i]=0;
                int j=H-1;//从第j行开始想上找
                while(j>=2)
                {
                    if(Image_Data[j][i]>Cmp&&Image_Data[j-1][i]<=Cmp&&Image_Data[j-2][i]<=Cmp)
                    {
                        change[i]=j;
                        break;
                    }
                    j--;
                }
                
                if(change[i]>change[i-1])
                {
                    min=i;
                }
            }
            
            
            down=0;
            up=0;
            for(int i=0;i<min;i++)
            {
                if(change[i]<change[i+1])
                  down++;  
            }
            for(int i=min;i<V/2;i++)
            {
                if(change[i]>change[i+1])
                  up++;  
            }
            
            //环标志位前移一个
            for(int i =0;i<9;i++)
            {
               leftflagnum[i]=leftflagnum[i+1];
            }
            
            if(min>13&&(V/2-min)>30&&down>3&&up>7)//判断其特征是否符合下三角,同时尚未进入环岛
            {
                leftflagnum[9]=1;
            }
            
            else
            {
                leftflagnum[9]=0;
            }
            
            sum=0;
            for(int i=0;i<10;i++)
            {
                if(leftflagnum[i]==1)
                  sum++;
            }
            if(sum>4)
            {
                gostraightflag=10;//环在车的右侧   
            }  
              
            
             if(gostraightflag>1)
             {
                  gostraightflag-=1;
             }
             if(gostraightflag==1)
             {
                  gostraightflag-=1;
                  state=0;
             }
          
        break;
        
        default:
          state=0;
        
  }
    
    
}

/*void stopcar()
{
     if(BlackEndL<H/3&&BlackEndML<H/3&&BlackEndM<H/3&&BlackEndMR<H/3&&BlackEndR>H/3)
     {
         int ave=(BlackEndL+BlackEndML+BlackEndM+BlackEndMR+BlackEndR)/5;
         if(abs(ave-BlackEndL)<5&&abs(ave-BlackEndML)<5&&abs(ave-BlackEndM)<5&&abs(ave-BlackEndMR)<5&&abs(ave-BlackEndR)<5)//特比较水平征2匹配。黑白交界处
         {
             outrightcircleflag=20;
         }
     }
}
*/

//右侧拐点取的范围可能不对  右侧不应该空白的地方置0  
//空白判断可能要修改成数量  比如80行有70行空白即可
void crosscopy()
{
  
        int Shizi_left = 0, Shizi_right = 0;
        int down1 = 0, up1 = 0;//左侧
        int down2 = 0, up2 = 0;//右侧
        rightcrossflag=0;
        leftcrossflag=0;
        middlecrossflag=0;
        
        leftpointflag=0;
        rightpointflag=0;
           
           
           //判断左侧或右侧是否看不到赛道边缘
           if(Bline_left[H-1] < 5)
           {
               Shizi_left = 1;
               for(int shizi_i = H-1; shizi_i > H-85; shizi_i--)
               {
                   if(Bline_left[shizi_i] >= 5 )
                   {
                      Shizi_left = 0;
                      
                   }
               }
           }
           //uart_printf(test_port,"左侧%d\n",Shizi_left);

           if(Bline_right[H-1] > V-6)
           {
                Shizi_right = 1;
                for(int shizi_i = H-1; shizi_i > H-85; shizi_i--)
                {
                     /*uart_printf(test_port,"右侧shuzu%d\n", Bline_right[shizi_i]);*/
                     if(Bline_right[shizi_i] <= V-8 )
                     {
                        Shizi_right = 0;
                        
                     }
                }
           }
           
           
           if(Bline_right[H-1] > V-6 &&Bline_left[H-1] < 5)
           {
                middlecrossflag = 1;
                for(int shizi_i = H-1; shizi_i > H/2 ; shizi_i--)
                {
                     if(Bline_right[H-1]<=V-6&&Bline_left[H-1]>=5)
                     {
                        middlecrossflag = 0;
                        
                     }
                }
           }
           //uart_printf(test_port,"右侧%d\n",Shizi_right);
           
           //根据上面的标志结果 ，如果左侧看不到，右侧看得到赛道边缘
           

                for(int shizi_i = H-1;shizi_i>70; shizi_i--)
                {  
                   
                   if(Bline_right[shizi_i] < Bline_right[shizi_i+1] )//右侧边线 左拐
                   {
                        down2++;          
                   }
                   else if(Bline_right[shizi_i] > Bline_right[shizi_i+1])//右侧边线右拐
                        up2++;
                   
                   if(Bline_left[shizi_i] >5)//发现左侧出现赛道时，退出
                        break;
                }
                
                
                for(int shizi_i = H-1;shizi_i>70; shizi_i--)
                {
                   if(Bline_left[shizi_i] > Bline_left[shizi_i+1])//左侧边线 右拐
                        down1++;
                   
                   else if(Bline_left[shizi_i] < Bline_left[shizi_i+1])//左侧边线 左拐
                        up1++;
                   
                   if(Bline_right[shizi_i] < V-6)//发现右侧出现赛道时，推出
                        break;
                }
           
                 
           
                 //OLED_Print_Num(50,110, (uint16_t)down1);
                 //OLED_Print_Num(15,110, (uint16_t)up1);
                 //OLED_Print_Num(30,110, (uint16_t)down2);
                 //OLED_Print_Num(90,110,(uint16_t)up2);
                
                /*uart_printf(test_port,"左线右拐:  %d  ",down1);
                uart_printf(test_port,"左线左拐:  %d ",up1);
                uart_printf(test_port,"右侧空白:  %d\n",Shizi_right);*/
                
                uart_printf(test_port,"右线左拐:  %d  ",down2);
                uart_printf(test_port,"右线右拐   %d  ",up2);
                uart_printf(test_port,"左侧空白:  %d\n",Shizi_left);
           
           
                   for(int i= H-1;i>H*7/8;i--) 
                   {
                     if(Bline_left[i]!=0&&Bline_left[i-1]!=0&&Bline_left[i-2]!=0&&Bline_left[i-3]!=0&&Bline_left[i-4]!=0  //连续5行不丢线，并且这行左边线都贴不到邮编
                        &&Bline_left[i]<V-1&&Bline_left[i-1]<V-1&&Bline_left[i-2]<V-1&&Bline_left[i-3]<V-1&&Bline_left[i-3]<V-1)  
                     {
                     
                      if((Bline_left[i]-Bline_left[i-1]<=-1)&&(Bline_left[i-1]-Bline_left[i-2]<=-1)&&(Bline_left[i-3]-Bline_left[i-2]<=-1)&&(Bline_left[i-4]-Bline_left[i-3]<=-1))//找到左拐点
                       {
                           leftpointflag=1;
                           /*uart_printf(test_port,"bline_left[%d]:%d\n",i-4,Bline_left[i-4]);
                           uart_printf(test_port,"bline_left[%d]:%d\n",i-3,Bline_left[i-3]);
                           uart_printf(test_port,"bline_left[%d]:%d\n",i-2,Bline_left[i-2]);
                           uart_printf(test_port,"bline_left[%d]:%d\n",i-1,Bline_left[i-1]);
                           uart_printf(test_port,"bline_left[%d]:%d\n",i,Bline_left[i]);*/
                           break;
                        }
                     }
                   }
           
           
                   //右拐点
                   for(int i= H-1;i>H*7/8;i--) 
                   {
                     if(Bline_right[i]!=0&&Bline_right[i-1]!=0&&Bline_right[i-2]!=0&&Bline_right[i-3]!=0&&Bline_right[i-4]!=0  //连续5行不丢线，并且这行左边线都贴不到邮编
                        &&Bline_right[i]<V-1&&Bline_right[i-1]<V-1&&Bline_right[i-2]<V-1&&Bline_right[i-3]<V-1&&Bline_right[i-3]<V-1)  
                     {
                     
                        if((Bline_right[i]-Bline_right[i-1]>=1)&&(Bline_right[i-1]-Bline_right[i-2]>=1)&&(Bline_right[i-3]-Bline_right[i-2]>=1)&&(Bline_right[i-4]-Bline_right[i-3]>=1))//找到左拐点
                        {
                           rightpointflag=1;
                        }
                     }
                   }
           
                  
                  //三角出现位置必须在屏幕一侧，不能太靠近中心
                //for(int i=H-1;i>H/2;I++)
                
                 
                
                if(((up1 >= 10 && down1 >= 10 )&&Shizi_right&&!Shizi_left))//控制好阈值 小了的话弯道会误判；大了的话十字不易判别
                 {
                          rightcrossflag=1;//右入十字
                          //uart_printf(test_port,"右入1\n");
                       
                  }
                 if((leftpointflag&&Shizi_right))
                 {
                          rightcrossflag=1;//右入十字
                        //  uart_printf(test_port,"右入2\n");
                       
                  }
                 if(((up2 >= 10 && down2 >= 10 )&&Shizi_left&&!Shizi_right))
                 {
                          leftcrossflag=1;//左入十字
                         // uart_printf(test_port,"左入1\n");
                         // uart_printf(test_port,"右侧空白:%d\n",Shizi_right);
                       
                  }
                  if((rightpointflag&&Shizi_left))
                 {
                          leftcrossflag=1;//左入十字
                      //    uart_printf(test_port,"左入2\n");
                       
                  }
             
                  
       
           
           
}




uint8 cuision_buffer[5];




//冲到断路时停车
void stopcar()
{
            //标志移位
            for(int i=0;i<9;i++)
            {
                stopcarflagnum[i]=stopcarflagnum[i+1];
            }
            //停车条件判断
            if(BlackEndL<H/3*2&&BlackEndML<H/3*2&&BlackEndM<H/3*2&&BlackEndMR<H/3*2&&BlackEndR<H/3*2)//特征1匹配 下方有整片空白
            {
                    int ave=(BlackEndL+BlackEndML+BlackEndM+BlackEndMR+BlackEndR)/5;
                 //uart_printf(test_port,"ave:%d\n",ave);
                 //uart_printf(test_port,"%d %d %d %d %d %d\n\n",BlackEndL,BlackEndML,BlackEndM,BlackEndMR,BlackEndR);
                    if(abs(ave-BlackEndL)<8&&abs(ave-BlackEndML)<8&&abs(ave-BlackEndM)<8&&abs(ave-BlackEndMR)<8&&abs(ave-BlackEndR)<8+1)//特比较水平征2匹配。黑白交界处
                    {
                         stopcarflagnum[9]=1;
                    }      
             }
             else
             {
                  stopcarflagnum[9]=0;
             }
             //数量统计
             int sum=0;
             for(int i=0;i<10;i++)
             {
                if(stopcarflagnum[i]==1)
                  sum++;    
             }
             if(sum>8)
             {
                stopcarflag=1;
             }
             //uart_printf(test_port,"stopcarflag:%d\n",stopcarflag);
}



//返回出发点时斑马线
void whiteblackline()
{
    int black=0;
    int white=0;
    
    for(int i =0;i<V-1;i++)
    {
        if(Image_Data[H/2][i]>Cmp&&Image_Data[H/2][i+1]<Cmp)
        {
           black++;
        }
        
        if(Image_Data[H/2][i]<Cmp&&Image_Data[H/2][i+1]>Cmp)
        {
           white++;
        }
    }
    
    if(black>5&&white>5)
        destinationflag=1;
}


uint16 leavecontroltime=0;

void leaveroad()
{       
    setpoint1=310;
    setpoint2=310;
    if(leavecontroltime<5)
    {
        direction_control(6);
    }
    else if(leavecontroltime<8)
    {
        direction_control(8);
    }
    else if(leavecontroltime<11)
    {
        direction_control(10);
    }
    else if(leavecontroltime<15)
    {
        direction_control(6);
    }
    else if(leavecontroltime<220)
    {
        direction_control(0);
    }
    else if(leavecontroltime<230)
    {
        direction_control(-10);
    }
    else if(leavecontroltime<240)
    {
        direction_control(-20);
    }
    else if(leavecontroltime<250)
    {
        direction_control(-30);
    }
    else if(leavecontroltime<260)
    {
        direction_control(-20);
    }
    else if(leavecontroltime<270)
    {
        direction_control(-10);
    }
    else if(leavecontroltime<330)
    {
        direction_control(0);
    }
    if(leavecontroltime<330)
      leavecontroltime++;
    else
    {
         leavecontroltime=0;
         leaveroadflag=0;
    }
     
    
    
}



//断路从右向左转弯掉头
uint8 turncontroltime=0;

void turnround(void)
{
    if(turncontroltime<25)//
    {
        setpoint1=100;
        setpoint2=100;
        direction_control(-30);//左后方转圈
    }
    else if(turncontroltime<167)
    {
        setpoint1=260;
        setpoint2=260;
        direction_control(50);
    }
    else if(turncontroltime<207)
    {
        setpoint1=150;
        setpoint2=150;
        direction_control(-25);
    }
    else
    {
        setpoint1=0;
        setpoint2=0;
        direction_control(0);
    }
    
    if(turncontroltime<208)
       turncontroltime++;
    else
    {
        stopcarflag=0;
        turncontroltime=0;
    }
      
}

uint8 leavewaitcontroltime=0;

void leavewait()
{       
    if(continueflag==0)//左转等待
    {
        if(leavewaitcontroltime<20)
        {
            setpoint1=30;
            setpoint2=30;
            direction_control(30);
        }
        else if(leavewaitcontroltime<40)
        {
            setpoint1=10;
            setpoint2=10;
            direction_control(50);
        }
        else if(leavewaitcontroltime<40)
        {
            setpoint1=0;
            setpoint2=0;
            //direction_control(30);
        }
        leavewaitcontroltime++;
    }
    else//收到继续行驶信号之后
    {
        if(leavewaitcontroltime<20)//倒退回原位置
        {
            setpoint1=-100;
            setpoint2=-100;
            direction_control(30);
            leavewaitcontroltime++;
        }
        else if(leavewaitcontroltime<30)//停车
        {
            setpoint1=0;
            setpoint2=0;
            direction_control(0);
            leavewaitcontroltime++;
        }
        else//利用电磁过断路
        {
            leavewaitcontroltime=0;
            continueflag=0;
            
            
            pit_irq_en(PIT1);//开启电磁进行控制
        }
    }
}

void findnewroad()
{
    if(BlackEndR&&BlackEndMR>5&&BlackEndM>5&&BlackEndML>5&&BlackEndL>5)
    {
          camera_flag=1;
          pit_irq_dis(PIT1);
     }
}






/*******************************************************************************
函数名称：stable_control_new
函数功能: 数据处理控制程序
参数：无
*******************************************************************************/
void stable_control_fix(int valid_line)
{
  
  uint8 weight[]={1,1,1,1,1,1,1,1,1,1,
                1,1,1,1,1,1,1,1,1,1,
                1,1,1,2,2,2,2,3,3,3,
                3,3,3,3,3,3,3,3,3,3,
                3,3,3,2,2,2,2,2,2,2,
                1,1,1,1,1,1,1,1,1,1               
                }; 
  int weight_sum=0;

  int Bline_left_all,Bline_right_all=0;
  
  
  if(leftcircleflag||rightcircleflag||outleftcircleflag||outrightcircleflag||gostraightflag)//圆环处理
  {

           if(leftcircleflag>0)
           {
               direction_control(40);//这个参数瞎写的
               //delayms(500);
               //uart_printf(test_port,"11\n");
           }
           else if(rightcircleflag>0)
           {
               direction_control(-40);//这个参数瞎写的
               //delayms(100);
               //uart_printf(test_port,"16\n");
           }
              
           else if(outleftcircleflag>0)
           {
               direction_control(40);//这个参数瞎写的
               //uart_printf(test_port,"12\n");
               //delayms(500);
           }
           else if(outrightcircleflag>0)
           {
               direction_control(-40);//这个参数瞎写的
               //delayms(500);
               //uart_printf(test_port,"13\n");
           }
           else
           {
               direction_control(0);//这个参数瞎写的
               //delayms(500);
           }
           
           //uart_printf(test_port,"%d %d %d %d %d\n",leftcircleflag,rightcircleflag,outleftcircleflag,outrightcircleflag,gostraightflag);
    
  }
  
  
  else if(stopcarflag)//断路检测
  {
      /*direction_control(0);
      setpoint1=0;
      setpoint2=0;*/
      
      leavewait();
      //断路返回
      //turnround();
      
  }
  
  else if(leaveroadflag)//躲避道路障碍，平时用于防撞
  {
      direction_control(0);
      setpoint1=0;
      setpoint2=0;
      //绕过障碍
      //leaveroad();
      
  }
  
  else if(destinationflag)//检测到斑马线停车
  {
      direction_control(0);
      setpoint1=0;
      setpoint2=0;
      p1=20;
      p2=20;
  }
  
  else
  {
          //舵机中线调整
          if(readyrightflag==1)
              middleline=92;
          else if(readyleftflag==1)
              middleline=94;
          else
              middleline=90;
          
          int cursion_all = 0;
           for(int line = H-10; line >=H -10- valid_line; line--)
           {
                cursion_all = cursion_all + (Pick_table[line] - middleline)*weight[line-60];//*矫正
                weight_sum+=weight[line-60];     
           }   
           
           int cursion_avg = 2.1*cursion_all/weight_sum;   
           uart_printf(test_port,"cursion_avg:%d\n\n\n",cursion_avg);
                    
           
           //分段差速控制 
           //cuision>0 舵机向右 pwm小
          if(cursion_avg <= -50||cursion_avg >= 50)
           {
              direction_control(-cursion_avg);
              if(cursion_avg>0)
              {
                 setpoint1=350+cursion_avg;
                 setpoint2=220-cursion_avg;
              }
              else
              {
                 setpoint1=220-cursion_avg;
                 setpoint2=350+cursion_avg;
              }
              
           }
          
           else if(cursion_avg <= -10||cursion_avg >= 10)
           {
              direction_control(-cursion_avg);
              setpoint1=360+cursion_avg;
              setpoint2=360-cursion_avg;
           }
          
           else//如果偏差较小
           {
              direction_control(-cursion_avg);
              //ftm_pwm_duty(FTM3, FTM_CH1,822);//舵机控制
              setpoint1=400;
              setpoint2=400; 
           } 
     }
}





  
     
     