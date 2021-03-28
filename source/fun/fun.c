#include "include.h"
#include "fun.h"
extern struct Bline Bline_lefts[PIC_H]; 
extern struct Bline Bline_rights[PIC_H];
extern uint8 PickCenter_flag;
//extern unsigned char const adjust_table[];//����ʧ�������������
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
extern uint8 par_num;
extern uint8 row_F[H];//���вɼ���ɱ�־
extern uint8 ctl_num;//���Ʊ�־ 0Ϊ�¿��ƣ�1Ϊ�Ͽ���
extern char startline_F;//������ʼ��
extern char endline_F;//���ֽ�����  
extern uint8 set_flag;//���ñ�־λ
extern uint32 switch_key;//���뿪�أ�IO�˿�ֵ
//extern ParValue myPar_num;
uint16 last_left_line=(V/2);
uint16 last_right_line=(V/2);
uint8 buttonsPressed=0;
static const char * const mainMenu[] = {
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
};
//IO��ʼ������
void IO_Init()
{
	/* �򿪸����˿ڵ�ʱ��Դ */
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | 
	SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
	//C0~C7��ΪGPIO����ģʽ������ov7260��8λ�Ҷ�����
	PORTC_PCR0=PORT_PCR_MUX(1);//B0��������ΪGPIOģʽ
	PORTC_PCR1=PORT_PCR_MUX(1);//B1��������ΪGPIOģʽ
	PORTC_PCR2=PORT_PCR_MUX(1);//B2��������ΪGPIOģʽ
	PORTC_PCR3=PORT_PCR_MUX(1);//B3��������ΪGPIOģʽ
	PORTC_PCR4=PORT_PCR_MUX(1);//B4��������ΪGPIOģʽ
	PORTC_PCR5=PORT_PCR_MUX(1);//B5��������ΪGPIOģʽ
	PORTC_PCR6=PORT_PCR_MUX(1);//B6��������ΪGPIOģʽ
	PORTC_PCR7=PORT_PCR_MUX(1);//B7��������ΪGPIOģʽ
	GPIOC_PDDR&=0XFFFFFF00;//B0~B7����Ϊ���룬��������ͷ8λ�Ҷ�����
	
	PORTA_PCR14=PORT_PCR_MUX(1)|PORT_PCR_IRQC(10);//A14��������ΪGPIOģʽ���½����ж�,���ж�
	
	PORTB_PCR22=PORT_PCR_MUX(1)|PORT_PCR_IRQC(9);//B22��������ΪGPIOģʽ���������ж�,���ж�
	
	/////////////////////////////���İ�LED///////////////////////////////////////////////
	PORTA_PCR15=PORT_PCR_MUX(1);
	
	//////////////////////////////SD��SPIģ��ڳ�ʼ��///////////////////////////////////
	//����PORTA pin16,pin17,pin18,pin19ΪGPIO��
	PORTE_PCR0=(0|PORT_PCR_MUX(1));
	PORTE_PCR1=(0|PORT_PCR_MUX(1)); 
	PORTE_PCR2=(0|PORT_PCR_MUX(1));
	PORTE_PCR3=(0|PORT_PCR_MUX(1));
	PORTE_PCR4=(0|PORT_PCR_MUX(1));
	//����PORTA pin16,pin17Ϊ�������
	GPIOE_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(0)|GPIO_PIN(1)|GPIO_PIN(2)|GPIO_PIN(3)|GPIO_PIN(4));
	GPIOE_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(0)|GPIO_PIN(1)|GPIO_PIN(2)|GPIO_PIN(3)|GPIO_PIN(4)); //�ȶ�ȡ��Ȼ��������
	//PORTB_PCR23=PORT_PCR_MUX(1)|PORT_PCR_IRQC(1);//B11��������ΪGPIOģʽ�������ش���DMA����
    ////////////////////// ��չI/0�ڳ�ʼ�� ////////////////////////////   
	 	PORTC_PCR16=PORT_PCR_MUX(1);
		PORTC_PCR15=PORT_PCR_MUX(1);
	////////////////////// ���뿪��I/0�ڳ�ʼ�� ////////////////////////////
		PORTD_PCR0=PORT_PCR_MUX(1);
		PORTD_PCR1=PORT_PCR_MUX(1);
		PORTD_PCR2=PORT_PCR_MUX(1);
		PORTD_PCR3=PORT_PCR_MUX(1);
	////////////////////// ָʾ��I/O�ڳ�ʼ�� ///////////////////////////////
		PORTC_PCR10=PORT_PCR_MUX(1);		//LED1	��ת��
		PORTD_PCR2 =PORT_PCR_MUX(1);		//LED2	��ת��
		PORTB_PCR16=PORT_PCR_MUX(1);		//LED3
    	PORTB_PCR17=PORT_PCR_MUX(1);		//LED4
    	PORTB_PCR20=PORT_PCR_MUX(1);		//LED5
    ////////////////////// IICģ��I/O��ʼ�� ////////////////////////////////
		PORTC_PCR8=PORT_PCR_MUX(1);
    	PORTC_PCR9=PORT_PCR_MUX(1);
	////////////////////// ����I/O��ʼ�� ////////////////////////////////
		PORTB_PCR10=PORT_PCR_MUX(1);	//KEY1
		PORTB_PCR9=PORT_PCR_MUX(1);		//KEY2
		PORTB_PCR3=PORT_PCR_MUX(1);		//KEY3
    	PORTB_PCR2=PORT_PCR_MUX(1);		//KEY4
		PORTB_PCR1=PORT_PCR_MUX(1);		//KEY5
		PORTB_PCR10|=0x03;//����Ϊ�ڲ�����ģʽ
		PORTB_PCR9 |=0x03;
		PORTB_PCR3 |=0x03;//����Ϊ�ڲ�����ģʽ
		PORTB_PCR2 |=0x03;
		PORTB_PCR1 |=0x03;//����Ϊ�ڲ�����ģʽ

		GPIOA_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(15));
		GPIOC_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(10));
		GPIOC_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(15)|GPIO_PIN(16));
    	GPIOB_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(20)|GPIO_PIN(16)|GPIO_PIN(17));//B��ָʾ��
		GPIOD_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(2));//D��ָʾ��
		//GPIOD_PDDR&=~GPIO_PDDR_PDD(GPIO_PIN(0)|GPIO_PIN(1)|GPIO_PIN(2)|GPIO_PIN(3));//���뿪��
		GPIOB_PDDR&=~GPIO_PDDR_PDD(GPIO_PIN(10)|GPIO_PIN(9)|GPIO_PIN(3)|GPIO_PIN(2)|GPIO_PIN(1));//����
		GPIOC_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(10));
		GPIOA_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(15)); 
		
		GPIOC_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(15)|GPIO_PIN(16));
		GPIOB_PDOR &= ~GPIO_PDOR_PDO(~GPIO_PIN(20)|~GPIO_PIN(16)|~GPIO_PIN(17)); //�ȶ�ȡ��Ȼ��������
		GPIOD_PDOR &= ~GPIO_PDOR_PDO(~GPIO_PIN(2)); //�ȶ�ȡ��Ȼ��������
    	GPIOC_PDOR &= ~GPIO_PDOR_PDO(~GPIO_PIN(14)); //�ȶ�ȡ��Ȼ��������
		
    	GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(20)|GPIO_PIN(16)|GPIO_PIN(17));	//IO������ߵ�ƽ����
		GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(2));	//IO������ߵ�ƽ����
		GPIOC_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(10));
	
}

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
    LCD_Print(0, 0, (uint8 *)menuText[0]); // Print the title
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
                    LCD_Print(0, (i-start_position+1)*2, (uint8 *)menuText[i]);
                }
                else {
                    // Highlight item at current position
                    LCD_P8x16StrInvert(0, (i-start_position+1)*2, (uint8 *)menuText[i]);
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
�������ƣ�mode
��������: ģʽѡ����
��������
*******************************************************************************/
void mode()
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
}
/*******************************************************************************
�������ƣ�sendFlashData
��������: ����flash����
��������
*******************************************************************************/
void sendFlashData()
{
    uint8 tmp=0;
	uint16 i=0;
	buttonsPressed=0;
	LCD_CLS();//����
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
�������ƣ�readSDData
��������: ��ȡSD����
��������
*******************************************************************************/
void readSDData()
{
    uint8 tmp=0;
	uint8 pixel;
	uint32 sector=0;
	int offset=0;
	//char data[20]={0};
	uint8 ok_flag=0;
	int num=0;
	buttonsPressed=0;
	LCD_CLS();//����
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	LCD_Print(0,2,"waiting....");
	while (!buttonsPressed) 
	{
		if(ok_flag==1)
		{
			LCD_Print(0,2,"read....  ");
			LPLD_Disk_Read(0,test_table,sector,1);
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
�������ƣ�choice_kz
��������: ѡ����Ʋ���
��������
*******************************************************************************/
void Reset()
{
  uint8 tmp;
	buttonsPressed=0;
	LCD_CLS();//����
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
�������ƣ�choice_kz
��������: ѡ����Ʋ���
��������
*******************************************************************************/
uint8 scan_key()
{
	uint32 key_code=0;
	uint16 key_num=0;
	uint8 key_value=0;
	//key_code=(GPIO_PDIR_REG(PORTB)&0x0000FFFF);
	//key_num=(uint16)key_code;
	TEST_KEY(key_code,key_num);
	if(key_num!=0x060E)
	{
		delay(1);
		TEST_KEY(key_code,key_num);
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
				TEST_KEY(key_code,key_num);
		}
	}
	return key_value;
}
void Testled()
{
	while (!buttonsPressed) 
	{
		GPIOB_PDOR&=~GPIO_PDDR_PDD(GPIO_PIN(20)|GPIO_PIN(16)|GPIO_PIN(17));//B��ָʾ��
		GPIOD_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(2));	//IO������ߵ�ƽ����
		GPIOC_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(10));
	}
	buttonsPressed=0;
}
/*******************************************************************************
�������ƣ�setThreshold
��������: ѡ����Ʋ���
��������
*******************************************************************************/
void setThreshold()
{
	char data[5];
	uint8 tmp=0;
	buttonsPressed=0;
	LCD_CLS();//����
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
�������ƣ�choice_kz
��������: ѡ����Ʋ���
��������
*******************************************************************************/
void setCDSpeed()
{
	char data[5];
	uint8 speed_tmp;
	buttonsPressed=0;
	LCD_CLS();//����
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
�������ƣ�setArgInt
��������: �������Ͳ���
��������
*******************************************************************************/
void setArgInt(char *str,int * Arg,const int step)
{
	char data[5];
	int tmp=0xff;
	buttonsPressed=0;
	LCD_CLS();//����
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
�������ƣ�setArgUchar
��������: �����޷����ַ��Ͳ���
��������
*******************************************************************************/
void setArgUchar(char *str,unsigned char * Arg,const unsigned char step)
{
	char data[5];
	unsigned char tmp=0xff;
	buttonsPressed=0;
	LCD_CLS();//����
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
�������ƣ�setArgFloat
��������: ���ø����Ͳ���
��������
*******************************************************************************/
void setArgFloat(char *str,float * Arg,const float step)
{
	char data[5];
	float tmp=100.0;
	buttonsPressed=0;
	LCD_CLS();//����
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
�������ƣ�BiLi
��������: ѡ����Ʋ���
��������
*******************************************************************************/
void BiLi()
{
	char data[5];
	float tmp;
	buttonsPressed=0;
	LCD_CLS();//����
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
�������ƣ�BiLi
��������: ѡ����Ʋ���
��������
*******************************************************************************/
void set_dj_Kp()
{
	char data[5];
	float tmp=0;
	buttonsPressed=0;
	LCD_CLS();//����
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
�������ƣ�setZDSpeed
��������: ѡ����Ʋ���
��������
*******************************************************************************/
void setZDSpeed()
{
	char data[5];
	uint8 speed_tmp;
	buttonsPressed=0;
	LCD_CLS();//����
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
�������ƣ�set_pid_P
��������: ����pid�� D ֵ
��������
*******************************************************************************/
void set_pid_D()
{
	char data[5];
	int tmp;
	buttonsPressed=0;
	LCD_CLS();//����
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=Kd)
		{
			LCD_Print(0,2," Kd:        ");
			sprintf(data,"%d",Kd);
	  		LCD_Print(80,2,(uint8*)data);
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
�������ƣ�choice_kz
��������: ѡ����Ʋ���
��������
*******************************************************************************/
void set_pid_P()
{
	char data[5];
	int tmp;
	buttonsPressed=0;
	LCD_CLS();//����
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
�������ƣ�djSet
��������: �������
��������
*******************************************************************************/
void djSet()
{
	uint16 tmp=0xff;
	uint16 dj_num=dj_center;
	char data[5]={0};
	buttonsPressed=0;
	LCD_CLS();//����
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=dj_num)
		{
			LCD_Print(0,2,"dj_num:     ");
			sprintf(data,"%d",dj_num);
			LCD_Print(60,2,(uint8 *)data);
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
�������ƣ�displaySet
��������: ��ʾͼ����Ʋ���
��������
*******************************************************************************/
void displaySet()
{
	uint8 tmp=0xff;
	buttonsPressed=0;
	LCD_CLS();//����
	LCD_Print(0, 0," CDU--Spower"); // Print the title
	while (!buttonsPressed) 
	{
		if(tmp!=DisImg_flag)
		{
			LCD_Print(0,2," Display:     ");
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
			tmp=DisImg_flag;
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
�������ƣ�choice_kz
��������: ѡ����Ʋ���
��������
*******************************************************************************/
void choice_kz()
{
  uint8 temp_key=0,temp_flag=0;
  uint8 put[3]={0};
  //LCD_CLS();
  LCD_Print(8,0,"wait_choicekz...");
  temp_key=gpio_get(PORTD,4);
  if(temp_key==0)
  {
	delay1(5);
	temp_key=gpio_get(PORTD,4);
	if(temp_key==0)
	{
		ctl_num=1;
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
	  ctl_num=0;
	  temp_flag=1;
	}
	while(gpio_get(PORTD,5)==0);
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
�������ƣ�threshold
��������: ��ֵ��ú���
��������
*******************************************************************************/
void threshold()
{
  
}
/*******************************************************************************
�������ƣ�test_threshold
��������: ��ֵ��ú���
��������
*******************************************************************************/
void test_threshold()
{
  uint32 threshold_temp=0;
  uint8 i,line,pixel,th,put[3]={0};
  enable_irq (87);//ʹ��A���ж� ��A14���ж�	   
  startline_F=0;//������ʼ�б�־��0
  endline_F=0;//���ֽ����б�־��0
  for(i=0;i<H&&endline_F==0;i++) //��ȡ�������ĵ㲢��������
  {
		while(row_F[i]==0);//�ȴ����в������
		row_F[i]=0;//����ɼ���ɱ�־λ
  }
  for(line=3;line<6;line++)
  {
	for(pixel=100;pixel<180;pixel++)
	{
	  threshold_temp+=video[line][pixel];
	}
  }
  th=threshold_temp/240;
  LCD_Print(8,0,"sd_liangdu...");
  LCD_Print(8,2,"th:");
  char_change_1(th,put);
  LCD_Print(52,2,put);
}
/*******************************************************************************
�������ƣ�choice_parameter
��������: ѡ���������
��������
*******************************************************************************/
void choice_par()
{
  uint8 temp_key=0,temp_flag=0;
  uint8 put[3]={0};
  //LCD_CLS();
  LCD_Print(8,0,"wait_choice.... ");
  temp_key=gpio_get(PORTD,4);
  if(temp_key==0)
  {
	delay1(5);
	temp_key=gpio_get(PORTD,4);
	if(temp_key==0)
	{
	  if(par_num>5)
		par_num=5;
	  else
	  	par_num=par_num+1;
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
	  if(par_num<1)
		par_num=0;
	  else
	  	par_num=par_num-1;
	  temp_flag=1;
	}
	while(gpio_get(PORTD,5)==0);
  }
  if(temp_flag==1)
  {
	LCD_Print(8,2,"par_num:         ");
 	char_change_1(par_num,put);
	delay2();
	LCD_Print(72,2,put);
	if(par_num==0)
	  LCD_Print(8,4,"Kp          ");
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
	  LCD_Print(8,4,"            ");
  }
  temp_flag=0;
}
/*******************************************************************************
�������ƣ�set_parameter
��������: ���ò�������
��������
*******************************************************************************/
void set_par()
{
  uint8 temp_key=0,temp_flag=0;
  uint8 put[3]={0};
  //LCD_CLS();
  LCD_Print(8,0,"wait_set.... ");
  if(par_num==0)//Kp��������
  {
		  temp_key=gpio_get(PORTD,4);
		  if(temp_key==0)
		  {
			delay1(5);
			temp_key=gpio_get(PORTD,4);
			if(temp_key==0)
			{
			  if(Kp>60)
				Kp=60;
			  else
				Kp=Kp+1;
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
			  if(Kp<1)
				Kp=0;
			  else
				Kp=Kp-1;
			  temp_flag=1;
			}
			while(gpio_get(PORTD,5)==0);
		  }
		  if(temp_flag==1)
		  {
			  LCD_Print(8,2,"speed_Kp:       ");
			  char_change_1(Kp,put);
			  delay2();
			  LCD_Print(80,2,put);
			  LCD_Print(8,4,"            ");
		  }
		  temp_flag=0;
	  }
	  else if(par_num==1)//Ki��������
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
	  else if(par_num==2)//Kd��������
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
		  temp_flag=0;
  }
  else if(par_num==3)//ֱ���ٶ�
  {
		  temp_key=gpio_get(PORTD,4);
		  if(temp_key==0)
		  {
			delay1(5);
			temp_key=gpio_get(PORTD,4);
			if(temp_key==0)
			{
			  if(zhidao_speed>200)
				zhidao_speed=200;
			  else
				zhidao_speed=zhidao_speed+2;
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
			  if(zhidao_speed<1)
				zhidao_speed=0;
			  else
				zhidao_speed=zhidao_speed-2;
			  temp_flag=1;
			}
			while(gpio_get(PORTD,5)==0);
		  }
		if(temp_flag==1)
 	  	{
		  LCD_Print(8,2,"zd_speed:     ");
		  char_change_1(zhidao_speed,put);
		  delay2();
		  LCD_Print(80,2,put);
		  LCD_Print(8,4,"            ");
	  	}
	  	temp_flag=0;
	  }
	 else if(par_num==4)//ȫ���ٶ�
	  {
			  temp_key=gpio_get(PORTD,4);
			  if(temp_key==0)
			  {
				delay1(5);
				temp_key=gpio_get(PORTD,4);
				if(temp_key==0)
				{
				  if(CD_speed>200)
					CD_speed=200;
				  else
					CD_speed=CD_speed+2;
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
				  if(CD_speed<1)
					CD_speed=0;
				  else
					CD_speed=CD_speed-2;
				  temp_flag=1;
				}
				while(gpio_get(PORTD,5)==0);
	         }
			 if(temp_flag==1)
 	  	    {
			  LCD_Print(8,2,"cd_speed:     ");
			  char_change_1(CD_speed,put);
			  delay2();
			  LCD_Print(80,2,put);
			  LCD_Print(8,4,"            ");
	  		}
	  		temp_flag=0;
     }
  else if(par_num==5)
	  {
			  temp_key=gpio_get(PORTD,4);
			  if(temp_key==0)
			  {
				delay1(5);
				temp_key=gpio_get(PORTD,4);
				if(temp_key==0)
				{
				  if(Cmp>200)
					Cmp=200;
				  else
					Cmp=Cmp+1;
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
				  if(Cmp<1)
					Cmp=0;
				  else
					Cmp=Cmp-1;
				  temp_flag=1;
				}
				while(gpio_get(PORTD,5)==0);
	         }
			 if(temp_flag==1)
 	  	    {
			  LCD_Print(8,2,"cmp:        ");
			  char_change_1(Cmp,put);
			  delay2();
			  LCD_Print(80,2,put);
			  LCD_Print(8,4,"            ");
	  		}
	  		temp_flag=0;
     }
}
/*******************************************************************************
�������ƣ�choice_xs
��������: ѡ����ʾ����
��������
*******************************************************************************/
void choice_xs()
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
�������ƣ�find_coordinate
��������: �ҵ���λ�ο���,��27��ǰհΪ80cm
������
*******************************************************************************/
uint8 find_coordinate()
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
	  	GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(10));//IO��������͵�ƽ
		return 1;
	  }
	  else
	  {
		GPIOB_PDOR |= 	GPIO_PDOR_PDO(GPIO_PIN(10));//IO�������ߵ�ƽ
		return 0;
	  }
}
//����ͼ���ܺ���
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
//����ͼ���ܺ���1
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
//����ͼ���ܺ���2
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
�������ƣ�put_center_char
��������: ���������ߣ�ÿ5�з���1�У����ַ�����
������
*******************************************************************************/
void put_center_char(uint16 *center,uint8 put_line)
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
�������ƣ�put_center_hex
��������: ���������ߣ�ÿ5�з���1�У���ʮ�����Ʒ���
������
*******************************************************************************/
void put_center_hex(uint16 *center,uint8 put_line)
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
�������ƣ�put_get_char_whole
��������: ���������ߺ������ߣ�ÿ5�з���1�У����ַ�����
������
*******************************************************************************/
void put_get_char_whole(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line)
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
�������ƣ�put_get_hex_whole
��������: ���������ߺ�������
������
*******************************************************************************/
void put_get_hex_whole(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line)
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
�������ƣ�put_get_hex
��������: ���������ߺ������ߣ�ÿ5�з���1�У���ʮ�����Ʒ���
������
*******************************************************************************/
void put_get_hex(uint16 *center,uint16 *bline_left,uint16 *bline_right,uint8 put_line)
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
//��ȡ�����߷���ͼ���ܺ���
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
//����ͼ���ܺ���3
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
�������ƣ�binaryzation
��������: ��ֵ��ͼ������
������
*******************************************************************************/
void Binaryzation()
{
  uint16 line,pixel;
  for(line=0;line<H;line++)
  {
    for(pixel=V/2;pixel<V;pixel++)
    {
      if(video[line][pixel]<Cmp)//CmpΪ����ͷ��ֵ����ֵ
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
�������ƣ�binaryzation_line
��������: ��ֵ��ͼ������
������
*******************************************************************************/
void Binary_line(uint8 line)
{
  uint16 pixel;
  uint32 gray_value=0;
  uint32 count=0;
  uint8 Deal_CMP=145;
  for(pixel=0;pixel<V;pixel++)
  {
	if((video[line][pixel]<235)&&(video[line][pixel]>30))
	{
	  	count++;
		gray_value+=video[line][pixel];
	}
  }
  Deal_CMP=gray_value/count;
  for(pixel=0;pixel<V;pixel++)
  {
     if(video[line][pixel]<Deal_CMP)//CmpΪ����ͷ��ֵ����ֵ
     {
        video_deal[line][pixel]=0;
     }
     else
        video_deal[line][pixel]=1;
  }
}
/*******************************************************************************
�������ƣ�getWholeArea
��������: �����Ч���ڵ��������
������
*******************************************************************************/
uint32 getWholeArea()
{
  	uint16 line;
  	uint32 area=0;
  	for(line=0;line<valid_line;line++)
  	{
	  //if((Deal_flag[line]&INVALID_LINE)==0)
	  {
		area+=(Bline_left[line]-Bline_right[line]);
	  }
    }
	return area;
}
/*******************************************************************************
�������ƣ�regression���ع飩
��������: ��С���˷���б��
������
*******************************************************************************/
int regression(uint16 Pick_table[],int startline,int endline)//����б�ʺ���
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
�������ƣ�getWholeArea
��������: �����Ч���ڵ��������
������
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
�������ƣ�get_area
��������: ��������
������
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
�������ƣ�get_curvature
��������: ��������
������
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
�������ƣ�get_area
��������: �������� 
������
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
�������ƣ�PickCenter_near
��������: Ѱ�ҽ��������ߺ���
������
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
�������ƣ�PickCenter_near
��������: Ѱ�ҽ��������ߺ���
������
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
  for(temp_line=0;temp_line<5;temp_line++)//ͳ��ǰ5���м�������50�кڵ���Ŀ 
  {																			  //���ڿ��Գ��Ա��ط�
		for(pixel=V/2-45;pixel<(V/2);pixel++)
		{
			if(video[temp_line][pixel]<Cmp)
			{
				Black_count_right++;
			}
		}
		for(pixel=V/2;pixel<(V/2+45);pixel++)
		{
			if(video[temp_line][pixel]<Cmp)
			{
				Black_count_left++;
			}
		}
  }
  if(Black_count_right>Black_count_left)//�м��ұߺڵ���Ŀ������� ����ƫ��
  {
		start_pixel=V/2+45;
  }
  else if(Black_count_right<Black_count_left)//�м���ߺڵ���Ŀ�����ұ� ����ƫ��
  {
		start_pixel=V/2-45;
  }
  else
  {
		start_pixel=V/2;
  }
  for(pick_line=0;pick_line<5;pick_line++)
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
			  Pick_flag[pick_line]|=LEFT_LOST_W;
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
			  Pick_flag[pick_line]|=RIGHT_LOST_W;
			}
			if(Bline_left[pick_line]==(V-1)&&Bline_right[pick_line]==0)//���ұ߽綼δ�ҵ�
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
 
  //Ϊ���������㷨��������ֵ
  last_left_line=Bline_left[4];
  last_right_line=Bline_right[4];
  if(temp_lost_line>0)
	return temp_lost_line;
  else
  	return 0;
}

/*******************************************************************************
�������ƣ�PickCenter_near
��������: ������Ѱ�߷�
������
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
	  ///////////////////////////////�����///////////////////////////////////
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
					if(flag>1)
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
		}
		else//�ҵ����� ����Ҫ��Ϊ�ж����Һ��߹�ϵ��������ֵ
		{
		  last_right_line = Bline_left[pick_line];
		}
		/////////////////////�ұ���///////////////////
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
		if(Bline_left[pick_line]==(V-1)&&Bline_right[pick_line]==0)//���ұ߽綼δ�ҵ�
		{
			temp_lost_line++;
		}
	}
 
	  //Ϊ���������㷨��������ֵ
	if(temp_lost_line==0)
	{
		return 0;
	}
	else
		return 1;
}
/*******************************************************************************
�������ƣ�PickCenter_diff
��������: ����Ѱ��
������line Ѱ����
*******************************************************************************/
void PickCenter_diff(uint16 line)
{
  uint16 pixel;
  uint16 left_start;//
  uint16 right_start;//
  uint16 left_end;//�����Ѱ�߽����߽�
  uint16 right_end;//�ұ���Ѱ�߽����߽�
  uint16 i;
  uint8 flag=0;//�����жϱ�־
  volatile uint16 left_lost_flag=0,right_lost_flag=0;
  //const int Cmp_D=25;
  //const int  MAX=240;
  //////////////////////////////Ѱ���㷨/////////////////////////////////////
  {
	    if(last_left_line>IMG_DIFF)
	    	left_start=last_left_line-IMG_DIFF;
		else
		    left_start=1;
	    if(Bline_left[line-1]<(V-IMG_DIFF))
	    	left_end=last_left_line+IMG_DIFF;//ȡ��һ�к���ƫ��IMG_DIFF������
		else
		  	left_end=V-1;
		if((video[line][left_start]<Cmp)&&(video[line][left_start+1]<Cmp))
		{
		  	pixel=left_start;
		}
		else
		{
		  	for(pixel=left_start;pixel<left_end;pixel++)
			{
				if(video[line][pixel-1]>Cmp&&video[line][pixel]<Cmp)
				{
				   flag=0;
				   for(i=0;i<3;i++)
				   {
					 if(video[line][pixel+i]<Cmp)
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
		else if(pixel==left_end)//δ�ҵ�����
		{
		  	Pick_flag[line] |= LEFT_LOST_W;
			Bline_left[line]=last_left_line;
			left_lost_flag=1;
			Lost_left_count++;
		}
/////////////////////////////////////////////////////////////////////////////////
		if(last_right_line>IMG_DIFF)
			right_end=last_right_line-IMG_DIFF;
		else
		    right_end=1;
		right_start=last_right_line+IMG_DIFF;
		if(right_start>(V-1))
		{
			 right_start=V-1;
		}//ע���Ƿ��ȥ��
		if((video[line][right_start]<Cmp)&&(video[line][right_start-1]<Cmp))
		{
			pixel=right_start;
		} 
		else
		{
		  	for(pixel=right_start;pixel>right_end;pixel--)
	  		{
				if(video[line][pixel]<Cmp)
				{
				   flag=0;
				   for(i=0;i<3;i++)
				   {
					 if(video[line][pixel-i]<Cmp)
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
		else if(pixel==right_end)//δ�ҵ��ұߺ���
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
�������ƣ�PickCenter_up
��������: Ѱ�������ߺ�������
������line Ѱ����
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
  else if(PickCenter_flag<50)//�˴����޸Ŀ��Ƿ��ʹ��ȡ���̳���
  {
	    if(Bline_left[line-1]<270)
	    	temp1=Bline_left[line-1]+5;//ȡ��һ�к���ƫ��IMG_DIFF������
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
			}//ע���Ƿ��ȥ��
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
  else //�˴����޸Ŀ��Ƿ��ʹ��ȡ���̳���
  {
	    if(Bline_left[line-1]<270)
	    	temp1=Bline_left[line-1]+5;//ȡ��һ�к���ƫ��IMG_DIFF������
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
  if((right_lost_flag==1)&&(left_lost_flag==1))
  {
	Lost_Line_count++;
  }
  PickCenter_flag++;
}
/*******************************************************************************
�������ƣ�PickCenter_up
��������: Ѱ�������ߺ�������
������line Ѱ����
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
	  else if(PickCenter_flag<30)//�˴����޸Ŀ��Ƿ��ʹ��ȡ���̳���
	  {
			if(Bline_left[line-1]<(V+IMG_DIFF))
				temp1=Bline_left[line-1]+IMG_DIFF;//ȡ��һ�к���ƫ��IMG_DIFF������
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
				}//ע���Ƿ��ȥ��
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
	  else //�˴����޸Ŀ��Ƿ��ʹ��ȡ���̳���
	  {
			if(Bline_left[line-1]<270)
				temp1=Bline_left[line-1]+IMG_DIFF;//ȡ��һ�к���ƫ��IMG_DIFF������
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
�������ƣ�test_center
��������: �ҵ������߽�������ͷ��
������
*******************************************************************************/
void test_center()
{
  uint8 line,center=0,put[3]={0};
  for(line=0;line<H;line++)
  {
	if(abs_sub(Pick_table[line],(V/2))<2)
	     center++;
  }
	LCD_Print(8,6,"center:");
    if(run_time%5==0)
 		char_change_1(center,put);
	delay2();
	LCD_Print(72,6,put);
}

/*******************************************************************************
�������ƣ�find_shizi()
��������: Ѱ��ʮ�ֵ�
������line ����ʮ�ֵ������� 
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
�������ƣ�ti_jiaozheng_new
��������: ����ͼ�������ʧ���·���
������
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
�������ƣ�ti_jiaozheng
��������: ����ͼ�������ʧ��
������
*******************************************************************************/
void ti_jiaozheng()
{
  uint16 line;
  int temp_left,temp_right;
  
  for(line=0;line<H;line++)//����ԭʼ�����ұ�����ȡ���
  {
	PrBline_left[line] = Bline_left[line];
	PrBline_right[line] = Bline_right[line];
  }
  
  for(line=0;line<H;line++)
  {
	   //////////////////////////////////����߽���////////////////////////////////
	    if(Bline_left[line]==(V-1))
		  Deal_flag[line] |= DEAL_LEFT_LOST;
    	temp_left=(5000*Bline_left[line]-520439)/(10000-60*(2*line+1))+100; //near
    	if(temp_left<0)
    	  Bline_left[line]=0;
    	else if(temp_left>(V-1))
    	  Bline_left[line]=(V-1);
		else
		  Bline_left[line]=temp_left;
		//////////////////////////////////�ұ��߽���////////////////////////////////
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
�������ƣ�duoji_control
��������: �������
������
*******************************************************************************/
void duoji_control(uint16 jiaodu,uint8 way)
{
  if(way==left_way)
  {
    FTM1_C0V=jiaodu;//dj_center-(jiaodu-dj_center);
    GPIOC_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(10));	//IO��������͵�ƽ,�������
    GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(2));//IO��������ߵ�ƽ
	GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));//IO��������ߵ�ƽ
  }
  else if(way==right_way)
  {
	FTM1_C0V=jiaodu;//dj_center+dj_center-jiaodu;
	GPIOD_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(2));	//IO��������͵�ƽ
	GPIOC_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(10));	//IO��������ߵ�ƽ
	GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));//IO��������ߵ�ƽ
  }
  else
  {  
    FTM1_C0V=dj_center;//�˴�ע���������޸�
    GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(2));//IO��������ߵ�ƽ
    GPIOC_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(10));//IO��������ߵ�ƽ
	 //center_led();
  }
  
}
/*******************************************************************************
�������ƣ�duoji_control
��������: ������Ʊ���
������
*******************************************************************************/
void duoji_control_backup(uint16 jiaodu,uint8 way)
{
  if(way==left_way)
  {
     FTM1_C0V=jiaodu;
     GPIOB_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(21));	//IO��������͵�ƽ,�������
     GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(4));//IO��������ߵ�ƽ
	 GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));//IO��������ߵ�ƽ
  }
  else if(way==right_way)
  {
    FTM1_C0V=jiaodu;
    GPIOD_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(4));	//IO��������͵�ƽ
    GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(21));	//IO��������ߵ�ƽ
	GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));//IO��������ߵ�ƽ
  }
  else
  {  
     FTM1_C0V=dj_center;//�˴�ע���������޸�
     GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(4));//IO��������ߵ�ƽ
     GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(21));//IO��������ߵ�ƽ
	 //center_led();
  }
  
}
/*******************************************************************************
�������ƣ�center_led
��������: ��������ʾ
������
*******************************************************************************/
void center_led()
{
  if(Pick_table[20]==Pick_table[12])
  {
	if(Pick_table[12]==Pick_table[5])
	  GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(20));//IO��������͵�ƽ LED5
  }
  else
  {
  	  GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(20)); //IO��������ߵ�ƽ LED5
  }
}
/*******************************************************************************
�������ƣ�abs_sub
��������: �����޷���������ľ���ֵ
������
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
�������ƣ�mult_f
��������: DSP��Ԫ��˽��Ϊ������
������
*******************************************************************************/
float mult_f(float mult1,float mult2)
{
  float temp;
  arm_mult_f32(&mult1, &mult2, &temp, 1);
  return temp;
}
/*******************************************************************************
�������ƣ�send_data
��������: ���ַ���ʽ��������
������
*******************************************************************************/
void send_data(short int data)
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
�������ƣ�lvbo
��������: �˲�
������
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
�������ƣ�buzzer
��������: ������
������
*******************************************************************************/
void buzzer_on()
{
  buzzer_flag=1;
  buzzer_num=0;
}
/*******************************************************************************
�������ƣ�buzzer
��������: ������
������
*******************************************************************************/
void buzzer_ctl()
{
  if((buzzer_flag==1)&&(buzzer_num<30))
  {
	buzzer_num++;
	GPIOC_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));	//IO������ߵ�ƽ
  }
  else
  {
	buzzer_flag=0;
	buzzer_num=0;
	GPIOC_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(16));	//IO������͵�ƽ
  }
}
/*******************************************************************************
�������ƣ�center_buxian
��������: �˲�
������
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
�������ƣ�bu_xian
��������: �˲�
������
*******************************************************************************/
void bu_xian()
{
  uint8 line;
  uint16 temp1,temp2;
  for(line=4;line<50;line++)
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
�������ƣ�lvbo_cu
��������: �˲�
������
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
�������ƣ�send_some
��������: ����Щ����
��������
*******************************************************************************/
void send_some()
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
�������ƣ�show_miss
��������: �رղɼ�ͼ���ж�
������
*******************************************************************************/
void show_miss(uint8 state)
{
  	if(state==1)
	{
	  	GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(9));//IO��������͵�ƽ
		GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(10));//IO��������͵�ƽ
		GPIOB_PDOR &= ~ GPIO_PDOR_PDO(GPIO_PIN(11));//IO��������͵�ƽ
	}
	else
	{
	  	GPIOB_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(9));//IO��������͵�ƽ
		GPIOB_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(10));//IO��������͵�ƽ
		GPIOB_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(11));//IO��������͵�ƽ
	}
}
/*******************************************************************************
�������ƣ�char_change
��������: ����ʾ�ַ�ת��Ϊ12864��ʾ�ַ�
��������
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
�������ƣ�lcd_int
��������: lcd��ʾ����
��������
*******************************************************************************/
void lcd_int(uint8 position,uint8 *string,int num)
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
  	LCD_Print(position,6,string);
  	LCD_Print(position+24,6,put);
}
/*******************************************************************************
�������ƣ�char_change
��������: ����ʾ�ַ�ת��Ϊ12864��ʾ�ַ�
��������
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
�������ƣ�char_change
��������: ����ʾ�ַ�ת��Ϊ12864��ʾ�ַ�
��������
*******************************************************************************/
void char_change(uint8 num,uint8 *table)
{

	*table=num/100+48;
 	*(table+1)=(num%100)/10+48;
  	*(table+2)=(num%100)%10+48;
}
/*******************************************************************************
�������ƣ�set_ov7620_ld
��������: ����ov7620����
��������
*******************************************************************************/
void set_ov7620_ld()
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
 	 set_ov7620(0x42,0x06,liangdu);//��������ͷ���� 
}
/*******************************************************************************
�������ƣ�Init_7620
��������: ��ʼ��ov7620
��������
*******************************************************************************/
void Init_7620()
{
  if(init_7620_flag==1)
  {
	set_ov7620(0x42,0x13,0x00);//ȡ���Զ��ع�ģʽ
	delay2();
	set_ov7620(0x42,0x06,liangdu);//��������ͷ���� 
	delay2();
	set_ov7620(0x42,0x10,0x50);//��������ͷ�ع�ʱ��
	set_ov7620(0x42,0x14,0x20);//��������ͷ�ֱ��� bit5=1 QVGA 320*240 Bit5=0 VGA 640*480  //20//2c//3c
	LCD_Print(8,0,"ov7620_ok!");//��ʾ��ʼ���ɹ�
	init_7620_flag=0;//���ʼ����־
  }
}
/*******************************************************************************
�������ƣ�set_ov7620_bg
��������: ����ov7620�ع�ʱ��
��������
*******************************************************************************/
void set_ov7620_bg()
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
	  if(baoguang>250)
		baoguang=255;
	  else
	  	baoguang=baoguang+2;
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
	  if(baoguang<3)
		baoguang=0;
	  else
	  	baoguang=baoguang-2;
	  temp_flag=1;
	}
	while(gpio_get(PORTD,5)==0);
  }
  LCD_Print(8,0,"l:");
  char_change_1(baoguang,put);
  LCD_Print(24,0,put);
  if(temp_flag==1)
  {
 	 set_ov7620(0x42,0x10,baoguang);//��������ͷ�ع�ʱ��
  }
}
/*******************************************************************************
�������ƣ�display1
��������: ģʽѡ����
��������
*******************************************************************************/
void display1()
{
  uint8 put_c[3];
  LCD_Print(8,0," CDU--Spower");  //�����ַ�����ʾ
  LCD_Print(8,2,"s:");  //�����ַ�����ʾ
  char_change(ideal_Bmq,put_c);
  LCD_Print(24,2,put_c);  //�����ַ�����ʾ
  char_change(Bmq,put_c);
  LCD_Print(56,2,put_c); 
  char_change(CD_speed,put_c);//���ַ�ÿһλ�ֱ�ȡ����ʾ����
  LCD_Print(88,2,put_c); 
  LCD_Print(8,4,"v_l:");  //�����ַ�����ʾ
  char_change_1(valid_line,put_c);
  LCD_Print(48,4,put_c); 
  /*if(my_num>0)
  	char_change_1((uint8)my_num,put_c);
  else
	char_change_1((uint8)(-my_num),put_c);
  LCD_Print(80,4,put_c); */
  if(roadFlag==0)
  	LCD_Print(72,4,"zhidao!");  //�����ַ�����ʾ
  else if(roadFlag==1)
	LCD_Print(72,4,"wandao!");  //�����ַ�����ʾ
  else
	LCD_Print(72,4,"shizidao!");  //�����ַ�����ʾ
  if(xianshi==0)
  	lcd_int(8,"diff:",even_diff);
  else if(xianshi==1)
	;//lcd_int("xl:",D_slope);
  /*if(my_zhidao==1)
  	LCD_Print(88,6,"1");  //�����ַ�����ʾ
  else
	LCD_Print(88,6,"0");  //�����ַ�����ʾ*/
}
/*******************************************************************************
�������ƣ�displayPar
��������: ������ʾ����
��������
*******************************************************************************/
void displayPar()
{
  char data[7]={0};
  LCD_P6x8Str(8,0," CDU--Spower");  //�����ַ�����ʾ
  LCD_P6x8Str(8,1,"s:");  //�����ַ�����ʾ
  sprintf(data,"%3d",ideal_Bmq);
  LCD_P6x8Str(24,1,(byte *)data);  //�����ַ�����ʾ
  sprintf(data,"%3d",Bmq);
  LCD_P6x8Str(56,1,(byte *)data); 
  sprintf(data,"%3d",CD_speed);
  LCD_P6x8Str(88,1,(byte *)data); 
  LCD_P6x8Str(8,2,"v_l:");  //�����ַ�����ʾ
  sprintf(data,"%3d",valid_line);
  LCD_P6x8Str(24,2,(byte *)data); 
  /*if(my_num>0)
  	char_change_1((uint8)my_num,put_c);
  else
	char_change_1((uint8)(-my_num),put_c);
  LCD_Print(80,4,put_c); */
  if(roadFlag==0)
  	LCD_P6x8Str(72,2,"zhidao!");  //�����ַ�����ʾ
  else if(roadFlag==1)
	LCD_P6x8Str(72,2,"wandao!");  //�����ַ�����ʾ
  else
	LCD_P6x8Str(72,2,"shizidao!");  //�����ַ�����ʾ
  sprintf(data,"diff:%3d",even_diff);
  LCD_P6x8Str(8,3,(byte *)data);
  sprintf(data,"xl:%3d",D_slope);
  LCD_P6x8Str(68,3,(byte *)data);
  sprintf(data,"jd:%5d",jiaodu_num);
  LCD_P6x8Str(8,4,(byte *)data);
  /*if(my_zhidao==1)
  	LCD_Print(88,6,"1");  //�����ַ�����ʾ
  else
	LCD_Print(88,6,"0");  //�����ַ�����ʾ*/
}
/*******************************************************************************
�������ƣ�displayPar
��������: ������ʾ����
��������
*******************************************************************************/
void LCD_P6x8Int(byte x,byte y,byte ch[],int num)
{
  char data[7]={0};
  LCD_P6x8Str(x,y,ch);  //�����ַ�����ʾ
  sprintf(data,"%3d",num);
  LCD_P6x8Str(x+24,y,(byte *)data);  //�����ַ�����ʾ
}
/*******************************************************************************
�������ƣ�displayBlineSimple
��������: ��ʾ���߹��ܺ�������棩
��������
*******************************************************************************/
void displayBlineSimple()
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
�������ƣ�displayBinary
��������: ��ʾ��ֵ��������Ϣ
��������
*******************************************************************************/
void displayBinary()
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
�������ƣ�displayBline
��������: ��ʾ���߹��ܺ��������棩
��������
*******************************************************************************/
void displayBline()
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
�������ƣ�test_lcd
��������: ����Һ����
��������
*******************************************************************************/
void test_lcd()
{
    LCD_Init();
	/*LCD_Fill(0xff);//���� 
	delay2();
	LCD_Fill(0x00);//����
	delay2();
	put_array[0]=(uint8)Kp/10+48;
	put_array[1]=(uint8)Kp%10+48;
	LCD_Print(8,0," CDU--Spower");  //�����ַ�����ʾ
	LCD_Print(8,2,put_array);  //�����ַ�����ʾ*/
}
/*******************************************************************************
�������ƣ�dis_img_irq
��������: �رղɼ�ͼ���ж�
������
*******************************************************************************/
void dis_img_irq()
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
}
/////////////////////////////////////////////////////