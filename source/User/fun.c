#include "include.h"
#include "fun.h"
//摄像头参数
///////////////////////////////////////////////摄像头///////////////////////////////////////////////////////////
////////////////////////////////全局变量定义////////////////////////////////////
#define test_port UART_0
#define alpha 0.875
unsigned int row=0;	//摄像头行计数，最大240
uint16 Bline_left[H];	 //左边线存放数组
uint16 Bline_right[H];	 //右边线存放数组
uint16 Pick_table[H];	 //中心线存放数组
uint8 Cmp=180;	//黑线阈值

uint8 GetAvg(uint8 data[120][188],int row,int col)
{
 uint32 sum = 0;
 for(int i = 0;i<row;i++)
 {
  for(int j = 0;j < col;j++)
  {
    sum += data[i][j];
  }
 }
  return sum/(row * col);
}


/*******************************************************************************
函数名称：find_edge
函数功能: 先寻找Cmp，再寻找边缘，然后寻找中心线
参数：
*******************************************************************************/
int lastmiddleplace= 94;

void find_edge()
{
  uint8 tmp = GetAvg(Image_Data,120,188);//GetOSTU(Image_Data);
  Cmp = (int)(Cmp*0.875+tmp*(1-0.875));

   for(int line=0;line<H;line++)
    {
      Bline_left[line]=0;
      Bline_right[line]=V-1;
      Pick_table[line]=V/2;
    }
              
    int j;
                     
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
       
        //记录丢线情况
        if(Bline_left[i]!=0 && Bline_right[i]!=V-1)//没有丢线
        {
            Pick_table[i]=(Bline_left[i]+Bline_right[i])/2;
        }
        else if(Bline_left[i]==0 && Bline_right[i]!=V-1)//左边线丢了
        {; 
            Pick_table[i]=Bline_right[i]/2;
         }
        
        else if(Bline_left[i]!=0 && Bline_right[i]==V-1)//丢了右线,没有丢左线
        {
            Pick_table[i]=(Bline_left[i]+V-1)/2;
        }
        
        else if(Bline_left[i]==0 && Bline_right[i]==V-1)//两边都丢了的话  
        {
            if(i ==H-1)//如果是首行就以图像中心作为中点
            {
                 Pick_table[i] = V/2;
            }       
            else 
            {
                 Pick_table[i] = Pick_table[i+1];//如果不是首行就用上一行的中线作为本行中点
             }             
        }                            
     
    }
                                                
    lastmiddleplace=Pick_table[H-1];                                               
}