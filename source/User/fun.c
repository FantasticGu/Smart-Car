#include "include.h"
#include "fun.h"
//����ͷ����
///////////////////////////////////////////////����ͷ///////////////////////////////////////////////////////////
////////////////////////////////ȫ�ֱ�������////////////////////////////////////
#define test_port UART_0
unsigned int row=0;	//����ͷ�м��������240
uint16 Bline_left[H];	 //����ߴ������
uint16 Bline_right[H];	 //�ұ��ߴ������
uint16 Pick_table[H];	 //�����ߴ������
uint8 Cmp=160;	//������ֵ
uint8 lastCmp=160;



/*******************************************************************************
�������ƣ�find_edge
��������: ��Ѱ��Cmp����Ѱ�ұ�Ե��Ȼ��Ѱ��������
������
*******************************************************************************/
int lastmiddleplace= 94;

void find_edge()
{
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

   for(int line=0;line<H;line++)
    {
      Bline_left[line]=0;
      Bline_right[line]=V-1;
      Pick_table[line]=V/2;
    }
              
    int j;
                     
    for(int i=H-1;i>H-41;i--)
    {
        //�������
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
        //���ұ���
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
       
        //��¼�������
        if(Bline_left[i]!=0 && Bline_right[i]!=V-1)//û�ж���
        {
            Pick_table[i]=(Bline_left[i]+Bline_right[i])/2;
        }
        else if(Bline_left[i]==0 && Bline_right[i]!=V-1)//����߶���
        {; 
            Pick_table[i]=Bline_right[i]/2;
         }
        
        else if(Bline_left[i]!=0 && Bline_right[i]==V-1)//��������,û�ж�����
        {
            Pick_table[i]=(Bline_left[i]+V-1)/2;
        }
        
        else if(Bline_left[i]==0 && Bline_right[i]==V-1)//���߶����˵Ļ�  
        {
            if(i ==H-1)//��������о���ͼ��������Ϊ�е�
            {
                 Pick_table[i] = V/2;
            }       
            else 
            {
                 Pick_table[i] = Pick_table[i+1];//����������о�����һ�е�������Ϊ�����е�
             }             
        }                            
     
    }
                                                
    lastmiddleplace=Pick_table[H-1];                                               
}