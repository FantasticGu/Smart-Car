//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4
/*!                    �����ʵ��ѧ K66 ѧϰ����
 *  �ļ����ƣ�       main.c
 *      ���ߣ�       maddevil
 *      ˵����       �����ڲ�ѧϰʹ�ã������⴫
 *  �ο����ϣ�       ����ѧ�����롢ɽ��K60�⡢����K66ģ�塢����KEAģ��
 *    �汾�ţ�       V1.0.0
 *  �����£�       2018-12-21 13:41
 */                                         
//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4


#include "include.h" 
//������
#define test_port UART_0
//#if -#elif -#endif  ��Ԥ������� 1��ʾҪ��������ݣ�����I2C
#if 1
//���ڴ�ӡ ��Hello world!��
void main()
{
  
  DisableInterrupts;
  pit_init(PIT0, 10);
  //uart_init(test_port, 256000); 
  LQMT9V034_Init();                         
  ftm_quad_init(FTM1);
  ftm_quad_init(FTM2);//ftm�������������ʼ��
  motor_init();                                    //�����ʼ��
  steer_init();
  //uart_rx_irq_en(UART_0);
  pit_irq_en(PIT0);
  EnableInterrupts;
  imagineProcess();

}

#endif

