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
//���ڴ�ӡ ��Hello world!void main()
void main()
{

  //todo: change tha algo in ��� to �����µ���ֵ
  
  DisableInterrupts;
  pit_init(PIT0, 100);
  uart_init(test_port, 115200); 
  LQMT9V034_Init();                         
  ftm_quad_init(FTM1);
  ftm_quad_init(FTM2);//ftm�������������ʼ��
  motor_init();       //�����ʼ��
  steer_init();
  gpio_init(PTD12,GPO,0);
  my_steer_init();
  uart_rx_irq_en(UART_0);
  pit_irq_en(PIT0);
  EnableInterrupts;
  //delayms(100);
  //ftm_pwm_duty(FTM3,FTM_CH7,5000);
  //uart_putchar(UART_0,'a');
  //delayms(100000);
  imagineProcess();

}

#endif

