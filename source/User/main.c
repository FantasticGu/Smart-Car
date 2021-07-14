//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4
/*!                    北京邮电大学 K66 学习例程
 *  文件名称：       main.c
 *      作者：       maddevil
 *      说明：       仅做内部学习使用，请勿外传
 *  参考资料：       历届学长代码、山外K60库、龙邱K66模板、北邮KEA模板
 *    版本号：       V1.0.0
 *  最后更新：       2018-12-21 13:41
 */                                         
//QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4QlVQVC1LNjYgbWFkZGV2aWwgNzkzNTU4NzU4


#include "include.h" 
//主函数
#define test_port UART_0
//#if -#elif -#endif  是预编译命令， 1表示要编译的内容，测试I2C
#if 1
//串口打印 “Hello world!void main()
void main()
{

  //todo: change tha algo in 大津法 to 计算新的阈值
  
  DisableInterrupts;
  pit_init(PIT0, 100);
  uart_init(test_port, 115200); 
  LQMT9V034_Init();                         
  ftm_quad_init(FTM1);
  ftm_quad_init(FTM2);//ftm正交解码计数初始化
  motor_init();       //电机初始化
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

