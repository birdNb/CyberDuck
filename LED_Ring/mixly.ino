//版权作者：慧净电子 2021年10月27日（转载本源码注明版权出处）
//硬件配置：Arduino UNO R3控制器+HJ-RGB灯带
//接线方法：灯带线颜色：红色=5V正，黑色=GND负，黄色=Din控制信号（接到数字I/O口9）
//实验效果：依次点亮RGB的三种颜色，再实现三种颜色的流水灯效果，最后实现RGB的配色操作。
//实验目的：让新手了解RGB灯内部结构，实现点亮RGB的方法，实现创意。

#include <Wire.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel  rgb_display_9(16);//定义PWM引脚为数字I/O口9，LED灯数量为60.

void setup()
{
  rgb_display_9.begin(); //引脚初始化
  rgb_display_9.setPin(9);
}

void loop(){
  for (int i = 1; i <= 16; i = i + (1)) //定义i的变量从1到60，步长为1，取决于灯珠数量，可自行设置。
  {
    rgb_display_9.setPixelColor(i-1, 255,0,0); //对RGB进行配色，R代表红色，G代表绿色，B代表蓝色；PWM值为0-255，最大255值颜色最亮
    rgb_display_9.show();//开始显示
  }
  delay(300); //显示演示，可自行修改。
  for (int i = 1; i <= 16; i = i + (1)) 
  {
    rgb_display_9.setPixelColor(i-1, 0,255,0);//显示绿色
    rgb_display_9.show();
  }
  delay(300);
  for (int i = 1; i <= 16; i = i + (1))
  {
    rgb_display_9.setPixelColor(i-1, 0,0,255);//显示蓝色
    rgb_display_9.show();
  }
  delay(300);

  for (int i = 1; i <= 16; i = i + (1)) 
  {
    rgb_display_9.setPixelColor(i-1, 255,0,0);//实现红色流水灯效果
    rgb_display_9.show();
    delay(10); //每个LED依次点亮时间，实现流动效果。
  }
  for (int i = 1; i <= 16; i = i + (1)) 
  {
    rgb_display_9.setPixelColor(i-1, 0,255,0); //实现绿色流水灯效果
    rgb_display_9.show();
    delay(10);
  }
  for (int i = 1; i <= 16; i = i + (1)) {
    rgb_display_9.setPixelColor(i-1, 0,0,255); //实现蓝色流水灯效果
    rgb_display_9.show();
    delay(10);
  }
  for (int i = 1; i <= 16; i = i + (1)) {
    rgb_display_9.setPixelColor(i-1, 180,50,50); //RGB配色为紫红色
    rgb_display_9.show();
    delay(10); //每个LED依次点亮时间，实现流动效果。
  }

}
