#include <Wire.h>
#include <JY901.h>
/*
Test on Uno R3.
JY901    UnoR3
SDA <---> SDA A4
SCL <---> SCL A5
*/
unsigned long time1 = 0;
const int selectPins[3] = {2, 3, 4}; 
void setup() 
{
  Serial.begin(38400);  
  for (int i=0; i<3; i++)
  {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], HIGH);
  }
  JY901.StartIIC();
}
void loop() 
{
  //使用的blender模型中都是XYZ欧拉角度，如果不是，请将关节进行更改。
  //其中，在对blender传输数据时
  //其中传感器的 y 轴数据对应模型的 euler[0],也就是关节主要运动的关节。关节和父关节之间的角度差为正值。PIP
  //             x 轴数据 主要控制  euler[1]，控制关节本身自己的旋转，这个数据可以不接受
  //             z 轴数据对应模型的 euler[2]，控制MP关节的运动。MP
  //blender模型传递子类和父类的角度差距数据，不是直接读取的传感器数据
  //下面的数据定义，限于两根手指的约束关系
  float metax,metay,metaz,indey,indez,
  selectMuxPin(0);
  time1 = micros();
  Serial.print(time1); Serial.print(",");
  JY901.GetAngle();
  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[2]/32768*180,5);

  //选择第二个传感器
  selectMuxPin(1);
  JY901.GetAngle();
  Serial.print(",");
  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[2]/32768*180,5);

//  //选择第三个传感器
  selectMuxPin(2);
  JY901.GetAngle();
  Serial.print(",");
  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[2]/32768*180,5);

//  //选择第四个传感器
  selectMuxPin(3);
  JY901.GetAngle();
  Serial.print(",");
  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[2]/32768*180,5);

  //  //选择第五个传感器
  selectMuxPin(4);
  JY901.GetAngle();
  Serial.print(",");
  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180,5);Serial.print(",");Serial.println((float)JY901.stcAngle.Angle[2]/32768*180,5);
//  JY901.GetGyro();
//  Serial.print((float)JY901.stcGyro.w[0]/32768*2000,5);Serial.print(",");Serial.print((float)JY901.stcGyro.w[1]/32768*2000,5);Serial.print(",");Serial.println((float)JY901.stcGyro.w[2]/32768*2000,5);
  //  Serial.println("");
  //  delay(500);
}
void selectMuxPin(byte pin)
{
  for (int i=0; i<3; i++)
  {
    if (pin & (1<<i))
      digitalWrite(selectPins[i], HIGH);
    else
      digitalWrite(selectPins[i], LOW);
  }
}


