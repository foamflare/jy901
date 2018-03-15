#include <Wire.h>
#include <JY901.h>
/*
Test on Uno R3.
JY901    UnoR3
SDA <---> SDA A4
SCL <---> SCL A5
*/
struct Data{
  float temp1;
  float temp2;
};

//下面的数据定义，限于两根手指的约束关系
//其中掌骨的三个数据都需要采集，其他关节只需要控制 x-->euler[1], y-->euler[0],z--->euler[2]
float cmetax,cmetay,cmetaz,cj_indey,cj_indez,cy_indey,cj_midy,cj_midz,cy_midy,cy_indez;   //传感器值，c表示传感器值，j表示近指，y表示中间关节，f远端关节,inde表示食指，mid表示中指
float metax,metay,metaz,j_indey,j_indez,y_indey,f_indey,j_midy,j_midz,y_midy,f_midy;   // 传递的手指关节数据，能直接进行关节的控制
float sum1=0,sum2=0;//传感器差值。用于初始化，其中，就是航向角的问题。其中，MP关节的航向角加上差值就是得到的角度
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
  for(int i = 0;i<100;i++){
      selectMuxPin(0);
      JY901.GetAngle();
      cmetaz=(float)JY901.stcAngle.Angle[2]/32768*180;
      //选择第二个传感器，食指近指，
      selectMuxPin(1);
      JY901.GetAngle();
      cj_indez = (float)JY901.stcAngle.Angle[2]/32768*180;
      //选择第三个传感器，中指近指，
      selectMuxPin(2);
      JY901.GetAngle();
      cj_midz = (float)JY901.stcAngle.Angle[2]/32768*180;
      sum1 = sum1+ (cmetaz-cj_indez);
      sum2 = sum2+ (cmetaz-cj_midz);
    }
    sum1 = sum1/100.0;
    sum2 = sum2/100.0;
}
void loop() 
{
  //使用的blender模型中都是XYZ欧拉角度，如果不是，请将关节进行更改。
  //其中，在对blender传输数据时
  //其中传感器的 y 轴数据对应模型的 euler[0],也就是关节主要运动的关节。关节和父关节之间的角度差为正值。PIP
  //             x 轴数据 主要控制  euler[1]，控制关节本身自己的旋转，这个数据可以不接受
  //             z 轴数据对应模型的 euler[2]，控制MP关节的运动。MP,只有近指才会发生。
  //blender模型传递子类和父类的角度差距数据，不是直接读取的传感器数据
  //选择主传感器，手掌传感器
  selectMuxPin(0);
  JY901.GetAngle();
  cmetax=(float)JY901.stcAngle.Angle[0]/32768*180;
  cmetay=(float)JY901.stcAngle.Angle[1]/32768*180;
  cmetaz=(float)JY901.stcAngle.Angle[2]/32768*180;
  //选择第二个传感器，食指近指，
  selectMuxPin(1);
  JY901.GetAngle();
  cj_indey = (float)JY901.stcAngle.Angle[1]/32768*180;
  cj_indez = (float)JY901.stcAngle.Angle[2]/32768*180+sum1;
  //选择第三个传感器，中指近指，
  selectMuxPin(2);
  JY901.GetAngle();
  cj_midy = (float)JY901.stcAngle.Angle[1]/32768*180;
  cj_midz = (float)JY901.stcAngle.Angle[2]/32768*180+sum2;
  //选择第四个传感器，食指远指，
  selectMuxPin(3);
  JY901.GetAngle();
  cy_indey = (float)JY901.stcAngle.Angle[1]/32768*180;
  cy_indez = (float)JY901.stcAngle.Angle[2]/32768*180;
  //选择第五个传感器，中指远指，
  selectMuxPin(4);
  JY901.GetAngle();
  cy_midy = (float)JY901.stcAngle.Angle[1]/32768*180;

 //Serial.print(cj_indey);Serial.print(",");Serial.print(cj_indez);Serial.print(",");Serial.print(cy_indey);Serial.print(",");Serial.println(cy_indez);
//  Serial.print(cj_indey);Serial.print(",");Serial.print(cj_indez);Serial.print(",");Serial.print(cy_indey);
//  Serial.print(cj_midy);Serial.print(",");Serial.print(cj_midz);Serial.print(",");Serial.println(cy_midy);
  //下面使用手掌模型，对传感器数据进行处理。并且将数据进行输出。
  //先处理掌骨数据，长模型中，可以看出，掌骨数据和传感器数据正好是相反的，因此
  metax = -cmetax;metay = -cmetay;metaz = -cmetaz;
  //处理近指掌骨
  Data temp;
  j_indey = getJ(cmetay,cmetaz,cj_indey,cj_indez).temp1;
  j_indez = getJ(cmetay,cmetaz,cj_indey,cj_indez).temp2;
  j_midy = getJ(cmetay,cmetaz,cj_midy,cj_midz).temp1;
  j_midz = getJ(cmetay,cmetaz,cj_midy,cj_midz).temp2;
  //处理中指掌骨数据
  y_indey = getM(cj_indey,cy_indey);
  y_midy = getM(cj_midy,cy_midy);
  //处理远指数据
  f_indey = getF(cj_indey,cy_indey);
  f_midy = getF(cj_midy,cy_midy);
  //上面对数据处理完毕，进行数据的传输
  Serial.print(metax);Serial.print(",");Serial.print(metay);Serial.print(",");Serial.print(metaz);Serial.print(","); //掌骨数据
  Serial.print(j_indey);Serial.print(",");Serial.print(j_indez);Serial.print(",");Serial.print(y_indey);Serial.print(",");Serial.print(f_indey);Serial.print(",");//食指近指远指数据
  Serial.print(j_midy);Serial.print(",");Serial.print(j_midz);Serial.print(",");Serial.print(y_midy);Serial.print(",");Serial.println(f_midy);//食指近指远指数据
  //数据格式：掌骨的x  y  z   食指近指 y z  食指中指 y  食指远指 y  中指近指 y z  中指中指 y  中指远指 y  
}
//使用掌骨和近指传感器数据，得到模型中近指的实际角度数据。其中涉及到mp关节，其中y控制上下弯曲，z控制左右弯曲
Data getJ(float zhangy,float zhangz,float datay,float dataz)
{
    Data temp;
    //下面对上下波动进行控制
    temp.temp1 = zhangy-datay;
    if(temp.temp1<0)temp.temp1 = - temp.temp1;
    if(temp.temp1>90)temp.temp1 = 90;
    //下面对左右晃动进行控制
    temp.temp2 = zhangz - dataz;
    if(temp.temp2<-10)temp.temp2 = -10;
    if(temp.temp2>10)temp.temp2 = 10;
    return temp;
}
//使用近指数据和中指传感器数据，获取模型中中指数据.PIP。jy表示近指传感器y轴数据，My表示中指传感器y轴数据
float getM(float jy,float My)
{
    float data = jy-My;
    if(data<0){
        data = -data;
      }
    if(data>110){
        data = 110;
      }
    return data;
}
//使用近指数据和中指弯曲数据，推出远指弯曲
float getF(float jy,float My)
{
    float data = jy-My;
    if(data<0){
        data = -data;
      }
    if(data>110){
        data = 110;
      } 
    data = 0.6777 *data;
    return data;
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


