//三路传感器
#define sensor1 19    //左侧2
#define sensor2 7    //左侧
#define sensor3 14   //中间  
#define sensor4 15   //右侧
#define sensor5 18   //右侧2
//自带灯定义
#define RED_LED 2
//蜂鸣器端口定义
#define Buzz 11 //控制蜂鸣器

//左电机端口定义
//#define MotorLpin1   11//控制位3
//#define MotorLpin2   15//控制位4
#define MotorLpwm     9//使能调速 ENB
#define MotorLcountA  10//编码器B 

//右电机端口定义
//#define MotorRpin1   8 //控制位1
//#define MotorRpin2   7  //控制位2
#define MotorRpwm     12  //使能调速 ENA
#define MotorRcountA 13  //编码器B

#include <hcrs04.h>

#define PINTRIG 6
#define PINECHO 5

hcrs04 mySensor(PINTRIG, PINECHO);
//循迹
float k_TrackB_1=1.1,k_TrackS_1=0.8;                //低速状态：2 0.6
float RequiredSpeed=120;                       //pwm低速 ：76.5
float RequiredValue=RequiredSpeed;
float k_TrackB_2=1.4,k_TrackS_2=0.6;              //大转弯参数   
float k_TrackB_3=1.3,k_TrackS_3=0.4;                
//速度环
volatile float motorL=0;//中断变量，左轮子脉冲计数
volatile float motorR=0;//中断变量，右轮子脉冲计数
float V_L=0; //左轮速度 单位cm/s
float V_R=0; //右边轮速 单位cm/s
int v1=0;  //单位cm/s
int v2=0;  //单位cm/s
float Target_V_L=8,Target_V_R=8;   //目标速度，单位cm/s
int Pwm_L=0,Pwm_R=0;  //左右轮PWM
int speedL,speedR;
int error;
float CurrentDistance;
float TargetDistance=20;
float Dis_p=1.5,Dis_i=0.1;
float I=0;

float TRACK_MODE=0;
int sensor[5];

static int TIME_Mode=0;
static int TIME_Value=0;

float kp=2,ki=0.1,kd=0.08;  //PID参数(控速)
int MODE=-1;
void ReadDistance()
{
  CurrentDistance=mySensor.read();
}
void Dis_Stop()                                   //暂时先这样，可以换成平均值或者中间值
{
  ReadDistance();
  if(CurrentDistance<28)
  {
  Stop();
  MODE=0;
  }
}
/********循迹************/
void Track()
{
   if(MODE==3&&TRACK_MODE==1)read_sensor_values_3();
   else read_sensor_values();
   
   switch(error){
    case -3:
      Set_Pwm(k_TrackB_3*RequiredSpeed,k_TrackS_3*RequiredSpeed);
      delay(10);
      break;
    case -2:
      Set_Pwm(k_TrackB_2*RequiredSpeed,k_TrackS_2*RequiredSpeed);
      break;
    case -1:
      Set_Pwm(k_TrackB_1*RequiredSpeed,k_TrackS_1*RequiredSpeed);
      break;
    case 0:
      Set_Pwm(RequiredSpeed,RequiredSpeed);
      break;
    case 1:
      Set_Pwm(k_TrackS_1*RequiredSpeed,k_TrackB_1*RequiredSpeed);
      break;
    case 2:
      Set_Pwm(k_TrackS_2*RequiredSpeed,k_TrackB_2*RequiredSpeed);
      break;
    case 3:
      Set_Pwm(k_TrackS_3*RequiredSpeed,k_TrackB_3*RequiredSpeed);
      delay(10);
      break;
   }
}
void read_sensor_values_3()
{

  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);
  sensor[4] = digitalRead(sensor5);
  
  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) 
  {
      error = 2;//          0 0 0 0 1
    }
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0))
    {
      error = 1;//          0 0 0 1 0
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      error = 0;//          0 0 1 0 0
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      error = -3;//          0 1 1 0 0
      if(millis()-TIME_Value>=6000)
      {
          TRACK_MODE=0;
      }
      digitalWrite(RED_LED,HIGH);
      //RequiredSpeed=255;                       //pwm低速 ：76.5
      TIME_Value=millis();
    } 
    else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      error = -3;//          1 0 1 0 0
      if(millis()-TIME_Value>=6000)
      {
          TRACK_MODE=0;
      }
      digitalWrite(RED_LED,HIGH);
     
      TIME_Value=millis();
    } 
    else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      error = -3;//          1 1 0 0 0
      if(millis()-TIME_Value>=6000)
      {
          TRACK_MODE=0;
      }
      digitalWrite(RED_LED,HIGH);
     
      TIME_Value=millis();
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) 
    {
      error = -1;//         0 1 0 0 0
    } 
    else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) 
    {
      error = -2;//         1 0 0 0 0
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      if (error == -2) 
      {//  0 0 0 0 0
        error =-3;
      }
      else if(error==2)
      {
        error = 3;
      }
    }
}
void read_sensor_values()
{

  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);
  sensor[4] = digitalRead(sensor5);
  
  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) 
  {
      error = 2;//          0 0 0 0 1
    }
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0))
    {
      error = 1;//          0 0 0 1 0
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      error = 0;//          0 0 1 0 0
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      error = 0;//          0 1 1 0 0
      if(millis()-TIME_Value>=6000)
      {
          TRACK_MODE=1;
      }
      TIME_Value=millis();
    } 
    else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      error = 0;//          1 0 1 0 0
      if(millis()-TIME_Value>=6000)
      {
          TRACK_MODE=1;
      }
      TIME_Value=millis();
    } 
    else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      error = 0;//          1 1 0 0 0
      if(millis()-TIME_Value>=6000)
      {
          TRACK_MODE=1;
      }
      TIME_Value=millis();
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))
    {
      error = 1;//          0 0 1 1 0
      digitalWrite(RED_LED,LOW);
      //RequiredSpeed=245;
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 1))
    {
      error = 2;//          0 0 1 0 1
      digitalWrite(RED_LED,LOW);
      //RequiredSpeed=245;
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))
    {
      error = 3;//          0 0 0 1 1
      digitalWrite(RED_LED,LOW);
      //RequiredSpeed=245;
      
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) 
    {
      error = -1;//         0 1 0 0 0
    } 
    else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) 
    {
      error = -2;//         1 0 0 0 0
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      if (error == -2) 
      {//  0 0 0 0 0
        error =-3;
      }
      else if(error==2)
      {
        error = 3;
      }
    }
}
/***************************************************************************************************************************************************/
/*********************************************************
 * 函数功能：增量式PI控制器(左轮)
 *********************************************************/
/* int Incremental_Pi_L(int current_speed,int target_speed){
  static float pwm,bias,last_bias,prev_bias;  //静态变量存在程序全周期：pwm:增量输出,bias:本次偏差,last_bias:上次偏差,prev_bais_:上上次偏差
  bias=current_speed-target_speed;    //计算本次偏差e(k)
  pwm-=(kp*(bias-last_bias)+ki*bias+kd*(bias-2*last_bias+prev_bias));   //增量式PID控制器
  prev_bias=last_bias;  //保存上上次偏差
  last_bias=bias;     //保存上一次偏差
  //PWM 限幅度  Arduino的PWM 最高为255  限制在250
  if(pwm<-250){
    pwm=250;     
  }
  if(pwm>250){
    pwm=250;  
  }
  //Serial.println(pwm);
  return pwm;         //增量输出
 }
 
//右轮速度增量式PID控制器
int Incremental_Pi_R(float current_speed,float target_speed){
  static float pwm,bias,last_bias,prev_bias;  //静态变量存在程序全周期：pwm:增量输出,bias:本次偏差,last_bias:上次偏差,prev_bais_:上上次偏差
  bias=current_speed-target_speed;    //计算本次偏差e(k)
  pwm-=(kp*(bias-last_bias)+ki*bias+kd*(bias-2*last_bias+prev_bias));   //增量式PID控制器
  prev_bias=last_bias;  //保存上上次偏差
  last_bias=bias;     //保存上一次偏差
 
  //PWM 限幅度  Arduino的PWM 最高为255限制在250
  if(pwm<-250){
    pwm=250;     
  }
  if(pwm>250){
    pwm=250;  
  }
  //Serial.println(pwm);
  return pwm;         //增量输出
 }
 */
void Set_Pwm(int speed_L,int speed_R){
  //前进模式
  //左电机
 // digitalWrite(MotorLpin1,HIGH);
  //digitalWrite(MotorLpin2,LOW);
  analogWrite(MotorLpwm,speed_L);
  //右电机
  //digitalWrite(MotorRpin1,HIGH);
  //digitalWrite(MotorRpin2,LOW);
  analogWrite(MotorRpwm,speed_R);
 // Serial.println("lello");
}
 
/***********************************
 * 电机实际速度计算：
 * 公式：
 * 已知参数：
 *     车轮直径65mm,
 *     左边轮子一圈：390脉冲（RISING）,
 *     右边轮子一圈：390脉冲（RISING），
 * 单位时间读两个轮子脉冲读取两个轮子脉冲
 ***********************************/
/* void Read_Moto_V(){
  unsigned long nowtime=0;
  motorL=0;
  motorR=0;
  nowtime=millis()+50;//读50毫秒
  attachInterrupt(digitalPinToInterrupt(MotorLcountA),Read_Moto_L,RISING);//左轮脉冲开中断计数
  attachInterrupt(digitalPinToInterrupt(MotorRcountA),Read_Moto_R,RISING);//右轮脉冲开中断计数
  while(millis()<nowtime); //达到50毫秒关闭中断
  detachInterrupt(digitalPinToInterrupt(MotorLcountA));//左轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorRcountA));//右轮脉冲关中断计数
  //Serial.println(motorL*100);
  //Serial.println(motorR*100);
  V_L=((motorL/390)*6.5*PI)/0.05;   //单位cm/s
  V_R=((motorR/390)*6.5*PI)/0.05;   //单位cm/s
  //Serial.println(V_L);
  v1=V_L;
  v2=V_R;
}*/
 
 
/***************************
 * 中断函数：读左轮脉冲
 **************************/
/*void Read_Moto_L(){
  motorL++;
}*/
 
 
/**************************
 * 中断函数：读右轮脉冲
 *************************/
/*void Read_Moto_R(){
  motorR++;
}
 
//前进函数
void RUN(int RV,int LV){
   Target_V_L=LV,Target_V_R=RV;
   Read_Moto_V();//读取脉冲计算速度
   Pwm_L=Incremental_Pi_L(V_L,Target_V_L);//左轮PI运算
   Pwm_R=Incremental_Pi_R(V_R,Target_V_R);//右轮PI运算
   //Serial.println(V_R);  //直接用串口绘图画出速度曲线
   Set_Pwm(Pwm_L,Pwm_R);  //设置左右轮速度
}*/

//终点停车函数
void Stop(){
   //左电机
   //digitalWrite(MotorLpin1,LOW);
   //digitalWrite(MotorLpin2,LOW);
   //右电机
   //digitalWrite(MotorRpin1,LOW);
   //digitalWrite(MotorRpin2,LOW);
   Set_Pwm(0,0);
   //声音提示
   digitalWrite(Buzz,HIGH);
   delay(500);
   digitalWrite(Buzz,LOW);
   delay(1000);
   
}
/**************************************
 * 初始化函数
 *************************************/
void setup() {
   Motor_Init();//电机端口初始化
   pinMode(sensor1,INPUT);  //传感器初始化
   pinMode(sensor2,INPUT);
   pinMode(sensor3,INPUT);
   pinMode(sensor4,INPUT);
   pinMode(sensor5,INPUT);
   Serial.begin(38400);//开启串口
   mySensor.begin(); /* Initialize the sensor */
   pinMode(Buzz,OUTPUT);//蜂鸣器初始化
   pinMode(RED_LED,OUTPUT);//蜂鸣器初始化
   digitalWrite(RED_LED,LOW);
   digitalWrite(Buzz,LOW);

}
 
void Motor_Init(){
  //左电机
  //pinMode(MotorLpin1,OUTPUT);  //驱动芯片控制引脚
  //pinMode(MotorLpin2,OUTPUT);  //驱动芯片控制引脚  
  pinMode(MotorLpwm,OUTPUT);   //驱动芯片控制引脚，PWM调速
  pinMode(MotorLcountA,INPUT); //左轮编码器B引脚
  
  //右电机
  //pinMode(MotorRpin1,OUTPUT);  //驱动芯片控制引脚
  //pinMode(MotorRpin2,OUTPUT);  //驱动芯片控制引脚  
  pinMode(MotorRpwm,OUTPUT);   //驱动芯片控制引脚，PWM调速
  pinMode(MotorRcountA,INPUT); //右轮编码器B引脚
 
  //驱动芯片控制引脚全部拉低
  //digitalWrite(MotorLpin1,LOW); //左电机
  //digitalWrite(MotorLpin2,LOW);
  digitalWrite(MotorLpwm,LOW);
  //digitalWrite(MotorRpin1,LOW); //右电机
  //digitalWrite(MotorRpin2,LOW);
  digitalWrite(MotorRpwm,LOW);
}

 
/***************************************
 * 主循环
 ***************************************/
void loop() 
{
   /*read_sensor_values();
   Serial.print(sensor[0]);
   Serial.print(sensor[1]);
   Serial.print(sensor[2]);
   Serial.print(sensor[3]);
   Serial.println(sensor[4]);*/
   //CalculateDistance();
   //Set_Pwm(k_TrackB_2*RequiredSpeed,k_TrackS_2*RequiredSpeed);
   while(Serial.available()>0)
   {
    char com_data=Serial.read();
    switch(com_data){
      case '0':MODE=10;break;
      case '1': 
          MODE=1;
          k_TrackB_1=1.1,k_TrackS_1=0.8;                //低速状态：2 0.6
          RequiredSpeed=110;                       //pwm低速 ：30cm/s
          k_TrackB_2=1.4,k_TrackS_2=0.6;              //大转弯参数   
          k_TrackB_3=1.3,k_TrackS_3=0.4;            
          break;
      case '2':
          MODE=2;
          k_TrackB_1=1,k_TrackS_1=0.9;            //低速状态：2 0.6
          k_TrackB_2=1,k_TrackS_2=0.5;              //大转弯参数   
          k_TrackB_3=1,k_TrackS_3=0.2;            
          RequiredSpeed=245;                       //pwm低速 ：50cm/s   
          break;
      case '3':
          MODE=3;
          k_TrackB_1=1.1,k_TrackS_1=0.8;                //低速状态：2 0.6
          RequiredSpeed=130;                       //pwm低速 ：30cm/s
          k_TrackB_2=1.4,k_TrackS_2=0.6;              //大转弯参数   
          k_TrackB_3=1.3,k_TrackS_3=0.4;       
          break;
      case '6':
          Set_Pwm(RequiredSpeed,RequiredSpeed);
          delay(500);
          Set_Pwm(k_TrackS_3*RequiredSpeed,k_TrackB_3*RequiredSpeed);
          delay(150);
          Set_Pwm(0,0);
          
      
    }
    
   
   }   
 
   if(MODE==1||MODE==2||MODE==10||MODE==3)
    {
      Track();
    }
    else if(MODE==0)
    {
      Set_Pwm(0,0);
    }
    
    if(MODE==10){Dis_Stop();} 
    else if(MODE==3||MODE==2)
    {
      ReadDistance();
      if(CurrentDistance<35)
      {
          k_TrackB_1=1.1,k_TrackS_1=0.8;                //低速状态：2 0.6
          RequiredSpeed=120;                       //pwm低速 ：30cm/s
          k_TrackB_2=1.4,k_TrackS_2=0.6;              //大转弯参数   
          k_TrackB_3=1.3,k_TrackS_3=0.4;            
                          
      }
      else
      {
        k_TrackB_1=1,k_TrackS_1=0.9;             //第二题保持距离
        k_TrackB_2=1,k_TrackS_2=0.4;            
        k_TrackB_3=1,k_TrackS_3=0.2;            
        RequiredSpeed=255;                        
      }
    }
   
}
