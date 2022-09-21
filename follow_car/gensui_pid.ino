//五路传感器
#define sensor1 P4_3   //1
#define sensor2 P4_0   //2
#define sensor3 P3_7   //3
#define sensor4 P4_2   //4
#define sensor5 P4_1   //5

//左电机端口定义
#define MotorLpin1   P3_2//控制位3
#define MotorLpin2   P6_4//控制位4
#define MotorLpwm    P3_5 //使能调速 ENB
#define MotorLcountA P1_3 //编码器B 


//右电机端口定义
#define MotorRpin1   P7_0//控制位1
#define MotorRpin2   P2_7//控制位2
#define MotorRpwm    P3_6 //使能调速 ENA
#define MotorRcountA P1_2 //编码器B

#define Key_1 P1_6
#define Key_2 P8_2
#define Key_3 P6_6
#define Key_4 P2_6

//蜂鸣器端口定义
#define Buzz P3_0 //控制蜂鸣器
//速度环
volatile float motorL=0;//中断变量，左轮子脉冲计数
volatile float motorR=0;//中断变量，右轮子脉冲计数
float V_L=0; //左轮速度 单位cm/s
float V_R=0; //右边轮速 单位cm/s
int v1=0;  //单位cm/s
int v2=0;  //单位cm/s
float Target_V_L=20,Target_V_R=20;   //目标速度，单位cm/s
int Pwm_L=0,Pwm_R=0;  //左右轮PWM
int value,mode,state,turn = 0;
int error,Mode_Tag,ss,interrupt;
int a = 1,b = 1,c = 1,d = 1,key =1,sent1 = 1,sent2 = 1;
unsigned long recenttime,nowtime;
unsigned long begintime;

int dengting=0;
//PID变量
float kp=2.4,ki=0.2,kd=0.1;  //PID参数速度环

float k_TrackB_1,k_TrackS_1;            //低速状态：2 0.6
float k_TrackB_2,k_TrackS_2;              //大转弯参数   
float k_TrackB_3,k_TrackS_3;            
float RequiredSpeed;                       //pwm低速 ：85->30cm/s  95->50cm/s

/**************************************
 * 初始化函数
 *************************************/
void setup() 
{
   Motor_Init();//电机端口初始化
   //循迹初始化
   Sensor_Init();
   Key_Init();
   Serial1.begin(38400);//开启串口
   pinMode(Buzz,OUTPUT);//蜂鸣器初始化
   digitalWrite(Buzz,LOW);
}

/***************************************
 * 主循环
 ***************************************/
void loop() 
{
  Key_Scan();
  Track();
}

void Key_Init()
{
  pinMode(Key_1,INPUT_PULLUP);
  pinMode(Key_2,INPUT_PULLUP);
  pinMode(Key_3,INPUT_PULLUP);
  pinMode(Key_4,INPUT_PULLUP);
}

void Key_Scan()
{
  int data[4];
  static int sss=0;
  data[0]=digitalRead(Key_1);
  data[1]=digitalRead(Key_2);
  data[2]=digitalRead(Key_3);
  data[3]=digitalRead(Key_4);
  
  if(data[0] == 0)
  {
    interrupt = 1;
    if(sss==0)sss++;delay(1000);
    mode = 1;
  }
  if(data[1] == 0) 
  {
    interrupt = 2;
    mode = 2;
    if(sss==0)sss++;delay(1000);
  }
  if(data[2] == 0)
  {
    mode = 3;
    key = 0;
    interrupt = 3;
    if(sss==0)sss++;delay(1000);
  }
  if(data[3] == 0) 
  {
    key = 0;
    mode = 4;
    interrupt = 1;
    if(sss==0)sss++;delay(1000); 
  }
}

void Mode_Control()
{
  if( mode == 0)
  {
  }
  else if(mode == 1)
  {
    //pwm低速 ：75->30cm/s */
    k_TrackB_1=1.1,k_TrackS_1=0.8;                //低速状态：2 0.6
    RequiredSpeed=140;                       //pwm低速 ：30cm/s
    k_TrackB_2=1.4,k_TrackS_2=0.6;              //大转弯参数   
    k_TrackB_3=1.3,k_TrackS_3=0.4;          
    if(a == 1)
    {
      Serial1.println('1');
      a = a - 1;
    }
  }
  else if(mode == 2)
  { 
     k_TrackB_1=1,k_TrackS_1=0.9;             //第二题保持距离
     k_TrackB_2=1,k_TrackS_2=0.4;            
     k_TrackB_3=1,k_TrackS_3=0.2;     
     RequiredSpeed=240;
     
     if(b == 1)
     {
     Serial1.println('2');
     b = b-1;
     } 
  } 
     else if(mode == 4)
  {
    //pwm低速 ：75->30cm/s */
    k_TrackB_1=1.1,k_TrackS_1=0.8;                //低速状态：2 0.6
    RequiredSpeed=140;                       //pwm低速 ：30cm/s
    k_TrackB_2=1.4,k_TrackS_2=0.6;              //大转弯参数   
    k_TrackB_3=1.3,k_TrackS_3=0.4;          
    if(d == 1)
    {
      Serial1.println('1');
      d = d - 1;
    }
  }
   else if(mode == 3)
  { 
     k_TrackB_1=1,k_TrackS_1=0.9;             //第三题超车
     k_TrackB_2=1,k_TrackS_2=0.4;            
     k_TrackB_3=1,k_TrackS_3=0.2;     
     RequiredSpeed=255;
     
     if(c == 1)
     {
     Serial1.println('3');
     c = c-1;
     }     
 }
}

 
void Sensor_Init(){
   pinMode(sensor1,INPUT);
   pinMode(sensor2,INPUT);
   pinMode(sensor3,INPUT);   
   pinMode(sensor4,INPUT);
   pinMode(sensor5,INPUT);
   delay(15);
}
 
void Motor_Init(){
  //左电机
  pinMode(MotorLpin1,OUTPUT);  //驱动芯片控制引脚
  pinMode(MotorLpin2,OUTPUT);  //驱动芯片控制引脚  
  pinMode(MotorLpwm,OUTPUT);   //驱动芯片控制引脚，PWM调速
  
  //右电机
  pinMode(MotorRpin1,OUTPUT);  //驱动芯片控制引脚
  pinMode(MotorRpin2,OUTPUT);  //驱动芯片控制引脚  
  pinMode(MotorRpwm,OUTPUT);   //驱动芯片控制引脚，PWM调速
 
  //驱动芯片控制引脚全部拉低
  digitalWrite(MotorLpin1,HIGH); //左电机
  digitalWrite(MotorLpin2,LOW);
  digitalWrite(MotorLpwm,LOW);
  digitalWrite(MotorRpin1,HIGH); //右电机
  digitalWrite(MotorRpin2,LOW);
  digitalWrite(MotorRpwm,LOW);
}

void Stop()
{
    digitalWrite(MotorLpin1,LOW); //左电机
    digitalWrite(MotorLpin2,LOW);
    digitalWrite(MotorRpin1,LOW); //右电机
    digitalWrite(MotorRpin2,LOW);
    //声音提示
   digitalWrite(Buzz,HIGH);
   delay(500);
   digitalWrite(Buzz,LOW);
   delay(1000);  
   
}

 
void Track()
{
   Mode_Control();
   read_sensor_values();
   switch(error)
   {
    case  4:
    if(nowtime-recenttime>1000)
    {
    digitalWrite(Buzz,HIGH);
    digitalWrite(Buzz,LOW);
    interrupt --;
    if(interrupt == 0)
    {
       Serial1.print('0');
       Stop();
    }
     else if(interrupt == 2 && mode == 3 && sent1 == 1) 
    {
      Serial1.println('6');
      sent1--;
    }
    else if(interrupt == 1 && mode == 3 && sent2 == 1) 
    {
      Set_Pwm(RequiredSpeed,RequiredSpeed);
      delay(500);
      Set_Pwm(k_TrackB_3*RequiredSpeed,k_TrackS_3*RequiredSpeed);
      delay(500);
      sent2--;
    }
    }
    recenttime = nowtime; 
    break;
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

void read_sensor_values()
{
  int sensor[5];
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);
  sensor[4] = digitalRead(sensor5);
  if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) 
  {
      error = 4;//          0 1 1 1 0 STOP
      
        
      
      nowtime = millis();
    }
  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) 
  {
      error = 2;//          0 0 0 0 1
    }
    else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) 
  {
      error = 0;
      if(dengting==0)
      {
        Serial1.println('0');
        Stop();
        delay(3500);
        dengting++;
        Serial1.println('1');
      }
      Motor_Init();
      
    }
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0))
    {
      error = 1;//          0 0 0 1 0
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      error = 0;//          0 0 1 0 0
    } 
    else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0) && (key == 0))
    {
      if(mode == 3)
      {
      error = 0;//          1 0 1 0 0 外圈
      }
      else
      {
         Set_Pwm(k_TrackS_3*RequiredSpeed,k_TrackB_3*RequiredSpeed);
        delay(10);//          0 1 1 0 0 内圈 
      }
      key = 1;
    } 
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)&&(key == 0))
    {
      if(mode == 3)
      {
        error = 0;// 0 1 1 0 0 外圈
      }
      else
      {
         Set_Pwm(k_TrackS_3*RequiredSpeed,k_TrackB_3*RequiredSpeed);
        delay(10);//          0 1 1 0 0 内圈
      }
         key = 1;
    }
    else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0) && (key == 0))
    {
      if(mode == 3)
      {
      error = 0;//        1 1 1 0 0 外圈
      }
      else
      {
        Set_Pwm(k_TrackS_3*RequiredSpeed,k_TrackB_3*RequiredSpeed);
        delay(10);//          0 1 1 0 0 内圈
      }
      key = 1;
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

void Set_Pwm(int speed_L,int speed_R)
{
  analogWrite(MotorLpwm,speed_L);
  analogWrite(MotorRpwm,speed_R);
}
