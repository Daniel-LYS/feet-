//这个程序中动作组的号数跟视频中稍有不同，因为六足只能使用高姿态来进行避障，低姿态的时候前面两个腿会挡到超声波的测距
//所以这个程序里的各个动作组都是高姿态的动作，25号高姿态立正，26号高姿态前进，27号高姿态后退，28号高姿态左转，29号高姿态右转

#include <Servo.h>
#include <NewPing.h>
#include <LobotServoController.h>

#define TRIG   9          /*超声波TRIG引脚为 9号IO*/
#define ECHO   8          /*超声波ECHO引脚为 8号IO*/
#define MAX_DISTANCE 150  /*最大检测距离为150cm*/

//#define GO_FORWARD  26     /*直走的动作组*/
//#define GO_BACK     27     /*后退动作组*/
//#define TURN_LEFT   28     /*左转动作组*/
//#define TURN_RIGHT  29     /*右转动作组*/

#define MIN_DISTANCE_TURN 35  /*避障距离，就是小于多少距离的时候进行避障*/
#define BIAS 3        /*舵机偏差，根据实际情况调整大小以使超声波朝向正前方*/

LobotServoController Controller(Serial1);  //实例化舵机控制板二次开发类,使用1号串口作为通信接口
NewPing Sonar(TRIG, ECHO, MAX_DISTANCE);      //实例化超声波测距类
Servo sonarServo;     //超声波云台舵机控制类实例

int gDistance;    //全局变量，用于存储中间位置超声波测得的距离
int gLDistance;   //用于存储机器人左侧测得的距离
int gRDistance;   //用于存储机器人右侧测得的距离

bool ledON = true;            //led点亮标识，true时点亮，false熄灭
void ledFlash() {
  static uint32_t Timer;      //定义静态变量Timer， 用于计时
  if (Timer > millis())       //Timer 大于 millis（）（运行的总毫秒数）时返回，
    return;
  //Timer 小于 运行总毫秒数时继续
  if (ledON) {
    digitalWrite(13, HIGH);   //如果点亮标识true，13号IO置高电平,板上LED灯被点亮
    Timer = millis() + 20;    //Timer = 当前运行的总毫秒数 + 20  实现 20毫秒后再次运行
    ledON = false;            //置点亮标识为false
  } else {
    ledON = false;            //如果点亮标识不是true，置点亮标识为false
    digitalWrite(13, LOW);    //置13号IO为低电平，板上LED熄灭
  }
}

int getDistance() {       //获得距离
  uint16_t lEchoTime;     //变量 ，用于保存检测到的脉冲高电平时间
  lEchoTime = Sonar.ping_median(6);           //检测6次超声波，排除错误的结果
  int lDistance = Sonar.convert_cm(lEchoTime);//转换检测到的脉冲高电平时间为厘米
  return lDistance;                           //返回检测到的距离
}

void getAllDistance()//获得前及左右三个方向的距离
{
  int tDistance;     //用于暂存测得距离
  sonarServo.write(90 + BIAS);   //超声波云台舵机转到90度即中间位置
  delay(200);                    //等待200ms，等待舵机转动到位
  gDistance = getDistance();     //测量距离，保存到全局变脸gDistance
  
  sonarServo.write(130 + BIAS);  //超声波云台舵机转到130度位置即机器人左面40度位置
  delay(200);                    //延时，等待舵机转动到位
  tDistance = getDistance();     //测量距离，保存到 tDistance
  sonarServo.write(170 + BIAS);  //转动到170度，即机器人左侧80度位置
  delay(200);                    //延时，等待舵机转动到位
  gLDistance = getDistance();    //测量距离，保存到 gLDistance
  if(tDistance < gLDistance)     //比较左侧测得的两个距离，取小的一个，保存到gLDistance作为左侧距离
    gLDistance = tDistance;
  
  sonarServo.write(50 + BIAS);   //超声波云台舵机转到50度位置即机器人右面40度位置    
  delay(200);                    //延时，等待舵机转动到位
  tDistance = getDistance();     //测量距离，保存到tDistance
  sonarServo.write(10 + BIAS);   //转到10度，即机器人右面80度位置
  delay(200);                    //延时，等待舵机转动到位
  gRDistance = getDistance();    //测量距离，保存到gRDistance
  if(tDistance < gRDistance)     //比较两个距离，将较小的一个保存到gRDistance
    gRDistance = tDistance;      
    
  sonarServo.write(90 + BIAS);   //超声波云台舵机转回中间位置
}

void sonar()  //避障逻辑
{
  static uint32_t timer = 0;   //静态变量，用于计时
  static uint8_t step = 0;     //静态变量，用于记录步骤
  if (timer > millis())  //如果设定时间大于当前毫秒数则返回，否侧继续后续操作
    return;
  switch (step)  //根据step分支
  {
    case 0:  //步骤0
      gDistance = getDistance();   //测量距离，保存到gDistance
      if (gDistance > MIN_DISTANCE_TURN || gDistance == 0) {  //如果测得距离大于指定的避障距离，前进
        if (!Controller.isRunning) {
          //Controller.runActionGroup(GO_FORWARD, 0); //一直前进
          step = 1; //转移到步骤1
          timer = millis() + 500;  //延时500ms
        }
      } else {  //如果测得距离小于指定距离
        step = 2;  //转移到步骤2
        timer = millis() + 100; //延时100ms
      }
      break; //结束switch语句
    case 1:  //步骤1
      gDistance = getDistance(); //测量距离
      if (gDistance < MIN_DISTANCE_TURN && gDistance > 0) {  //如果测得距离小于指定的避障距离，则停止所有动作组，转移到步骤2
        Controller.stopActionGroup();
        step = 2;
      }
      break; //结束switch语句
    case 2:  //步骤2
      if (!Controller.isRunning) {   //没有动作组在运行，即等待动作组运行完毕
        getAllDistance();            //获得三个方向的距离
//        Serial.println(gDistance);  //打印测得距离
//        Serial.println(gLDistance);
//        Serial.println(gRDistance);
        step = 3; //转移到步骤3
        //此处没有break，执行完后直接之心case 3
      }else{
        gDistance = getDistance();   //如果正在运行的动作组没有运行完毕，那么获得中间方向距离，并延时500ms
        timer = millis() + 500;
        break;  //结束switch
      }
    case 3:  //步骤3
      static bool lastActionIsGoBack = false;   //静态变量，记录最后的动作是不是后退
      if (((gDistance > MIN_DISTANCE_TURN) || (gDistance == 0)) && lastActionIsGoBack == false) {
        //中间距离大于指定避障距离且最后的一个动作不是后退，那么就回到步骤0，
        //此处判断最后一个动作是不是后退，是避免程序陷入后退-》前进-》后退-》前进...这样的死循环
        //当最后一步是后退是就不执行前进
        step = 0;
        timer = millis() + 200;
        lastActionIsGoBack = false;
        return; //返回，结束函数
      }
      if ((((gLDistance > gRDistance) && (gLDistance > MIN_DISTANCE_TURN)) || gLDistance == 0) && gDistance > 15) {
      //超声波测得左侧的最小距离大于右侧的最小距离大于指定的避障距离，并且中间测得距离大于15时
      //检测中间的距离目的是避免有物体处于机器人两个前腿之间，导致机器人无法转向
        if (!Controller.isRunning) {   //等待动作组运行完毕
          //Controller.runActionGroup(TURN_LEFT, 4);  //左转4次，
          lastActionIsGoBack = false;  //标识最后一个动作不是后退
          step = 2;  //转移到步骤2
        }
        timer = millis() + 500; //延时500ms
        return; //返回，结束函数
      }
      if ((((gRDistance > gLDistance) && (gRDistance > MIN_DISTANCE_TURN)) || gRDistance == 0) && gDistance > 15) {
      //超声波测得左侧的最小距离大于右侧的最小距离大于指定的避障距离，并且中间测得距离大于15时
        if (!Controller.isRunning) {   //等待动作组运行完毕
          //Controller.runActionGroup(TURN_RIGHT, 4);  //右转4次
          lastActionIsGoBack = false;  //标识最后一个动作不是后退
          step = 2;  //转移到步骤2
        }
        timer = millis() + 500;   //延时500ms
        return;  //返回，结束函数
      }
      //当前面的都不符合，所有的return都没有被执行
      //程序就会执行到这里
     // Controller.runActionGroup(GO_BACK, 3);  //执行后退动作组3次
      lastActionIsGoBack = true;  //标识最后一个动作是后退
      step = 2;     //转移到步骤2
      timer = millis() + 500;     //延时500ms
  }
}

void setup() {
  // put your setup code here, to run once
  Serial.begin(9600);                        //初始化0号串口
  Serial1.begin(9600);                       //初始化1号串口
  pinMode(13, OUTPUT);                        //设置板上led 13号IO口为输出
  delay(500);                                 //延时500ms，等待舵机控制就绪
  sonarServo.attach(10);                      //绑定10号io为舵机控制iO
  sonarServo.write(90 + BIAS);                //转到中间位置，加上了偏差
  Controller.runActionGroup(25, 1);            //运行25号动作组，25号为高姿态立正，回初始位置
  delay(2000);                                //延时两秒，简单延时，确保0号动作组运行完毕
  Controller.stopActionGroup();               //停止动作运行，确保停止
}
void loop() {
  // put your main code here, to run repeatedly:
  Controller.receiveHandle(); //接收处理函数，从串口接收缓存中取出数据
  sonar();                    //避障逻辑实现
  ledFlash();                 //led闪灯，用于运行状态提示
}
