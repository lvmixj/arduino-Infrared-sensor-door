/*
   主要功能:感应开门,经过5s关门
   进阶:无人5s内关门,关门时期有人继续开门
   优化功能:意外中断的处理
   time函数,millis(),使用差值
   外部中断,与millis,delay冲突,不用
   连线:https://mc.dfrobot.com.cn/thread-297263-1-1.html
   automatic infrared door
*/

#define DIR 6 //方向信号
#define PUL 5 //脉冲信号
#define ENA 7 //使能信号,可不接
#define RED 3 //红外热释电信号,黄色接地,蓝色正极
#define LED 13

int doorTime = 2700;//完全的开关门时间
int gapTime = 5000;//延迟时间
int door; //不完全关or正在开1，完全关or正在关0
int redState ;//热释电状态

unsigned long openBeginMoment; //开始开门的时刻
unsigned long openEndMoment; //正常开门完成时刻
unsigned long closeBeginMoment; //开始关门的时刻
unsigned long closeEndMoment; //正常关门完成时刻
unsigned long attachMoment; //中断时刻

void setup()
{
  pinMode(RED, INPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  digitalWrite(ENA, LOW); //使能
  //digitalWrite(DIR, HIGH);//设置步进电机转动方向
  door = 0;
}

void loop()
{
  //Serial.println(digitalRead(DIR));
  redState = digitalRead(RED);
  //Serial.println(redState);
  if (redState == 1 && door == 0)
  {
    openDoor(doorTime);
  }
  else if (door == 1 && redState == 0)
  {
    closeDoor(doorTime);
  }
}

//创建开门函数，变量为开门时间，单位ms
void openDoor(int openTime)
{
  Serial.println("开门");
  door = 1;
  stateLED();
  openBeginMoment = millis();
  openEndMoment = openBeginMoment + (unsigned long)openTime;
  digitalWrite(DIR, HIGH);//正方向
  analogWrite(PUL, 100);
  while (true)
  {
    if (openEndMoment < millis() )
    {
      analogWrite(PUL, 0);
      break;
    }
  }
  delay(gapTime);
}

//创建关门函数,变量为关门时间,单位毫秒
void closeDoor(int closeTime)
{
  Serial.println("关门");
  door = 0;
  stateLED();
  closeBeginMoment = millis();
  closeEndMoment = closeBeginMoment + (unsigned long)closeTime;
  digitalWrite(DIR, LOW);//反方向
  //Serial.println(digitalRead(DIR));
  analogWrite(PUL, 100);
  while (true)
  {
    //   attachInterrupt(digitalPinToInterrupt(RED),closeANDcross,HIGH);
    if (digitalRead(RED) == 1)
    {
      closeANDcross();
      break;
    }
    else if (closeEndMoment < millis() )
    {
      analogWrite(PUL, 0);
      break;
    }
  }
  //Serial.println("关门结束");
}

//通过LED来显示开关门状态
void stateLED()
{
  if (door == 1 ) digitalWrite(LED, HIGH);
  else digitalWrite(LED, LOW);
}

//关门时红外感应高电平的处理函数
void closeANDcross()
{
  attachMoment = millis();
  Serial.println(attachMoment - closeBeginMoment);
  openDoor(int(attachMoment - closeBeginMoment));
}
