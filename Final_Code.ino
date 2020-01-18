//library
#include <Servo.h>
#include <PID_v1.h>
#include <util/atomic.h>
//Define the Arduino Mega pins
#define Encoder_A_Right A1  //the input of the right motor encoder
#define Encoder_A_Left A2   //the input of the left motor encoder
#define Sensor_Test_Right 53   //sensor used to capt the right black bar for the robot stops 
#define Sensor_Test_Left 3     //sensor used to capt the Left black bar for the robot stops 
#define sensor_left A9         //one of 3 sensors used for line follower
#define sensor_middle A8       //one of 3 sensors used for line follower
#define sensor_right A10       //one of 3 sensors used for line follower
#define threshold 500          //the threshold used to digitized the analog value of sensors
//Define variabled
Servo Push; 
long ultrasound_duration=0,ultrasound_Distance=0,Last_sampling=0,Last_Robot_Stop=0; 
int Servo_Position=0;
int ultrasound_trigPin=10,ultrasound_echoPin=8;
int motor_right_up=15;
int motor_right_down=16;
int motor_left_up=4;
int motor_left_down=5;
int pwm_Left=11,pwm_Right=13;
int speed_Motor_Right=0,speed_Motor_Left=0;
int Nb_of_Stops=0,TotalNb_Ticks_Left=0,TotalNb_Ticks_Right=0;
double Setpoint=20,Input_Right=0,Output_Right=0,Input_Left=0,Output_Left=0;         //variabled for PID regulator and setpoint= how many ticks per sampling period we want
double Kp_Right=3.5,Ki_Right=9,Kd_Right=0.6,Kp_Left=4.5,Ki_Left=10.9,Kd_Left=0.5;   //variabled for PID regulator
PID PID_Right(&Input_Right,&Output_Right,&Setpoint,Kp_Right,Ki_Right,Kd_Right,DIRECT);
PID PID_Left(&Input_Left,&Output_Left,&Setpoint,Kp_Left,Ki_Left,Kd_Left,DIRECT);
volatile int Current_NbTicks_Right=0,Current_NbTicks_Left=0;  

//define functions
int Read_One_Sensor(int pin)
{
  if(analogRead(pin)>threshold)
        return(1); 
  else 
        return(0);
}
int Read_All_Sensors()
{
  return (Read_One_Sensor(sensor_left)*100+Read_One_Sensor(sensor_middle)*10+Read_One_Sensor(sensor_right));
}
void Stop_Motors () 
{
  analogWrite(pwm_Left,0);
  digitalWrite(motor_left_up,LOW);
  digitalWrite(motor_left_down,LOW);
  analogWrite(pwm_Right,0);
  digitalWrite(motor_right_up,LOW);
  digitalWrite(motor_right_down,LOW);
}
void Turn_Left () 
{   
  analogWrite (pwm_Right,200); 
  digitalWrite(motor_right_up, HIGH);
  digitalWrite(motor_right_down, LOW);
  analogWrite (pwm_Left,150); 
  digitalWrite(motor_left_up, LOW);
  digitalWrite(motor_left_down, HIGH);
}
void Turn_Right () 
{ 
  analogWrite (pwm_Left,200); 
  digitalWrite(motor_left_up, HIGH);
  digitalWrite(motor_left_down, LOW);
  analogWrite (pwm_Right,150); 
  digitalWrite(motor_right_up, LOW);
  digitalWrite(motor_right_down, HIGH);
}
void Counter_Encoder_Right()
{
  Current_NbTicks_Right++;
}
void Counter_Encoder_Left()
{
  Current_NbTicks_Left++;
}

void Going_Forward ()     //the robot going forward with a controlled speed (the two motors have the same speed ((setpoint=20ticks) per (sampling_period=10ms))), this function is executed when the robot is on a going forward condition
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    Current_NbTicks_Right=0;
    Current_NbTicks_Left=0;
  }
  while(Read_All_Sensors()==10)
  { 
     Test_Robot_Stops();
     if(millis()-Last_sampling>10)
     {
         Last_sampling=millis();
         ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
         speed_Motor_Right=Current_NbTicks_Right;
         speed_Motor_Left=Current_NbTicks_Left;
         Current_NbTicks_Right=0;
         Current_NbTicks_Left=0;}
         Input_Right=speed_Motor_Right;
         Input_Left=speed_Motor_Left;
         PID_Right.Compute();
         PID_Left.Compute();
         int Output_Right=(int)Output_Right;
         int Output_Left=(int)Output_Left;
         analogWrite (pwm_Right,Output_Right); 
         digitalWrite(motor_right_up, HIGH);
         digitalWrite(motor_right_down, LOW);
         analogWrite (pwm_Left,Output_Left); 
         digitalWrite(motor_left_up, HIGH);
         digitalWrite(motor_left_down, LOW);
     }
  }
}//end od Going_Forward function

void Automatic_Part()   //this function used for the automatic part of the robot where the robot it's not on line follower mode (distance control based on number of ticks of the motors)
{
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    Current_NbTicks_Right=0;
    Current_NbTicks_Left=0;
  }
  
  while((TotalNb_Ticks_Right<1560)&&(TotalNb_Ticks_Left<1560))
  {
     if(millis()-Last_sampling>10)
     {
        Last_sampling=millis();
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
          speed_Motor_Right=Current_NbTicks_Right;
          speed_Motor_Left=Current_NbTicks_Left;
          Current_NbTicks_Right=0;
          Current_NbTicks_Left=0;
        }
        Input_Right=speed_Motor_Right;
        Input_Left=speed_Motor_Left;
        PID_Right.Compute();
        PID_Left.Compute();
        int Output_Right=(int)Output_Right;
        int Output_Left=(int)Output_Left;
        analogWrite (pwm_Left,Output_Left); 
        digitalWrite(motor_left_up, HIGH);
        digitalWrite(motor_left_down, LOW);
        analogWrite (pwm_Right,Output_Right); 
        digitalWrite(motor_right_up, HIGH);
        digitalWrite(motor_right_down, LOW);
        TotalNb_Ticks_Right+=speed_Motor_Right;
        TotalNb_Ticks_Left+=speed_Motor_Left;
     }
  }
  
  Stop_Motors();
  delay(1000);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    Current_NbTicks_Left =0;
  }
  TotalNb_Ticks_Left=0;
  
  while(TotalNb_Ticks_Left<2000)
  {
     if(millis()-Last_sampling>10)
     {
        Last_sampling=millis();
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
           speed_Motor_Left=Current_NbTicks_Left;
           Current_NbTicks_Left=0;
        }
        Input_Left=speed_Motor_Left;
        PID_Left.Compute();
        int Output_Left=(int)Output_Left;
        analogWrite (pwm_Left,Output_Left); 
        digitalWrite(motor_left_up, LOW);
        digitalWrite(motor_left_down, HIGH);
        TotalNb_Ticks_Left+=speed_Motor_Left;
     }
  }
  
  Stop_Motors();
  delay(1000);
  TotalNb_Ticks_Right=0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    Current_NbTicks_Right =0;
  }
  
  while(TotalNb_Ticks_Right<1870)
  {
     if(millis()-Last_sampling>10)
     {
        Last_sampling=millis();
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
          speed_Motor_Right=Current_NbTicks_Right;
          Current_NbTicks_Right=0;
        }
        Input_Right=speed_Motor_Right;
        PID_Right.Compute();
        int Output_Right=(int)Output_Right;
        analogWrite (pwm_Right,Output_Right); 
        digitalWrite(motor_right_up, HIGH);
        digitalWrite(motor_right_down, LOW);
        TotalNb_Ticks_Right+=speed_Motor_Right;
     }
  }
  
  Stop_Motors();
  
}//end of the automatic function

void Test_Robot_Stops()     //function to test whether the robot conditions are stop conditions or not
{
if(Nb_of_Stops==0)
{
    digitalWrite(ultrasound_trigPin,LOW);
    delayMicroseconds(5);
    digitalWrite(ultrasound_trigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasound_trigPin,LOW);
    ultrasound_duration=pulseIn(ultrasound_echoPin,HIGH);
    ultrasound_Distance=(ultrasound_duration/2)*0.0343;
    if((ultrasound_Distance<=15)&&(ultrasound_Distance>11))
    {
            Stop_Motors();
            for ( Servo_Position = 150; Servo_Position >= 50; Servo_Position -= 1) { 
            Push.write(Servo_Position);              
            delay(15);                      
            }
            Nb_of_Stops++;
            delay(5000);   
    }
      
}
if(Read_One_Sensor(Sensor_Test_Left)&&((millis()-Last_Robot_Stop)>4000)&&Read_One_Sensor(Sensor_Test_Right))  //millis used in this condition to avoid that the robot stay stopped in the same black bars for ever
{
    Stop_Motors();
    if(Nb_of_Stops==1)
    {
        for(Servo_Position=50;Servo_Position<=150;Servo_Position+=1) 
        {Push.write(Servo_Position);              
         delay(15);}           
    }         
    if(Nb_of_Stops==2)
    {
        for(Servo_Position=150;Servo_Position>=50;Servo_Position-=1)
        {Push.write(Servo_Position);              
         delay(15);}
    }
    if(Nb_of_Stops==3)
    { 
        for(Servo_Position=50;Servo_Position<=150;Servo_Position+=1)
        {Push.write(Servo_Position);              
         delay(15);}
    }
    if(Nb_of_Stops==4)
    {
        Automatic_Part();
    }
    delay(5000);
    Nb_of_Stops++;   
    Last_Robot_Stop=millis();
}
}//end of Test_Robot_Stops function

//setup mode
void  setup(){
pinMode(ultrasound_trigPin,OUTPUT);
pinMode(ultrasound_echoPin,INPUT);
Push.attach(9);
Push.write(150);
pinMode(Encoder_A_Right, INPUT_PULLUP);
pinMode(Encoder_A_Left, INPUT_PULLUP);
pinMode(motor_left_up, OUTPUT);
pinMode(motor_left_down, OUTPUT);
pinMode(motor_right_up, OUTPUT);
pinMode(motor_right_down, OUTPUT);
pinMode(pwm_Right, OUTPUT);
pinMode(pwm_Left, OUTPUT);
pinMode(sensor_left,INPUT);
pinMode(sensor_middle,INPUT);
pinMode(sensor_right,INPUT);
pinMode(Sensor_Test_Left,INPUT);
pinMode(Sensor_Test_Right,INPUT);
attachInterrupt(digitalPinToInterrupt(Encoder_A_Right),Counter_Encoder_Right, RISING);
attachInterrupt(digitalPinToInterrupt(Encoder_A_Left),Counter_Encoder_Left, FALLING);
PID_Right.SetMode(AUTOMATIC);
PID_Left.SetMode(AUTOMATIC);
PID_Right.SetOutputLimits(-255, 255);
PID_Left.SetOutputLimits(-255, 255);
}

//loop mode
void loop() 
{
  Test_Robot_Stops(); //first of all we have to test if there is a condition of stop than we can go to the line follower code 
  
  switch(Read_All_Sensors())
  {
    case(010):
       {
          Going_Forward();
          break;
       }
    case(001):
       {
          Turn_Right();
          break;
       }
    case(011):
       {
          Turn_Right();
          break;
       }
    case(100):
       {
          Turn_Left();
          break;
       }
    case(110):
       {
          Turn_Left();
          break;
       }
  }  
}
