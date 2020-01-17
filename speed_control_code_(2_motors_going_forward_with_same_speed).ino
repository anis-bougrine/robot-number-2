//PID Regulator library and atomic block macro
#include <PID_v1.h>  
#include <util/atomic.h>
//Define Arduino Mega Pin
#define encoder_A_Right 18  //encoder input of the right motor
#define encoder_A_Left 2    //encoder input of the left motor
#define motor_left_up 8
#define motor_left_down 9
#define motor_right_up 4
#define motor_right_down 5
#define pwm_Left 11
#define pwm_Right 13
//Define Variables
double Kp_Right=3.5, Ki_Right=9,Kd_Right=0.5;
double Kp_Left=4.5, Ki_Left=10.9 ,Kd_Left=0.6;
double Setpoint, Input_Right, Output_Right, Input_Left, Output_Left;
volatile int count_Right=0; //the compiler load the variable from RAM and not from a storage register,A variable should be declared volatile whenever its value.. 
volatile int count_Left=0;  //..can be changed by something beyond the control of the code section (because under certain conditions, the value for a variable stored in..                         
int previoustime=0;         //..registers can be inaccurate) such as interrupt service routine
int speed_Right=0; 
int speed_Left =0; 
PID PID_Right(&Input_Right, &Output_Right, &Setpoint, Kp_Right, Ki_Right, Kd_Right, DIRECT);
PID PID_Left(&Input_Left, &Output_Left, &Setpoint, Kp_Left, Ki_Left, Kd_Left, DIRECT);
//Define configurations and initialisations
void setup()
{
pinMode(encoder_A_Right, INPUT_PULLUP);
pinMode(encoder_A_Left, INPUT_PULLUP);
pinMode(motor_left_up, OUTPUT);
pinMode(motor_left_down, OUTPUT);
pinMode(motor_right_up, OUTPUT);
pinMode(motor_right_down, OUTPUT);
pinMode(pwm_Right, OUTPUT);
pinMode(pwm_Left, OUTPUT);
analogWrite (pwm_Right,0);
analogWrite (pwm_Left,0);
attachInterrupt(digitalPinToInterrupt(encoder_A_Right),counter_Right, RISING);
attachInterrupt(digitalPinToInterrupt(encoder_A_Left),counter_Left, FALLING);
Input_Right = 0;
Input_Left = 0;
Setpoint = 20;                   //speed required (20 ticks every 10ms)
PID_Right.SetMode(AUTOMATIC);
PID_Left.SetMode(AUTOMATIC);
PID_Right.SetOutputLimits(-255, 255);
PID_Left.SetOutputLimits(-255, 255);
}
//The loop mode
void loop(){
//sampling (Te = 10ms)
if(millis()-previoustime>10)
{
    previoustime=millis();
    /*If the volatile variable is bigger than a byte  then the microcontroller can not read it in one step, because it is an 8 bit microcontroller
    This means that while your main code section (e.g. your loop) reads the first 8 bits of the variable, the interrupt might already change the second 8 bits
    so we use the ATOMIC_BLOCK macro to disable interrupts while next block executing*/
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
          {
          //saving the number of ticks of the last period (last 10ms)
          speed_Right=count_Right;
          speed_Left=count_Left;
          //initialisation
          count_Right=0;
          count_Left=0;
          }//end of atomic block
    //It's time for PIDs regulators calculations
    Input_Right=speed_Right;
    Input_Left=speed_Left;
    PID_Right.Compute();
    PID_Left.Compute();
    int output_Right=(int)Output_Right;
    int output_Left=(int)Output_Left;
    //assignment the PIDs outputs to the MOTORS inputs
    analogWrite (pwm_Right,output_Right); 
    digitalWrite(motor_right_up, HIGH);
    digitalWrite(motor_right_down, LOW);
    analogWrite (pwm_Left,output_Left); 
    digitalWrite(motor_left_up, HIGH);
    digitalWrite(motor_left_down, LOW);
            
}
}
//A function to execute every interrupt of encoder_Right
void counter_Right()
{
  count_Right++;
}
//A function to execute every interrupt of encoder_Left
void counter_Left()
{
  count_Left++;
}

