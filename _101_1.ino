/*****************************************
motor one :GPIO_D4 正反转控制端 
          :PWM_D5  PWM调速
          :GPIO_D2 返回编码器参数
motor two :GPIO_D7 正反转控制端 
          :PWM_D6   PWM调速       
          :GPIO_D3 返回编码器参数
*******************************************/
#include "MsTimer2.h"              //定时器库的 头文件
#include <PID_v1.h>
/******************************************************/
int INA = 4; //电机A正反转控制端
int PWMA = 5; //电机A调速端
int PWMB = 6; //电机B调速端
int INB = 7; //电机B正反转控制端
int pAin=0,pBin=1; //外部中断 GPIO_D2    GPIO_D3
/****************************************************************************/
//Define Variables we'll be connecting to
double Setpoint_r, Input_r, Output_r,Setpoint_l, Input_l, Output_l;

double output_r_Setpoint_r,output_l_Setpoint_l;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID_r(&Input_r, &Output_r, &Setpoint_r, consKp, consKi, consKd, DIRECT);
PID myPID_l(&Input_l, &Output_l, &Setpoint_l, consKp, consKi, consKd, DIRECT);
/*********************************************************************************/

static volatile long int  Encoder_data_r,Encoder_data_l;
static volatile long int r_Speed,l_Speed;

void setup()
{
    Serial.begin(9600);
     Serial.println("Speed_L_R:");
     MsTimer2::set(5000, Speed_L_R);
     MsTimer2::start();
     myPID_r.SetMode(AUTOMATIC);
     myPID_l.SetMode(AUTOMATIC);

}
void loop()
{     
      Setpoint_r = 0;
      Setpoint_l = 0;
      output_r_Setpoint_r = Setpoint_r;
      output_l_Setpoint_l = Setpoint_l;
      Input_l = l_Speed;  //25
      Input_r = r_Speed; 
      
     if(Setpoint_r<0)
          {
           Setpoint_r=abs(Setpoint_r);
          } 
                double gap_r =Setpoint_r-Input_r; //abs(Setpoint_r-Input_r); //distance away from setpoint
                     if (gap_r < 10){
                              myPID_r.SetTunings(consKp, consKi, consKd);//we're close to setpoint, use conservative tuning parameters
                            }
                          else {
                                     myPID_r.SetTunings(aggKp, aggKi, aggKd); //we're far from setpoint, use aggressive tuning parameters
                                } 
     if(Setpoint_l<0)
          {
              Setpoint_l=abs(Setpoint_l);
          }          
                double gap_l = Setpoint_l-Input_l; //abs(Setpoint_l-Input_l); //distance away from setpoint
                 if (gap_l < 10)
                        {
                          myPID_l.SetTunings(consKp, consKi, consKd);//we're close to setpoint, use conservative tuning parameters
                        }
                      else 
                            {
                                myPID_l.SetTunings(aggKp, aggKi, aggKd); //we're far from setpoint, use aggressive tuning parameters
                            } 
                            
                              
            myPID_r.Compute(); //开始计算pid 参数
            myPID_l.Compute();
      _direction(output_r_Setpoint_r,output_l_Setpoint_l);         
     motosp(Output_r,Output_l); 
     
     attachInterrupt(pAin,a_state_temp,RISING );//外部中断0
     attachInterrupt(pBin,b_state_temp,RISING );//外部中断1     
}
void a_state_temp()
      {Encoder_data_r++;}
void b_state_temp()
      {Encoder_data_l++;}
      
void Speed_L_R()
{  
      r_Speed=Encoder_data_l*10/20/89;
      l_Speed=Encoder_data_r*10/20/89;

      Serial.print("Encoder_data_l=");Serial.println(Encoder_data_l);
      Serial.print("Encoder_data_r=");Serial.println(Encoder_data_r);

      Serial.print("r_Speed_R_R=");Serial.print(r_Speed);Serial.println("s/mm");
      Serial.print("l_Speed_L_L=");Serial.print(l_Speed);Serial.println("s/mm");
    
     // Serial.print("Output_r=");Serial.println(Output_r);
     // Serial.print("Output_l=");Serial.println(Output_l);
      Encoder_data_l=0;  
      Encoder_data_r=0; 
}   

void motosp(int sp1,int sp2)//电机速度控制函数。括号内分别为左右电机速度值，
{
  analogWrite(PWMA,abs (sp1)); 
  analogWrite(PWMB,abs (sp2));
}

void _direction(int y,int x)
{
  if(x>0)
       { digitalWrite(INA, HIGH);}    
     else{
              digitalWrite(INA, LOW); }
  if(y>0){
         digitalWrite(INB, HIGH);}
      else{
             digitalWrite(INB, LOW);}
}
/*
void motosp(int sp1,int sp2)//电机速度控制函数。括号内分别为左右电机速度值，
{
  if(sp1>0)                 //范围-255～+255，正值为正转，负值为反转。
        {
            digitalWrite(INA, HIGH);    
        }
  else
  {
   digitalWrite(INA, LOW); 
  }
  if(sp2>0)
  {
   digitalWrite(INB, HIGH);                                                                                                                                                                                                                                                                                               
  }
  else
  {
   digitalWrite(INB, LOW); 
  }
  analogWrite(PWMA,abs (sp1)); 
  analogWrite(PWMB,abs (sp2));
}*/
