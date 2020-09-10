/***************************************************************************************
 *  Title: Snake Robot source code
 *  Author(s): Sumajit, J., & Le, C. 
 *  Date: 3/26/2019
 *  Code Version: 4.5
 *  Company: Ryerson University, Mechanical Engineering, Tetra Engineering Consultants
***************************************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  800   // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2200  // this is the 'maximum' pulse length count (out of 4096)
#define FREQUENCY 50    // nominal rated frequency

int myServos[8];

float pi=3.14159;
int TotalNumberofServos=8; 
int ServosPerSide=TotalNumberofServos/2;
float shift = 2*pi/ServosPerSide; // Phase lag between segments
int Amplitude;
int offset;
float rads;
float Speed=1;
float Wavelengths=1.5;
int Multiplier;

int InteriorAngle, SetpointAngle, MaxAngleDisplacement;

void setup() {
  Serial.begin(9600); 
  //Initializing Vertical Servos:    0, 2', 4, 6'
  //Initializing Horizontal Servos:  1, 3', 5, 7'
  myServos[0] = 0;
  myServos[1] = 1;
  myServos[2] = 2;
  myServos[3] = 3;
  myServos[4] = 4;
  myServos[5] = 5;
  myServos[6] = 6;
  myServos[7] = 7;

  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
}
 
int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  if(angle < 0 ){angle = 0;}
  if(angle >165){angle = 165;}
  pulse_wide   = map(angle, 0, 165, SERVOMIN, SERVOMAX);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}




/*********************************************************************************************************************
*    Title: 1D Robotic Snake source code
*    Author: Donaldson, W.
*    Date: 2018
*    Availability: https://github.com/WillDonaldson/Robotic-Snake/blob/master/_1D_robotic_snake/_1D_robotic_snake.ino
*    Revised By: Sumajit, J.
*********************************************************************************************************************/
//Only Horizontal Servos will be active

void straightline(){
  for(int i=0; i<8; i++){  
    pwm.setPWM(myServos[i], 0, pulseWidth(90));
    delay(100); 
  }
}

void Cshape(){
  for(int i=0; i<4; i++){
    if(i%2!=0){//passes odd numbers only
      pwm.setPWM(myServos[2*i-1], 0, pulseWidth(60));//Actvie Servos: 1, 5
      pwm.setPWM(myServos[2*i+1], 0, pulseWidth(120));//Actve Servos: 3', 7'
    }
    else{
      pwm.setPWM(myServos[2*i], 0, pulseWidth(90));//Active Servos: 0, 4
      pwm.setPWM(myServos[2*i+2], 0, pulseWidth(90));//Active Servos: 2', 6'                               
    }
    delay(100); 
  }
}

void ring(){
  InteriorAngle=180-360/(TotalNumberofServos+1); //general formula for a polygon with 3 or more vertices, +1 vertice between tail and head segment
  for(int i=0; i<4; i++){
    if(i%2!=0){
      pwm.setPWM(myServos[2*i-1], 0, pulseWidth(abs(InteriorAngle-90)));//Active Servos: 1, 5
      pwm.setPWM(myServos[2*i+1], 0, pulseWidth(abs(InteriorAngle+90)));//ACtive Servos: 3', 7' 
    }  
    else{
      pwm.setPWM(myServos[2*i], 0, pulseWidth(90));//Active Servos: 0, 4
      pwm.setPWM(myServos[2*i+2], 0, pulseWidth(90));//Active Servos: 2', 6'                          
    }
    delay(100); 
  }
}

//**InchWorm
//**HARDCODED
void InchWorm(){
  for(int pos = 0; pos < 45; pos +=  1){
    pwm.setPWM(myServos[0], 0, pulseWidth(90-pos));
    pwm.setPWM(myServos[2], 0, pulseWidth(90-2*pos));
    delay(10);
  }    
  for(int pos = 0; pos < 45; pos +=  1){
    pwm.setPWM(myServos[2], 0, pulseWidth(45+pos));
    pwm.setPWM(myServos[4], 0, pulseWidth(180-3*pos));  
    pwm.setPWM(myServos[6], 0, pulseWidth(45+3*pos));
    delay(10);   
  }
  for(int pos = 0; pos < 45; pos +=  1){
    pwm.setPWM(myServos[4], 0, pulseWidth(45+pos));
    pwm.setPWM(myServos[6], 0, pulseWidth(180-2*pos));  
    delay(10);
  }  
}

void slither(String Direction, int Amplitude){
  if(Direction=="Forward"){offset=0;}
  else if(Direction=="Left"){offset=10;}
  else if(Direction=="Right"){offset=-10;}
  MaxAngleDisplacement=abs(offset)+abs(Amplitude);//amount servo can rotate from the SetpointAngle without going out of the [0,180] degree range
  while(MaxAngleDisplacement>90){//prevents a setpoint angle outside the range of[0,180]
    Amplitude=abs(Amplitude)-1;
    MaxAngleDisplacement=abs(offset)+Amplitude;
  }
  for(int i=0; i<360; i++){
   rads=i*pi/180.0;//convert from degrees to radians
   for(int j=1; j<4; j++){  
    if(j%2!=0){
      pwm.setPWM(myServos[2*j-1], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift)));//Active Servos: 1, 5 
      pwm.setPWM(myServos[2*j+1], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift)));//Active Servos: 3', 7' 
    }
   }
   delay(10);
  }
}



/*****************************************************************************************************************************
*    Title: 2D Snake Sidewinding source code
*    Author: Donaldson, W.
*    Date: 2018
*    Availability: https://github.com/WillDonaldson/Robotic-Snake/blob/master/_2D_snake_sidewinding/_2D_snake_sidewinding.ino
*    Revised By: Sumajit, J.
*****************************************************************************************************************************/

void sidewind(String Direction, int Amplitude) {
  if(Direction=="Left"){
    Serial.println("Running: sidewind(Left)"); 
    Multiplier=1; 
    }
  else if(Direction=="Right"){
    Serial.println("Running: sidewind(Right)"); 
    Multiplier=-1; 
    }
  for(int i=0; i<360; i++){
   rads=i*pi/180.0;//convert from degrees to radians
   for(int j=0; j<4; j++){
    if(j%2!=0){
    pwm.setPWM(myServos[2*j], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift-(Multiplier-1)*pi/4)));//Vertical Servos Active: 2', 6'
    pwm.setPWM(myServos[2*j+1], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift+(Multiplier+1)*pi/4)));//Horiztonal Servos Active: 3', 7'
    }
    else{//passes eve value of j       0,2,4,6 
    pwm.setPWM(myServos[2*j], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift-(Multiplier-1)*pi/4)));//Vertical Servos Active: 0, 4
    pwm.setPWM(myServos[2*j+1], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift+(Multiplier+1)*pi/4)));//Horizontal Servos Active: 1, 5
    }
   }
   delay(10);
  }
}

void sidewindTurn(String Direction, int Amplitude){
  offset = 0;
  if(Direction=="Left"){ Multiplier=1; }
  else if(Direction=="Right"){ Multiplier=-1; }
  for(int i=0; i<360; i++){
   rads=i*pi/180.0;//convert from degrees to radians
   for(int j=0; j<2; j++){
    if(j%2!=0){
      pwm.setPWM(myServos[2*j], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift+(Multiplier+1)*pi/4)));//Vertical Servos Active: 2'
      pwm.setPWM(myServos[2*j+1], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift-(Multiplier-1)*pi/4)));//Horizontal Servos Active: 3'
    }
    else{
      pwm.setPWM(myServos[2*j], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift+(Multiplier+1)*pi/4)));//Vertical Servos Active: 0
      pwm.setPWM(myServos[2*j+1], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift-(Multiplier-1)*pi/4)));//Horiztonal Servos Active: 1
    }
   }
   for(int j=2; j<4; j++){
    if(j%2!=0){
      pwm.setPWM(myServos[2*j], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift-(Multiplier-1)*pi/4)));//Vertical Servos Active: 6'
      pwm.setPWM(myServos[2*j+1], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift+(Multiplier+1)*pi/4)));//Horiztonal Servis Active: 7'
    }
    else{
      pwm.setPWM(myServos[2*j], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift-(Multiplier-1)*pi/4)));//Vertical Servos Active: 4
      pwm.setPWM(myServos[2*j+1], 0, pulseWidth(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift+(Multiplier+1)*pi/4)));//Horizontal Servos Active: 5
    }
   }
   delay(10);
  }
}

void loop() {
  //***1D Functions
  //slither("Forward",35);
  //slither("Left",35);
  //slither("Right",35);
  //InchWorm();
  //Cshape();
  //ring();
  //straightline();  
  
  //***2D Functions
  sidewind("Left",50);  
  //sidewind("Right",50);
  //sidewindTurn("Left",50);
  //sidewindTurn("Right",50);

  //***TEST
  //pwm.setPWM(myServos[0],0,pulseWidth(90));
}
