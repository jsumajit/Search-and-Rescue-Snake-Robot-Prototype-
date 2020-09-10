#include <Servo.h>
Servo myServos[8]; //create 8 servos

float pi=3.14159;
int TotalNumberofServos=8; //change as required 
int ServosPerSide=TotalNumberofServos/2;
float shift = 2*pi/ServosPerSide; // Phase lag between segments
int Amplitude;
int offset;
float rads;
float Speed=2;
float Wavelengths=1.5;
int Multiplier;

int InteriorAngle, SetpointAngle, MaxAngleDisplacement;

void setup() {
  //The following code can be cleaned up in a single for-loop if you solder the wires in a more continuous pattern than I did
  //Order of servos is such that myServos[0]=tail segment, and myServos[7]=head segment
  Serial.begin(9600);
  myServos[0].attach(A0);// V
  myServos[1].attach(A1);// H
  myServos[2].attach(A2);// V'
  myServos[3].attach(A3);// H'
  myServos[4].attach(A4);// V
  myServos[5].attach(A5);// H
  myServos[6].attach(4);//  V'
  myServos[7].attach(3);//  H'
  
  //Initialise snake in a straight line
  for(int i=0; i<10; i++){  
  myServos[i].write(90);
  delay(15);
  }
}

//***1D Robot Codes: Only Veritcal or Horinzontal moving
//**Vertical    0 2' 4 6'
//**Horizontal  1 3' 5 7'
void straightline(){
  for(int i=0; i<8; i++){  
    myServos[i].write(90);
    delay(100); 
  }
}

void Cshape(){
  for(int i=0; i<4; i++){
    if(i%2!=0){//                   1  3 
      myServos[2*i-1].write(60);//  1  5
      myServos[2*i+1].write(120);// 3' 7' 
    }
    //MAY BE REMOVED    
    else{//                       0   2                          
      myServos[2*i].write(90);//  0   4
      myServos[2*i+2].write(90);//2'  6'
    }
    delay(100); 
  }
}

void ring(){
  InteriorAngle=180-360/(TotalNumberofServos+1); //general formula for a polygon with 3 or more vertices, +1 vertice between tail and head segment
  for(int i=0; i<4; i++){
    if(i%2!=0){//                                     1  3
      myServos[2*i-1].write(abs(InteriorAngle-90));// 1  5
      myServos[2*i+1].write(abs(InteriorAngle+90));// 3' 7'
    }
    //MAY BE REMOVED   
    else{//                       0   2                          
      myServos[2*i].write(90);//  0   4
      myServos[2*i+2].write(90);//2'  6'
    }
    delay(100); 
  }
}

//**InchWorm needs to be teseted
void InchWorm(){
  for(int pos = 0; pos < 45; pos +=  1){
    myServos[1].write(90-pos);
    myServos[3].write(90-2*pos);
    delay(10);
  }    
  for(int pos = 0; pos < 45; pos +=  1){  
    myServos[1].write(45+pos);
    myServos[3].write(180-3*pos);
    myServos[5].write(45+3*pos);
    myServos[7].write(90-pos);
    delay(10);
  }    
  for(int pos = 0; pos < 45; pos +=  1){  
    myServos[5].write(45+pos);
    myServos[7].write(180-2*pos);
    delay(10);
  }  
}

void slither(String Direction, int Amplitude){
  if(Direction=="Forward"){offset=0;}
  else if(Direction=="Left"){offset=10;}
  else if(Direction=="Right"){offset=-10;}
  MaxAngleDisplacement=abs(offset)+abs(Amplitude); //amount servo can rotate from the SetpointAngle without going out of the [0,180] degree range
  while(MaxAngleDisplacement>90){ //prevents a setpoint angle outside the rage of[0,180]
    Amplitude=abs(Amplitude)-1;
    MaxAngleDisplacement=abs(offset)+Amplitude;
  }
  for(int i=0; i<360; i++){
   rads=i*pi/180.0;     //convert from degrees to radians
   for(int j=0; j<8; j++){  
    if(i%2!=0){//                                                                      1  3
      myServos[2*j-1].write(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift));//1  5
      myServos[2*j+1].write(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift));//3' 7'                                   3' 7'
    }
   }
   delay(10);
  }
}



//***2D Robot Codes: Vertical and Horizontal moving
//**Vertical    0 2' 4 6'
//**Horizontal  1 3' 5 7'
void sidewind(String Direction, int Amplitude){
  //works best with Wavelengths=1.5
  offset = 0; 
  if(Direction=="Left"){ Multiplier=1; }
  else if(Direction=="Right"){ Multiplier=-1; }
  for(int i=0; i<360; i++){
   rads=i*pi/180.0;     //convert from degrees to radians
      for(int j=0; j<4; j++){
        if(j%2!=0){//                                                                                                                       1   3 
          myServos[2*j].write(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift-(Multiplier-1)*pi/4));//moves servos in vert. plane'   2'  6'
          myServos[2*j+1].write(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift+(Multiplier+1)*pi/4));//moves servos in vert. plane' 3'  7'
        }
        else{//                                                                                                                                   0 2 
          myServos[2*j].write(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift-(Multiplier-1)*pi/4));   //moves servos in vertical plane    0 4 
          myServos[2*j+1].write(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift+(Multiplier+1)*pi/4)); //moves servos in horizontal plane  1 5
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
   rads=i*pi/180.0;     //convert from degrees to radians
   for(int j=0; j<2; j++){
    if(j%2!=0){//                                                                                                                       1                                                                                                 
      myServos[2*j].write(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift+(Multiplier+1)*pi/4));//moves servos in vert. plane'   2'
      myServos[2*j+1].write(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift-(Multiplier-1)*pi/4));//moves servos in vert. plane' 3'
    }
    else{//                                                                                                 0       
      myServos[2*j].write(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift+(Multiplier+1)*pi/4));//   0
      myServos[2*j+1].write(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift-(Multiplier-1)*pi/4));// 1
    }
   }
   for(int j=2; j<4; j++){
    if(j%2!=0){//                                                                                                                       3
      myServos[2*j].write(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift-(Multiplier-1)*pi/4));//moves servos in vert. plane'   6'
      myServos[2*j+1].write(90+offset+Amplitude*sin(Speed*rads-j*Wavelengths*shift+(Multiplier+1)*pi/4));//moves servos in vert. plane' 7'
    }
    else{//                                                                                                 2  
      myServos[2*j].write(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift-(Multiplier-1)*pi/4));//   4
      myServos[2*j+1].write(90+offset+Amplitude*sin(Speed*rads+j*Wavelengths*shift+(Multiplier+1)*pi/4));// 5
    }
   }
   delay(10);
  }
}

void loop() {
  //comment and uncomment for the movement you want
  //as mentioned in the instructable, this code is skeletal in that there is no remote control function, stay tuned for version 2

  //***1D Functions
  //slither("Forward",35);
  //slither("Left",35);
  //slither("Right",35);
  //InchWorm();
  //Cshape();
  //ring();
  //straightline();  

  //***2D Functions
  //sidewind("Left",50);  
  //sidewind("Right",50);
  //sidewindTurn("Left",50);
  //sidewindTurn("Right",50);
}
