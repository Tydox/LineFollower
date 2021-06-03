#include <Wire.h>
#include <SparkFun_APDS9960.h>
#define SL -70
#define SR 70

void get_color();
void set_speed(int left, int right);
void set_motors();
void set_direction(char dir);
void get_line_pos();
void interrupt_routine();

SparkFun_APDS9960 apds = SparkFun_APDS9960();
byte isr_flag = 0;
#define APDS9960_INT    2  
#define PROX_INT_HIGH   15 //was 30
#define PROX_INT_LOW    0  

uint8_t proximity_data = 0;
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

bool PLAYED_MUSIC=false;
byte buzzer = A0;
bool interupted=false;
bool flagturn=false;

const byte LEFT_MOTOR = 3;
const byte LEFT_FORWARD = 6;
const byte LEFT_BACK = 5;
const byte RIGHT_FORWARD = 10;
const byte RIGHT_BACK = 9;
const byte RIGHT_MOTOR = 11;


int LEFT_MOTOR_SPEED=0;
int RIGHT_MOTOR_SPEED=0;

const byte INITIAL_MOTOR_SPEED=105;
int olderror=0;

const byte IR_Sensor_Num=7;

long Line_Position=5;

int PID=0;
int error=0;
int P=0;
int I=0;
int D=0;
int prevError=0;
int prevI=0;

const byte Kp=5;
const byte Ki=0;
const byte Kd=2;

void get_line_pos()
{
 Line_Position = 5;
 Line_Position = (Line_Position * 10) + digitalRead(A2);
 Line_Position = (Line_Position * 10) + digitalRead(4);
 Line_Position = (Line_Position * 10) + digitalRead(7);
 Line_Position = (Line_Position * 10) + digitalRead(8);
 Line_Position = (Line_Position * 10) + digitalRead(12);
 Line_Position = (Line_Position * 10) + digitalRead(13);
 Line_Position = (Line_Position * 10) + digitalRead(A1);

  switch (Line_Position)    {
    case 51111110:    {        error = SL;         break;    }
    case 51111100:    {        error = SL;         break;    }
    case 51111000:    {        error = SL;         break;    }
    case 51000000:    {        error = -30;        break;    }
    case 51100000:    {        error = -28;        break;    }
    case 51110000:    {        error = -20;        break;    }
    case 50110000:    {        error = -10;        break;    }
    case 50111000:    {        error = -4;         break;    }
    case 50011000:    {        error = -3;         break;    }
    case 50011100:    {        error = 0;          break;    }
    case 50001100:    {        error = 4;          break;    }
    case 50001110:    {        error = 5;          break;    }
    case 50000110:    {        error = 10;         break;    }
    case 50000111:    {        error = 20;         break;    }
    case 50000011:    {        error = 28;         break;    }
    case 50000001:    {        error = 30;         break;    }
    case 50001111:    {        error = SR;         break;    }
    case 50011111:    {        error = SR;         break;    } 
    case 50111111:    {        error = SR;         break;    }
    case 50000000:    {                            break;    }
    case 51111111:    {        error = 777;        break;    }
    default:          {                            break;    }
    }
}

void get_color() {
  if (!apds.readRedLight(red_light) || !apds.readGreenLight(green_light) || !apds.readBlueLight(blue_light)) {
    return;
  } else {
    if (!PLAYED_MUSIC) {
      if (red_light > 30 && red_light > blue_light && red_light > green_light) {

        tone(buzzer, 3000);
        delay(100);
        tone(buzzer, 1500);
        delay(100);
        tone(buzzer, 600);
        delay(100);
        noTone(buzzer);
        PLAYED_MUSIC = true;
      }
      if (blue_light > 30 && blue_light > red_light && blue_light > green_light) {
        tone(buzzer, 3000);
        delay(150);
        tone(buzzer, 5000);
        delay(150);
        tone(buzzer, 7000);
        delay(150);
        noTone(buzzer);
        PLAYED_MUSIC = true;
        return;
      }

      if (green_light > 70 && green_light > red_light && green_light > blue_light) {

        tone(buzzer, 3500);
        delay(100);
        noTone(buzzer);
        PLAYED_MUSIC = true;
        return;
      }
    }
  }
}

void interrupt_routine() {
  isr_flag = 1;
}

void set_direction(char dir){
   switch(dir) {
    case 'F':  {  digitalWrite(LEFT_FORWARD, HIGH);  digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, LOW);     break;  }
    case 'B':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK, HIGH);    digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, HIGH);    break;  }
    case 'S':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, LOW);     break;  }
    case '>':  {  digitalWrite(LEFT_FORWARD, HIGH);  digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, HIGH);    break;  }
    case '<':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK,HIGH);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, LOW);     break;  }
    case 'R':  {  digitalWrite(LEFT_FORWARD, HIGH);  digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, LOW);    break;  }
    case 'L':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK,LOW);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, LOW);     break;  }
    case 'C':  {  digitalWrite(LEFT_FORWARD, HIGH);   digitalWrite(LEFT_BACK,HIGH);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, HIGH);     break;  }
    
   }
}

void set_motors()
{
    
  if (error == SL || error == SR)     
    { 
      delay(50);
      get_line_pos();      
      if (error == SR)       {        set_direction('>');        set_speed(120, 120);        delay(100);        return;      }

      if (error == SL)       {        set_direction('<');        set_speed(120,120);         delay(100);        return;      } 
      
    }
    if(error == 777 )     {       set_direction('F');       set_speed(INITIAL_MOTOR_SPEED+5, INITIAL_MOTOR_SPEED);       return;       } 

if(error==-30)
  {
  P = error + 15;
  I = I + prevI;
  D = error - prevError;
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  prevI = I;
  prevError = error;

  set_direction('<');//maybe change this to value 60 to make the other motor just resist movements
    set_speed(INITIAL_MOTOR_SPEED - PID, INITIAL_MOTOR_SPEED - PID);

  return;
  }

  if(error==30)
  {
    P = error - 15;
  I = I + prevI;
  D = error - prevError;
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  prevI = I;
  prevError = error;

    set_direction('>');
    set_speed(INITIAL_MOTOR_SPEED + PID,INITIAL_MOTOR_SPEED + PID);

  return;
  }


  P = error;
  I = I + prevI;
  D = error - prevError;
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  prevI = I;
  prevError = error;


  set_direction('F');
  set_speed(INITIAL_MOTOR_SPEED + PID, INITIAL_MOTOR_SPEED - PID);

}

void set_speed(int left, int right)
{
if(error==0)
{
  analogWrite(LEFT_MOTOR, 110); 
  analogWrite(RIGHT_MOTOR, 115);
  return;
}


  LEFT_MOTOR_SPEED = constrain(left-5 , 0, 255);
  RIGHT_MOTOR_SPEED = constrain(right+1, 0, 255);
  analogWrite(LEFT_MOTOR, LEFT_MOTOR_SPEED); 
  analogWrite(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
}


void setup() {
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(4,INPUT);
  pinMode(7,INPUT);
  pinMode(8,INPUT);
  pinMode(12,INPUT);
  pinMode(13,INPUT);
  pinMode(RIGHT_MOTOR,OUTPUT);
  pinMode(RIGHT_FORWARD,OUTPUT);
  pinMode(RIGHT_BACK,OUTPUT);
  pinMode(LEFT_MOTOR,OUTPUT);
  pinMode(LEFT_FORWARD,OUTPUT);
  pinMode(LEFT_BACK,OUTPUT);
  pinMode(APDS9960_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), interrupt_routine, FALLING);

  apds.init();   delay(200);
  apds.setProximityGain(PGAIN_2X);    delay(200);    
  apds.enableProximitySensor(true);   delay(200);    
  apds.enableLightSensor(false);      delay(200);
  apds.setProximityIntLowThreshold(PROX_INT_LOW);  delay(50);
  apds.setProximityIntHighThreshold(PROX_INT_HIGH); delay(100);

   tone(buzzer, 1000);   delay(300);   noTone(buzzer);
   tone(buzzer, 2000);   delay(100);   noTone(buzzer);
   tone(buzzer, 3000);   delay(50);    noTone(buzzer);

}

void loop() {

  if (isr_flag == 1) {
    set_direction('S');
    set_speed(0, 0);
    interupted = true;
    apds.readProximity(proximity_data);
    get_color();
    isr_flag = 0;
    apds.clearProximityInt();
    PLAYED_MUSIC = false;
  }

  if (interupted == false)
 {
  get_line_pos();
  set_motors();
  delay(5);
  return; 

 } else {
    interupted = false;
    set_direction('S');
    set_speed(0, 0);
    delay(200); //500 works
    return;
  }
}
