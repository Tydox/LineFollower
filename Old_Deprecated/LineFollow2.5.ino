//libraries------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <SparkFun_APDS9960.h>
#define SL -70
#define SR 70
//functions
void get_color();
void set_speed(int left, int right);
void set_motors();
void set_direction(char dir);
void get_line_pos();
void interrupt_routine();

// Global Variables____________________________________________________________________________________________________________________________
SparkFun_APDS9960 apds = SparkFun_APDS9960();
byte isr_flag = 0;
#define APDS9960_INT    2  // Needs to be an interrupt pin
#define PROX_INT_HIGH   30 // Proximity level for interrupt
#define PROX_INT_LOW    0  // No far interrupt

uint8_t proximity_data = 0;
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

//BUZZER AUDIO - COLOR SENSOR
bool PLAYED_MUSIC=false;
byte buzzer = A0;
bool interupted=false;
bool flagturn=false;

//motor pins------------------------------------------------------------------------------------------------------
const byte LEFT_MOTOR = 11; //ena//left motor - set speed
const byte LEFT_FORWARD = 10;//in1
const byte LEFT_BACK = 9;//in2
const byte RIGHT_FORWARD = 6;//in3
const byte RIGHT_BACK = 5;//in4
const byte RIGHT_MOTOR = 3;//enb//right motor - set speed


//motor speed------------------------------------------------------------------------------------------------------
int LEFT_MOTOR_SPEED=0;
int RIGHT_MOTOR_SPEED=0;

//initial Motor Speed----------------------------------------------------------------------------------------------
const byte INITIAL_MOTOR_SPEED=90;
int olderror=0;

//IR SENSORS--------------------------------------------------------------------------------------------------------
const byte IR_Sensor_Num=7;

//Line Position-----------------------------------------------------------------------------------------------------
long Line_Position=5; //5 is just so i can have 0 as msb

//PID----------------------------------------------------------------------------------------------------------------
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

//END GLOBAL VAR__________________________________________________________________________________________________________________________________________________________
void get_line_pos()
{
 Line_Position = 5; //reset value
 Line_Position = (Line_Position * 10) + digitalRead(A2);
 Line_Position = (Line_Position * 10) + digitalRead(4);
 Line_Position = (Line_Position * 10) + digitalRead(7);
 Line_Position = (Line_Position * 10) + digitalRead(8);
 Line_Position = (Line_Position * 10) + digitalRead(12);
 Line_Position = (Line_Position * 10) + digitalRead(13);
 Line_Position = (Line_Position * 10) + digitalRead(A1);

  switch (Line_Position)    {
    case 51111110:    {        error = SL;        break;    }
    case 51111100:    {        error = SL;        break;    }
    case 51111000:    {        error = SL;        break;    }
    case 51000000:    {        error = -35;         break;    }
    case 51100000:    {        error = -30;         break;    }
    case 51110000:    {        error = -30;         break;    }
    case 50110000:    {        error = -22;         break;    }
    case 50111000:    {        error = -3;         break;    }
    case 50011000:    {        error = -2;         break;    }
    case 50011100:    {        error = 0;          break;    }
    case 50001100:    {        error = 2;          break;    }
    case 50001110:    {        error = 3;          break;    }
    case 50000110:    {        error = 22;          break;    }
    case 50000111:    {        error = 30;          break;    }
    case 50000011:    {        error = 30;          break;    }
    case 50000001:    {        error = 35;          break;    }
    case 50001111:    {        error = SR;         break;    }
    case 50011111:    {        error = SR;         break;    } 
    case 50111111:    {        error = SR;         break;    }
    case 50000000:    {                            break;    }//do what you did last
    case 51111111:    {        error = 777;        break;    }//go straight
    default:          {                            break;    }
    }
}
//APDS9960----------------------------------------------------------------------------------------------------------------

void get_color() {
  // Read the light levels (ambient, red, green, blue)
  if (!apds.readRedLight(red_light) || !apds.readGreenLight(green_light) || !apds.readBlueLight(blue_light)) {
    return;
  } else {
    if (!PLAYED_MUSIC) {
      if (red_light > 50 && red_light > blue_light && red_light > green_light) {

        tone(buzzer, 500);
        delay(500);
        noTone(buzzer);
        PLAYED_MUSIC = true;
        return;
      }
      if (blue_light > 50 && blue_light > red_light && blue_light > green_light) {
        tone(buzzer, 2000);
        delay(200);
        noTone(buzzer);
        delay(50);
        tone(buzzer, 2000);
        delay(200);
        noTone(buzzer);
        PLAYED_MUSIC = true;
        return;
      }

      if (green_light > 50 && green_light > red_light && green_light > blue_light) {

        tone(buzzer, 3000);
        delay(500);
        noTone(buzzer);
        tone(buzzer, 100);
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
  //END APDS9960--------------------------------------------------------------------------------------------------------------------

//MOTOR CONTROL--------------------------------------------------------------------------------------------------------------------


//set direction if wanting to turn ----------------------------------------------------------------------------------------------------
void set_direction(char dir){
   switch(dir) {
    case 'F':  {  digitalWrite(LEFT_FORWARD, HIGH);  digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, LOW);     break;  }
    case 'B':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK, HIGH);    digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, HIGH);    break;  }
    case 'S':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, LOW);     break;  }
    case '>':  {  digitalWrite(LEFT_FORWARD, HIGH);  digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, HIGH);    break;  }
    case '<':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK,HIGH);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, LOW);     break;  }
    case 'C':  {  digitalWrite(LEFT_FORWARD, HIGH);   digitalWrite(LEFT_BACK,HIGH);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, HIGH);     break;  }

   }
}


//CALCULATE PID VALUE BASED ON ERRORS ----------------------------------------------------------------------------------------------
//CONTROL MOTOR BASED ON PID----------------------------------------------------------------------------------------------------------
void set_motors()
{
  
 // if (error == 777 || error == SL || error == SR)  {  return;  }
  
  if (error == SL || error == SR)     
    { 
      delay(50);
      get_line_pos();  
      
      
      //delay(500);
      //Serial.print(Line_Position);
      //Serial.println(error);
    
    
      if (error == SR)  //sharp turn right - add back track
      {        set_direction('<');        set_speed(150, 130);        delay(100);        return;      }

      if (error == SL)  //sharp turn left
      {        set_direction('>');        set_speed(130,150);        delay(100);        return;      } 
      

    }
    if(error == 777 )
     {
       set_direction('F');
       set_speed(INITIAL_MOTOR_SPEED, INITIAL_MOTOR_SPEED);
       return;
       } 

  P = error;
  I = I + prevI;
  D = error - prevError;
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  prevI = I;
  prevError = error;

  LEFT_MOTOR_SPEED = constrain(INITIAL_MOTOR_SPEED - PID+5, 0, 255);
  RIGHT_MOTOR_SPEED = constrain(INITIAL_MOTOR_SPEED + PID, 0, 255);
  set_direction('F');
  set_speed(LEFT_MOTOR_SPEED, RIGHT_MOTOR_SPEED);
  /*  Serial.print("left motor");
       Serial.print(LEFT_MOTOR_SPEED);
          Serial.print("right motor");

       Serial.print(RIGHT_MOTOR_SPEED);
          Serial.print("line");

       Serial.print(Line_Position);
          Serial.print("error");

       Serial.println(error);
       delay(500);*/
}

//set motor analog speed--------------------------------------------------------------------------------------------------------------
void set_speed(int left, int right)
{
  analogWrite(LEFT_MOTOR, left); //Left Motor Speed
  analogWrite(RIGHT_MOTOR, right);   //Right Motor Speed
}


//END MOTOR CONTROL--------------------------------------------------------------------------------------------------------------------

void setup() {
     //Serial.begin(9600);
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

   tone(buzzer, 500);   delay(100);   noTone(buzzer);
  // tone(buzzer, 2000);   delay(100);   noTone(buzzer);
  // tone(buzzer, 3000);   delay(100);   noTone(buzzer);

}

void loop() {
  if (isr_flag == 1) {
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
    delay(100);
    return;
  }
}
