//libraries------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <SparkFun_APDS9960.h>

//functions
void update_Proxy();
void update_Color();
void set_speed(int left, int right);
void set_motors();
void update_PID();
void set_direction(char dir);
void getError();
void getLinePositionNum();
void interruptRoutine();

const int wait_period = 25;//non blocking delay for proximity sensor because it reads too much too fast
unsigned long time_now = 0;
unsigned long time_hold=0;

// Global Variables____________________________________________________________________________________________________________________________
SparkFun_APDS9960 apds = SparkFun_APDS9960();
byte isr_flag = 0;
#define APDS9960_INT    2  // Needs to be an interrupt pin
#define PROX_INT_HIGH   20 // Proximity level for interrupt
#define PROX_INT_LOW    0  // No far interrupt

uint8_t proximity_data = 0;
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

//BUZZER AUDIO - COLOR SENSOR
bool PLAYED_MUSIC=false;
byte buzzer = A0;

//motor pins------------------------------------------------------------------------------------------------------
const byte LEFT_MOTOR = 11; //ena//left motor - set speed
const byte LEFT_FORWARD = 10;//in1
const byte LEFT_BACK = 9;//in2
const byte RIGHT_FORWARD = 6;//in3
const byte RIGHT_BACK = 5;//in4
const byte RIGHT_MOTOR = 3;//enb//right motor - set speed


//motor speed------------------------------------------------------------------------------------------------------
float LEFT_MOTOR_SPEED=0;
float RIGHT_MOTOR_SPEED=0;

//initial Motor Speed----------------------------------------------------------------------------------------------
const byte INITIAL_MOTOR_SPEED=85;
float olderror=0;

//IR SENSORS--------------------------------------------------------------------------------------------------------
const byte IR_Sensor_Num=7;
//const int IR_Sensor_Pin[IR_Sensor_Num]={A2,4,7,8,12,13,A1};
//Iterate over the IR sensors, and create a 6digit num (sensor)*10+new sensor - 000000 001100 010000



//Line Position-----------------------------------------------------------------------------------------------------
long Line_Position=5; //5 is just so i can have 0 as msb

//PID----------------------------------------------------------------------------------------------------------------
float PID=0;
float error=0;
float P=0;
int I=0;
float D=0;
float prevError=0;
float prevI=0;
//PID CONSTANTS-------------------------------------------------------------------------------------------------------
const byte Kp=20;
const byte Ki=0;
const byte Kd=7;

//END GLOBAL VAR__________________________________________________________________________________________________________________________________________________________
void getLinePositionNum()
{
  Line_Position = 5; //reset value
 // for (byte i = 0; i < IR_Sensor_Num; ++i)
 //   Line_Position = (Line_Position * 10) + digitalRead(IR_Sensor_Pin[i]);
 Line_Position = (Line_Position * 10) + digitalRead(A2);
 Line_Position = (Line_Position * 10) + digitalRead(4);
 Line_Position = (Line_Position * 10) + digitalRead(7);
 Line_Position = (Line_Position * 10) + digitalRead(8);
 Line_Position = (Line_Position * 10) + digitalRead(12);
 Line_Position = (Line_Position * 10) + digitalRead(13);
 Line_Position = (Line_Position * 10) + digitalRead(A1);
}
//APDS9960----------------------------------------------------------------------------------------------------------------

void update_Color()
{
  // Read the light levels (ambient, red, green, blue)
  if (!apds.readAmbientLight(ambient_light) || !apds.readRedLight(red_light) || !apds.readGreenLight(green_light) || !apds.readBlueLight(blue_light))
  {   // Serial.println(F("Error reading light values"));  
  return;
  }
  else
  {
     if(!PLAYED_MUSIC)
  {
    if (red_light > 50 && red_light > blue_light && red_light > green_light)
    {
      
        tone(buzzer, 500);
        delay(1000);
        noTone(buzzer);
        PLAYED_MUSIC = true;
        return;
    }
    if (blue_light > 50 && blue_light > red_light && blue_light > green_light)
    {
      for(int i=0;i<5;i++){
        tone(buzzer, 2000);
        delay(300);
        noTone(buzzer);
      }
      PLAYED_MUSIC = true;
      return;
    }

    if (green_light > 50 && green_light > red_light && green_light > blue_light)
    {
      for(int i=0;i<3;i++){
        tone(buzzer, 500);
        delay(500);
        noTone(buzzer);
      }
        PLAYED_MUSIC = true;
        return;
    }
  }
  }
}

//check proxy&play-----------------------------------------------------------------------------------------------------------------
void interruptRoutine() {
  isr_flag = 1;
}
  //END APDS9960--------------------------------------------------------------------------------------------------------------------

//MOTOR CONTROL--------------------------------------------------------------------------------------------------------------------
 


  //check line position and set an error value---------------------------------------------------------------------------------------
  void getError() {
    switch (Line_Position)    {
    case 51111110:    {        error = -21;        break;    }
    case 51111100:    {        error = -21;        break;    }
    case 51111000:    {        error = -21;        break;    }
    case 51000000:    {        error = -3.5;         break;    }
    case 51100000:    {        error = -3;         break;    }
    case 51110000:    {        error = -2;         break;    }
    case 50110000:    {        error = -3;         break;    }
    case 50111000:    {        error = -1;         break;    }
    case 50011000:    {        error = -0.75;         break;    }
    case 50011100:    {        error = 0;          break;    }
    case 50001100:    {        error = 0.75;          break;    }
    case 50001110:    {        error = 1;          break;    }
    case 50000110:    {        error = 3;          break;    }
    case 50000111:    {        error = 2;          break;    }
    case 50000011:    {        error = 3;          break;    }
    case 50000001:    {        error = 3.5;          break;    }
    case 50001111:    {        error = 21;         break;    }
    case 50011111:    {        error = 21;         break;    } 
    case 50111111:    {        error = 21;         break;    }
    case 50000000:    {                            break;    }//do what you did last
    case 51111111:    {        error = 777;        break;    }//go straight
    default:    { /*Serial.println("Unkown Error - Line Following Status");*/ break;    }
    }
}

//set motor analog speed--------------------------------------------------------------------------------------------------------------
void set_speed(int left, int right)
{
  if(Line_Position== 50110000 ||Line_Position== 51110000 ||Line_Position== 51100000||Line_Position== 51000000 ||Line_Position== 50000001 ||Line_Position== 50000011 ||Line_Position== 50000111||Line_Position== 50000110) 
  {
  LEFT_MOTOR_SPEED = constrain(INITIAL_MOTOR_SPEED - PID, 0, 90);
  RIGHT_MOTOR_SPEED = constrain(INITIAL_MOTOR_SPEED + PID, 0, 90);
  analogWrite(LEFT_MOTOR_SPEED, left + 5); //Left Motor Speed - maybe add a bit more 
  analogWrite(RIGHT_MOTOR_SPEED, right);   //Right Motor Speed
  return;
  }
  
  analogWrite(LEFT_MOTOR, left + 5); //Left Motor Speed - maybe add a bit more 
  analogWrite(RIGHT_MOTOR, right);   //Right Motor Speed
}
//set direction if wanting to turn ----------------------------------------------------------------------------------------------------
void set_direction(char dir){
   switch(dir) {
    case 'F':  {  digitalWrite(LEFT_FORWARD, HIGH);  digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, LOW);     break;  }
    case 'B':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK, HIGH);    digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, HIGH);    break;  }
    case 'R':  {  digitalWrite(LEFT_FORWARD, HIGH);  digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, LOW);     break;  }
    case 'L':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, LOW);     break;  }
    case 'S':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, LOW);     break;  }
    case '<':  {  digitalWrite(LEFT_FORWARD, HIGH);  digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, HIGH);    break;  }
    case '>':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK,HIGH);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, LOW);     break;  }
   }
}


//CALCULATE PID VALUE BASED ON ERRORS ----------------------------------------------------------------------------------------------
//CONTROL MOTOR BASED ON PID----------------------------------------------------------------------------------------------------------
void set_motors()
{
   if (error == 777 || error == 20 || error ==-20) //all white - skip - do what you did last time until back on line
  {    return;  }

        P = error;
        I = I + prevI;
        D = error - prevError;
        PID = (Kp * P) + (Ki * I) + (Kd * D);
        prevI = I;
        prevError = error;


  // The motor speed should not exceed the max PWM value
  LEFT_MOTOR_SPEED = constrain(INITIAL_MOTOR_SPEED - PID, 0, 255);
  RIGHT_MOTOR_SPEED = constrain(INITIAL_MOTOR_SPEED + PID, 0, 255);

  set_speed(LEFT_MOTOR_SPEED, RIGHT_MOTOR_SPEED);
  set_direction('F');
}




//END MOTOR CONTROL--------------------------------------------------------------------------------------------------------------------
bool interupted=false;
bool flagturn=false;

void setup() {
  //total initialization time is 0.8s seconds + 1.2 seconds for starting buzzer
Serial.begin(9600);
  //Set Pins
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
  attachInterrupt(digitalPinToInterrupt(2), interruptRoutine, FALLING);

  apds.init();   delay(200);
  apds.setProximityGain(PGAIN_2X);    delay(200);    
  apds.enableProximitySensor(true);   delay(200);    
  apds.enableLightSensor(false);      delay(200);
  apds.setProximityIntLowThreshold(PROX_INT_LOW);  
  apds.setProximityIntHighThreshold(PROX_INT_HIGH); 

  //play a small audio to let the user know when it will start to move
   tone(buzzer, 1000);   delay(150);   noTone(buzzer);
  // tone(buzzer, 2000);   delay(100);   noTone(buzzer);
  // tone(buzzer, 3000);   delay(100);   noTone(buzzer);

}

void loop() {
  //getLinePositionNum();
  // If interrupt occurs, print out the proximity level
  if (isr_flag == 1) {
    interupted = true;
    apds.readProximity(proximity_data);

    //error=0;
    update_Color();
    isr_flag = 0;
    apds.clearProximityInt();
    PLAYED_MUSIC = false;
  }

  if (interupted == false)
 {
    getLinePositionNum();
    getError();
    Serial.print(F("Digital Reading: "));
    Serial.println(Line_Position);
 
    delay(500);
    if (error >= 20 || error <= -20)     { 
      delay(30);
      getLinePositionNum();  //get the line position from the IR sensors XXX-XXX
      getError();
      if (error > 15)  //sharp turn right
      {        set_direction('>');        set_speed(150, 100);        delayMicroseconds(50000);        return;      }

      if (error < -15)  //sharp turn left
      {        set_direction('<');        set_speed(100, 150);        delayMicroseconds(50000);        return;      } 
      }  else {
     set_motors();

    //delayMicroseconds(1000);
    delay(10);  
    return; 
    }

 } else {
    interupted = false;
    set_direction('S');
    set_speed(0, 0);
    delay(100);
  }
}
