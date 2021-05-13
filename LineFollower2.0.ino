//libraries------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <SparkFun_APDS9960.h>

//functions
void update_Proxy();
void musika();
void update_Color();
void set_speed(int left, int right);
void set_motors();
void update_PID();
void set_direction(char dir);
void getError();
void getLinePositionNum();
// void printMotors();
// void printErrorVal();
// void printIRDigital();
// void print_Colors();
// void print_Proxy();

const int wait_period = 25;//non blocking delay for proximity sensor because it reads too much too fast
unsigned long time_now = 0;
unsigned long time_hold=0;

// Global Variables____________________________________________________________________________________________________________________________
SparkFun_APDS9960 apds = SparkFun_APDS9960();
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
int LEFT_MOTOR_SPEED=0;
int RIGHT_MOTOR_SPEED=0;

//initial Motor Speed----------------------------------------------------------------------------------------------
const byte INITIAL_MOTOR_SPEED=85;
int olderror=0;

//IR SENSORS--------------------------------------------------------------------------------------------------------
const byte IR_Sensor_Num=7;
const byte IR_Sensor_Pin[IR_Sensor_Num]={2,4,7,8,12,13,A1};

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
//PID CONSTANTS-------------------------------------------------------------------------------------------------------
const byte Kp=5;
const byte Ki=0;
const byte Kd=1;

//END GLOBAL VAR__________________________________________________________________________________________________________________________________________________________

//print functions-------------------------------------------------------------------------------------------------------
  void print_Proxy()
  {
    Serial.print(F("Proximity: "));
    Serial.print(proximity_data);
    Serial.println();
  }

  void print_Colors()
  {
    // Serial.print("\tAmbient: ");
    //   Serial.print(ambient_light);
    Serial.print(F("Red: "));
    Serial.print(red_light);
    Serial.print(F(" Green: "));
    Serial.print(green_light);
    Serial.print(F(" Blue: "));
    Serial.println(blue_light);
  }

  void printIRDigital()
  {
    Serial.print(F("Digital Reading: "));
    Serial.print(Line_Position);
    Serial.print("\t");
  }

  void printErrorVal()
  {
    Serial.print(F("Error Reading: "));
    Serial.println(error);
  }

  void printMotors()
  {
    Serial.print(F("PID Value:  "));
    Serial.print(PID);
    Serial.print(F("   \tLeft:  "));
    Serial.print(LEFT_MOTOR_SPEED);
    Serial.print(F("\tRight:  "));
    Serial.print(RIGHT_MOTOR_SPEED);
    Serial.print("\t");

  }

//end print function-------------------------------------------------------------------------------------------------------

//APDS9960----------------------------------------------------------------------------------------------------------------

void update_Color()
{
  // Read the light levels (ambient, red, green, blue)
  if (!apds.readAmbientLight(ambient_light) || !apds.readRedLight(red_light) || !apds.readGreenLight(green_light) || !apds.readBlueLight(blue_light))
  {    Serial.println(F("Error reading light values"));  }
  else
  {
    musika();
  }
}
//set music-------------------------------------------------------------------------------------------------------------
void musika()
{
  if(!PLAYED_MUSIC)
  {
    if (red_light > 50 && red_light > blue_light && red_light > green_light)
    {
      for(int i=0;i<10;i++){
        tone(buzzer, 1000);
        delay(100);
        noTone(buzzer);
      }
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

//check proxy&play-----------------------------------------------------------------------------------------------------------------
void update_Proxy() {
  if (!apds.readProximity(proximity_data)) {
    //    Serial.println("Error reading proximity value");
  } else {
    if (proximity_data >= 20) {
      set_speed(45, 45);  //slow down car
      time_hold = millis();
      while (proximity_data > 10)  //if you want to change distance 20 is start to slow down 230 is where to stop engines
      {
        apds.readProximity(proximity_data);
        //print_Proxy();
        while (millis() < time_hold + wait_period) {}  //CHECK IF NEED TO MAKE THIS HIGHER!
        if (proximity_data >= 200) {
          set_speed(0, 0);
          break;
        }
      }

    }  //stop car
    time_hold = millis();
    PLAYED_MUSIC = false;

    while (proximity_data >= 20) {

      if (millis() > time_hold + wait_period + 70) {
        time_hold = millis();
        apds.readProximity(proximity_data);
        if (proximity_data >= 50) {
          // Serial.print(time_hold);
          if (!PLAYED_MUSIC) {

            update_Color();

            //print_Colors();
            //print_P}roxy();
          }
        }
      }
    }
  }
//END APDS9960--------------------------------------------------------------------------------------------------------------------

//MOTOR CONTROL--------------------------------------------------------------------------------------------------------------------
//check line position and set an error value---------------------------------------------------------------------------------------
void getError(){
    switch (Line_Position)    {
    case 51111110:    {        error = -20;        break;    }
    case 51111100:    {        error = -20;        break;    }
    case 51111000:    {        error = -20;        break;    }
    case 51000000:    {        error = -10;         break;    }
    case 51100000:    {        error = -8;         break;    }
    case 51110000:    {        error = -7;         break;    }
    case 50110000:    {        error = -6;         break;    }
    case 50111000:    {        error = -5;         break;    }
    case 50011000:    {        error = -3;         break;    }
    case 50011100:    {        error = 0;          break;    }
    case 50001100:    {        error = 3;          break;    }
    case 50001110:    {        error = 5;          break;    }
    case 50000110:    {        error = 6;          break;    }
    case 50000111:    {        error = 7;          break;    }
    case 50000011:    {        error = 8;          break;    }
    case 50000001:    {        error = 10;          break;    }
    case 50001111:    {        error = 20;         break;    }
    case 50011111:    {        error = 20;         break;    } 
    case 50111111:    {        error = 20;         break;    }
    case 50000000:    {                            break;    }//do what you did last
    case 51111111:    {        error = 777;        break;    }//go straight
    default:    { /*Serial.println("Unkown Error - Line Following Status");*/ break;    }
    }
}

//set direction if wanting to turn ----------------------------------------------------------------------------------------------------
void set_direction(char dir){
   switch(dir) {
    case 'F':  {  digitalWrite(LEFT_FORWARD, HIGH);  digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, LOW);     break;  }
    case 'B':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK, HIGH);    digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, HIGH);    break;  }
    case 'L':  {  digitalWrite(LEFT_FORWARD, HIGH);  digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, LOW);     break;  }
    case 'R':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, LOW);     break;  }
    case 'S':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, LOW);     break;  }
    case '<':  {  digitalWrite(LEFT_FORWARD, HIGH);  digitalWrite(LEFT_BACK, LOW);     digitalWrite(RIGHT_FORWARD, LOW);    digitalWrite(RIGHT_BACK, HIGH);    break;  }
    case '>':  {  digitalWrite(LEFT_FORWARD, LOW);   digitalWrite(LEFT_BACK,HIGH);     digitalWrite(RIGHT_FORWARD, HIGH);   digitalWrite(RIGHT_BACK, LOW);     break;  }
   }
}

//Iterate over the IR sensors, and create a 6digit num (sensor)*10+new sensor - 000000 001100 010000
void getLinePositionNum()
{
  Line_Position = 5; //reset value
  for (byte i = 0; i < IR_Sensor_Num; ++i)
    Line_Position = (Line_Position * 10) + digitalRead(IR_Sensor_Pin[i]);
}

//CALCULATE PID VALUE BASED ON ERRORS ----------------------------------------------------------------------------------------------
// void update_PID(){
// if (error < 300)//as long as its not white\black line calc a new pid value
//     {
//         P = error;
//         I = I + prevI;
//         D = error - prevError;
//         PID = (Kp * P) + (Ki * I) + (Kd * D);
//         prevI = I;
//         prevError = error;
//     }
// }

//CONTROL MOTOR BASED ON PID----------------------------------------------------------------------------------------------------------
void set_motors()
{
   if (error == 777 || error == 20 || error ==-20) //all white - skip - do what you did last time until back on line
  {  
    return;
  }

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

//set motor analog speed--------------------------------------------------------------------------------------------------------------
void set_speed(int left, int right)
{
  analogWrite(LEFT_MOTOR, left + 5); //Left Motor Speed
  analogWrite(RIGHT_MOTOR, right);   //Right Motor Speed
}

bool sharpturn()
{
  if (error == -20 || error == 20)  //check again to make sure the error is indeed a 90 degree turn
  {
      time_hold = millis();
    while (millis() > time_hold +40) {
      getLinePositionNum();
      getError();
      if (error == 777) {
        set_speed(INITIAL_MOTOR_SPEED+5, INITIAL_MOTOR_SPEED); //Left Motor Speed
        set_direction('F');
       // Serial.print("black liNE");
        return false;
      }  //if full black line is present -> exit function
    }
      getLinePositionNum();
      getError();

   if (error == -20) //SHARP TURN LEFT
  {
    set_direction('<');
    set_speed(200, 75);
    delay(50);
    while (!(error >= -3 || error <=3)) //continue the turn until the robot is back on the line
    {
      getLinePositionNum();
      getError();
      delay(40);
    }
          //Serial.println("<<<<SHARP");

    return true;
  }

  if (error == 20) //SHARP TURN RIGHT
  {
    set_direction('>');
    set_speed(75, 200);
    delay(50);

    while (!(error >= -3 || error <=3)) //continue the turn until the robot is back on the line
    {
      getLinePositionNum();
      getError();
      delay(40);
    }
             // Serial.println(">>>SHARP");

    return true;
  }

  }
  return false;
}

//END MOTOR CONTROL--------------------------------------------------------------------------------------------------------------------


void setup() {
  //total initialization time is 2 seconds + 1.2 seconds for starting buzzer
  //Serial.begin(115200);
  //  while(!Serial); // wait for serial to be ready

  apds.init(); // Initialize APDS-9960 (configure I2C and initial values)
   delay(150);// Wait for initialization and calibration to finish
  apds.setProximityGain(PGAIN_2X);  // Adjust the Proximity sensor gain
   delay(150);    // Wait for initialization and calibration to finish
   apds.enableProximitySensor(false);// Start running the APDS-9960 proximity sensor (no interrupts)
   delay(150);    // Wait for initialization and calibration to finish
   apds.enableLightSensor(false);  // Start running the APDS-9960 light sensor (no interrupts)
   delay(150);    // Wait for initialization and calibration to finish

  //Set Pins
  pinMode(RIGHT_MOTOR,OUTPUT);
  pinMode(RIGHT_FORWARD,OUTPUT);
  pinMode(RIGHT_BACK,OUTPUT);
  pinMode(LEFT_MOTOR,OUTPUT);
  pinMode(LEFT_FORWARD,OUTPUT);
  pinMode(LEFT_BACK,OUTPUT);

  
  //play a small audio to let the user know when it will start to move
   tone(buzzer, 1000);
   delay(150);  //wait to allow manual alignment
   noTone(buzzer);
  // tone(buzzer, 2000);
  // delay(100);
  // noTone(buzzer);
  // tone(buzzer, 3000);
  // delay(100);
  // noTone(buzzer);

}

void loop()
{
//update proxy to be interuupt!!!
  if(millis()>=time_now + wait_period){
  update_Proxy(); //check proxy and if senses a block at 20 distance interrupt and stop the car and sense the color
  time_now += wait_period;
  
  //olderror=error;
    getLinePositionNum(); //get the line position from the IR sensors XXX-XXX
    getError();
             //get the amount of sway off track
  //if(error==olderror)
  //    return;
  
  if(!sharpturn()){
  set_motors(); //set speed
    }
  }

  //--debug--
  //printMotors();
  //printIRDigital();
  //printErrorVal();
  
//  update_PID(); //check PID
  //delay(300);
}
