//libraries------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <SparkFun_APDS9960.h>
//end libraries-------------------------------------------------------------------------------------------------

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
const int INITIAL_MOTOR_SPEED=150;


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
const byte Kd=2;

//END GLOBAL VAR__________________________________________________________________________________________________________________________________________________________

//declerations
void update_Proxy();
void musika();
void update_Color();
void set_speed(byte left, byte right);
void set_motors();
void update_PID();
void set_direction(char dir);
void getError();
void getLinePositionNum();
void printMotors();
void printErrorVal();
void printIRDigital();
void print_Colors();
void print_Proxy();
//end declerations


//print functions-------------------------------------------------------------------------------------------------------
void print_Proxy()
{
    Serial.print("Proximity: ");
    Serial.print(proximity_data);
    Serial.print("\t");
}

void print_Colors()
{
  // Serial.print("\tAmbient: ");
  //   Serial.print(ambient_light);
    Serial.print("Red: ");
    Serial.print(red_light);
    Serial.print(" Green: ");
    Serial.print(green_light);
    Serial.print(" Blue: ");
    Serial.println(blue_light);
}

//PRINT DATA
void printIRDigital()
{
    Serial.print("Digital Reading: ");
    Serial.print(Line_Position);
    Serial.print("\t");
}

void printErrorVal()
{
    Serial.print("Error Reading: ");
    Serial.println(error);
}

void printMotors()
{
    Serial.print("PID Value:  ");
    Serial.print(PID);
    Serial.print("\t");
    Serial.print("Left:  ");
    Serial.print(LEFT_MOTOR_SPEED);
    Serial.print("\t");
    Serial.print("Right:  ");
    Serial.print(RIGHT_MOTOR_SPEED);
    Serial.print("\t");
}

// void print_DEBUG()
// { 
//     print_Proxy();
//     print_Colors();
//     printMotors();
//     printIRDigital();
//     printErrorVal(); 
// }
//end print function-------------------------------------------------------------------------------------------------------

//APDS9960----------------------------------------------------------------------------------------------------------------

void update_Color(){
  // Read the light levels (ambient, red, green, blue)
  if (!apds.readAmbientLight(ambient_light) || !apds.readRedLight(red_light) || !apds.readGreenLight(green_light) || !apds.readBlueLight(blue_light)){    
    Serial.println("Error reading light values"); 
    }   else {
            musika();
             }
}
//set music-------------------------------------------------------------------------------------------------------------
void musika()
{
  if(PLAYED_MUSIC==false)
  {
    if (red_light > 200 && red_light > blue_light && red_light > green_light)
    {
      for(int i=0;i<3;i++){
        tone(buzzer, 1000);
        delay(100);
        noTone(buzzer);
      }
        PLAYED_MUSIC = true;
        return;
    }
    if (blue_light > 200 && blue_light > red_light && blue_light > green_light)
    {
      for(int i=0;i<3;i++){
        tone(buzzer, 2000);
        delay(100);
        noTone(buzzer);
      }
      PLAYED_MUSIC = true;
      return;
    }

    if (green_light > 200 && green_light > red_light && green_light > blue_light)
    {
      for(int i=0;i<3;i++){
        tone(buzzer, 2000);
        delay(100);
        noTone(buzzer);
      }
        PLAYED_MUSIC = true;
        return;
    }
  }
    PLAYED_MUSIC = false; 
}

//check proxy&play-----------------------------------------------------------------------------------------------------------------
void update_Proxy(){

  if ( !apds.readProximity(proximity_data) ) {       
     Serial.println("Error reading proximity value");  
        } else {    
                 if(proximity_data>=20)
                  { 
                    set_speed(40,40); //slow down car
                    while(proximity_data>=20 && proximity_data <230)//if you want to change distance 20 is start to slow down 230 is where to stop engines
                      { apds.readProximity(proximity_data); }
                        if(proximity_data>=230){  set_speed(0,0); }      
                  }//stop car
                    bool flag=false;
                     while(proximity_data>=100)    {      apds.readProximity(proximity_data); print_Colors(); update_Color(); delay(100); }  
               }
}

//END APDS9960--------------------------------------------------------------------------------------------------------------------

//MOTOR CONTROL--------------------------------------------------------------------------------------------------------------------

//check line position and set an error value---------------------------------------------------------------------------------------
void getError(){
    switch (Line_Position)    {
    //case 5111110:    {        error = -15;        break;    }
    //case 5111100:    {        error = -15;        break;    }
    //case 5111000:    {        error = -15;        break;    }
    case 51000000:    {        error = -6;         break;    }
    case 51100000:    {        error = -5;         break;    }
    case 51110000:    {        error = -4;         break;    }
    case 50110000:    {        error = -3;         break;    }
    case 50111000:    {        error = -2;         break;    }
    case 50011000:    {        error = -1;          break;    }
    case 50011100:    {        error = 0;          break;    }
    case 50001100:    {        error = 1;          break;    }
    case 50001110:    {        error = 2;          break;    }
    case 50000110:    {        error = 3;          break;    }
    case 50000111:    {        error = 4;          break;    }
    case 50000011:    {        error = 5;          break;    }
    case 50000001:    {        error = 6;          break;    }
    
    //case 5000111:    {        error = 15;         break;    } //turn right
    //case 5001111:    {        error = 15;         break;    } //turn right
    //case 5011111:    {        error = 15;         break;    }//turn right
    case 5000000:    {        error = 999;        break;    }//continue
    case 5111111:    {        error = 777;        break;    }//continue
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
void getLinePositionNum(){
    Line_Position = 5; //reset value
    for (int i = 0; i < IR_Sensor_Num; ++i)
        Line_Position = (Line_Position * 10) + digitalRead(IR_Sensor_Pin[i]);
}

//CALCULATE PID VALUE BASED ON ERRORS ----------------------------------------------------------------------------------------------
void update_PID(){
if (error != 777 && error != 999)//as long as its not white\black line calc a new pid value
    {
        P = error;
        I = I + prevI;
        D = error - prevError;
        PID = (Kp * P) + (Ki * I) + (Kd * D);
        prevI = I;
        prevError = error;
    }
}

//CONTROL MOTOR BASED ON PID----------------------------------------------------------------------------------------------------------
void set_motors(){
if (error == 777)//all black
{ 
    analogWrite(LEFT_MOTOR, INITIAL_MOTOR_SPEED +5);  //Left Motor Speed
    analogWrite(RIGHT_MOTOR, INITIAL_MOTOR_SPEED); //Right Motor Speed
    set_direction('F');
    return;
}

 if(error == 999) //all white - skip - do what you did last time until back on line
        return;

    LEFT_MOTOR_SPEED = INITIAL_MOTOR_SPEED + PID;
    RIGHT_MOTOR_SPEED= INITIAL_MOTOR_SPEED - PID;

 // The motor speed should not exceed the max PWM value
    LEFT_MOTOR_SPEED = constrain(LEFT_MOTOR_SPEED+5, 0, 255);
    RIGHT_MOTOR_SPEED = constrain(RIGHT_MOTOR_SPEED, 0, 255);

    set_speed(LEFT_MOTOR_SPEED,RIGHT_MOTOR_SPEED);
    set_direction('F');
}

//set motor analog speed--------------------------------------------------------------------------------------------------------------
void set_speed(byte left, byte right){
    analogWrite(LEFT_MOTOR, left+5);  //Left Motor Speed
    analogWrite(RIGHT_MOTOR, right); //Right Motor Speed
}

void sharpturn()
{
}


//END MOTOR CONTROL--------------------------------------------------------------------------------------------------------------------


void setup() {
  //total initialization time is 2 seconds + 1.2 seconds for starting buzzer
  Serial.begin(9600);  while(!Serial); // wait for serial to be ready
  if ( apds.init() ) {    Serial.println(F("APDS-9960 initialization complete"));  } else {    Serial.println(F("Something went wrong during APDS-9960 init!"));  }  // Initialize APDS-9960 (configure I2C and initial values)
  delay(500);// Wait for initialization and calibration to finish
  if ( !apds.setProximityGain(PGAIN_2X) ) {    Serial.println(F("Something went wrong trying to set PGAIN"));  }  // Adjust the Proximity sensor gain
  delay(500);    // Wait for initialization and calibration to finish
  if ( apds.enableProximitySensor(false) ) {    Serial.println(F("Proximity sensor is now running"));  } else {    Serial.println(F("Something went wrong during sensor init!"));  }// Start running the APDS-9960 proximity sensor (no interrupts)
  delay(500);    // Wait for initialization and calibration to finish
  if ( apds.enableLightSensor(false) ) {    Serial.println(F("Light sensor is now running"));  } else {    Serial.println(F("Something went wrong during light sensor init!"));  }  // Start running the APDS-9960 light sensor (no interrupts)
  delay(500);    // Wait for initialization and calibration to finish

  //Set Pins
  pinMode(RIGHT_MOTOR,OUTPUT);
  pinMode(RIGHT_FORWARD,OUTPUT);
  pinMode(RIGHT_BACK,OUTPUT);
  pinMode(LEFT_MOTOR,OUTPUT);
  pinMode(LEFT_FORWARD,OUTPUT);
  pinMode(LEFT_BACK,OUTPUT);

  
  //play a small audio to let the user know when it will start to move
  // tone(buzzer, 1000);
  // delay(1000);  //wait to allow manual alignment
  // noTone(buzzer);
  // tone(buzzer, 2000);
  // delay(100);
  // noTone(buzzer);
  // tone(buzzer, 3000);
  // delay(100);
  // noTone(buzzer);

}

void loop() {
    //update_Proxy(); //check proxy and if senses a block at 20 distance interrupt and stop the car and sense the color
    getLinePositionNum(); //get the line position from the IR sensors XXX-XXX
    getError();           //get the amount of sway off track
    
    //--debug--
  // print_Proxy();
  // print_Colors();
  //  printMotors();
    printIRDigital();
    printErrorVal(); 
    update_PID();//check PID
    set_motors();//set speed

    delay(1000);
}
