//libraries------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <SparkFun_APDS9960.h>
//end libraries-------------------------------------------------------------------------------------------------

//defines-------------------------------------------------------------------------------------------------------
//DEFINES FOR MUSIC
//#define NOTE_B0  31
//#define NOTE_C1  33
//#define NOTE_CS1 35
//#define NOTE_D1  37
//#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
//#define NOTE_FS6 1480
//#define NOTE_G6  1568
//#define NOTE_GS6 1661
//#define NOTE_A6  1760
//#define NOTE_AS6 1865
//#define NOTE_B6  1976
//#define NOTE_C7  2093
//#define NOTE_CS7 2217
//#define NOTE_D7  2349
//#define NOTE_DS7 2489
//#define NOTE_E7  2637
//#define NOTE_F7  2794
//#define NOTE_FS7 2960
//#define NOTE_G7  3136
//#define NOTE_GS7 3322
//#define NOTE_A7  3520
//#define NOTE_AS7 3729
//#define NOTE_B7  3951
//#define NOTE_C8  4186
//#define NOTE_CS8 4435
//#define NOTE_D8  4699
//#define NOTE_DS8 4978
#define REST 0
//end defines-----------------------------------------------------------------------------------------------------

//MUSIC -------------------------------------------------------------------------------------------------------------------
bool PLAYED_MUSIC=false;
int melodywii[] = {

  NOTE_FS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, //1
  NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, REST,4, REST,8, NOTE_CS4,8,
  NOTE_D4,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
  NOTE_E5,-4, NOTE_DS5,8, NOTE_D5,8, REST,8, REST,4,
  
  NOTE_GS4,8, REST,8, NOTE_CS5,8, NOTE_FS4,8, REST,8,NOTE_CS5,8, REST,8, NOTE_GS4,8, //5
  REST,8, NOTE_CS5,8, NOTE_G4,8, NOTE_FS4,8, REST,8, NOTE_E4,8, REST,8,
  NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,8, REST,4, NOTE_E4,8, NOTE_E4,8,
  NOTE_E4,8, REST,8, REST,4, NOTE_DS4,8, NOTE_D4,8, 

  NOTE_CS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, //9
  NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, NOTE_E5,8, NOTE_E5,8, NOTE_E5,8, REST,8,
  REST,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
  NOTE_E5,2, NOTE_D5,8, REST,8, REST,4,

  NOTE_B4,8, NOTE_G4,8, NOTE_D4,8, NOTE_CS4,4, NOTE_B4,8, NOTE_G4,8, NOTE_CS4,8, //13
  NOTE_A4,8, NOTE_FS4,8, NOTE_C4,8, NOTE_B3,4, NOTE_F4,8, NOTE_D4,8, NOTE_B3,8,
  NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,4, REST,4, NOTE_AS4,4,
  NOTE_CS5,8, NOTE_D5,8, NOTE_FS5,8, NOTE_A5,8, REST,8, REST,4, 

};

int melodyharry[] = {
  REST, 2, NOTE_D4, 4,
  NOTE_G4, -4, NOTE_AS4, 8, NOTE_A4, 4,
  NOTE_G4, 2, NOTE_D5, 4,
  NOTE_C5, -2, 
  NOTE_A4, -2,
  NOTE_G4, -4, NOTE_AS4, 8, NOTE_A4, 4,
  NOTE_F4, 2, NOTE_GS4, 4,
  NOTE_D4, -1, 
  NOTE_D4, 4,

  NOTE_G4, -4, NOTE_AS4, 8, NOTE_A4, 4, //10
  NOTE_G4, 2, NOTE_D5, 4,
  NOTE_F5, 2, NOTE_E5, 4,
  NOTE_DS5, 2, NOTE_B4, 4,
  NOTE_DS5, -4, NOTE_D5, 8, NOTE_CS5, 4,
  NOTE_CS4, 2, NOTE_B4, 4,
  NOTE_G4, -1,
  NOTE_AS4, 4,
     
  //  NOTE_D5, 2, NOTE_AS4, 4,//18
  //  NOTE_D5, 2, NOTE_AS4, 4,
  //  NOTE_DS5, 2, NOTE_D5, 4,
  //  NOTE_CS5, 2, NOTE_A4, 4,
  //  NOTE_AS4, -4, NOTE_D5, 8, NOTE_CS5, 4,
  //  NOTE_CS4, 2, NOTE_D4, 4,
  //  NOTE_D5, -1, 
  //  REST,4, NOTE_AS4,4,  

  // NOTE_D5, 2, NOTE_AS4, 4,//26
  // NOTE_D5, 2, NOTE_AS4, 4,
  // NOTE_F5, 2, NOTE_E5, 4,
  // NOTE_DS5, 2, NOTE_B4, 4,
  // NOTE_DS5, -4, NOTE_D5, 8, NOTE_CS5, 4,
  // NOTE_CS4, 2, NOTE_AS4, 4,
  // NOTE_G4, -1, 
  
};
int melody[] = {

  NOTE_A4,-4, NOTE_A4,-4, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_F4,8, REST,8,
  NOTE_A4,-4, NOTE_A4,-4, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_F4,8, REST,8,
  NOTE_A4,4, NOTE_A4,4, NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16,

  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,//4
  NOTE_E5,4, NOTE_E5,4, NOTE_E5,4, NOTE_F5,-8, NOTE_C5,16,
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
  
};

byte buzzer = A0;
byte tempoharry = 144;
byte tempowii = 114;
byte tempo = 120;

int noteswii = sizeof(melodywii) / sizeof(melodywii[0]) / 2;
int notesharry = sizeof(melodyharry) / sizeof(melodyharry[0]) / 2;
int notes = sizeof(melody) / sizeof(melody[0]) / 2;

int wholenotewii = (60000 * 4) / tempowii;
int wholenoteharry = (60000 * 4) / tempoharry;
int wholenote = (60000 * 4) / tempo;

int divider = 0;
int noteDuration = 0;
//end music-----------------------------------------------------------------------------------------------------------



// Global Variables____________________________________________________________________________________________________________________________
SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint8_t proximity_data = 0;
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;


//motor pins------------------------------------------------------------------------------------------------------
const byte RIGHT_DIRECTION=10;//PH1
const byte RIGHT_MOTOR=11;
const byte LEFT_DIRECTION=6;//PH2
const byte LEFT_MOTOR=5;
const byte MODE=3;

//motor speed------------------------------------------------------------------------------------------------------
int LEFT_MOTOR_SPEED=0;
int RIGHT_MOTOR_SPEED=0;

//initial Motor Speed----------------------------------------------------------------------------------------------
const int INITIAL_MOTOR_SPEED=150;


//IR SENSORS--------------------------------------------------------------------------------------------------------
const byte IR_Sensor_Num=7;
const byte IR_Sensor_Pin[IR_Sensor_Num]={13,12,9,2,8,7,4};

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
  if(PLAYED_MUSIC==false){
    if (red_light > 200 && red_light > blue_light && red_light > green_light)
    {
        for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2)
        {
            divider = melody[thisNote + 1];
            if (divider > 0)
            {
                noteDuration = (wholenote) / divider;
            }
            else if (divider < 0)
            {
                noteDuration = (wholenote) / abs(divider);
                noteDuration *= 1.5;
            }
            tone(buzzer, melody[thisNote], noteDuration * 0.9);
            delay(noteDuration);
            noTone(buzzer);
        }
        
        PLAYED_MUSIC = true;
        return;
    }

    if (blue_light > 200 && blue_light > red_light && blue_light > green_light)
    {
        for (int thisNote = 0; thisNote < noteswii * 2; thisNote = thisNote + 2)
        {
            divider = melodywii[thisNote + 1];
            if (divider > 0)
            {
                noteDuration = (wholenotewii) / divider;
            }
            else if (divider < 0)
            {
                noteDuration = (wholenotewii) / abs(divider);
                noteDuration *= 1.5;
            }
            tone(buzzer, melodywii[thisNote], noteDuration * 0.9);
            delay(noteDuration);
            noTone(buzzer);
        }
                PLAYED_MUSIC = true;
                return;

    }

    if (green_light > 200 && green_light > red_light && green_light > blue_light)
    {
        for (int thisNote = 0; thisNote < notesharry * 2; thisNote = thisNote + 2)
        {
            divider = melodyharry[thisNote + 1];
            if (divider > 0)
            {
                noteDuration = (wholenoteharry) / divider;
            }
            else if (divider < 0)
            {
                noteDuration = (wholenoteharry) / abs(divider);
                noteDuration *= 1.5;
            }
            tone(buzzer, melodyharry[thisNote], noteDuration * 0.9);
            delay(noteDuration);
            noTone(buzzer);
        }
        PLAYED_MUSIC = true;
        return;
    }
           PLAYED_MUSIC = false; 

  }
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

//Iterate over the IR sensors, and create a 6digit num (sensor)*10+new sensor - 000000 001100 010000
void getLinePositionNum(){
    Line_Position = 5; //reset value
    for (int i = 0; i < IR_Sensor_Num; ++i)
        Line_Position = (Line_Position * 10) + digitalRead(IR_Sensor_Pin[i]);
}

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
if (error == 777 || error == 999) //all black\white - skip
        return;

    LEFT_MOTOR_SPEED = INITIAL_MOTOR_SPEED + PID;
    RIGHT_MOTOR_SPEED= INITIAL_MOTOR_SPEED - PID;

 // The motor speed should not exceed the max PWM value
    LEFT_MOTOR_SPEED = constrain(LEFT_MOTOR_SPEED, 0, 255);
    RIGHT_MOTOR_SPEED = constrain(RIGHT_MOTOR_SPEED, 0, 255);

    analogWrite(LEFT_MOTOR, LEFT_MOTOR_SPEED+5);  //Left Motor Speed
    analogWrite(RIGHT_MOTOR, RIGHT_MOTOR_SPEED); //Right Motor Speed

    set_direction('F');
}

//set motor analog speed--------------------------------------------------------------------------------------------------------------
void set_speed(byte left, byte right){
  analogWrite(LEFT_MOTOR, left+5);  //Left Motor Speed
    analogWrite(RIGHT_MOTOR, right); //Right Motor Speed
}

//set direction if wanting to turn ----------------------------------------------------------------------------------------------------
//TURN - set wheels direction - 0=forward 1=backward
void set_direction(char dir)
{
    switch (dir)
    {
    case 'F':    {        digitalWrite(LEFT_DIRECTION, LOW);        digitalWrite(RIGHT_DIRECTION, LOW);          break;    }//forward
    case 'B':    {        digitalWrite(LEFT_DIRECTION, HIGH);       digitalWrite(RIGHT_DIRECTION, HIGH);         break;    }//backwards
   // case 'L':    {        digitalWrite(LEFT_DIRECTION, LOW);        analogWrite(RIGHT_MOTOR, 0);                 break;    }//left turn 90
   // case 'R':    {        analogWrite(LEFT_MOTOR, 0);               digitalWrite(RIGHT_DIRECTION, LOW);          break;    }//right turn 90
    case 'S':    {        analogWrite(LEFT_MOTOR, 0);               analogWrite(RIGHT_MOTOR, 0);                 break;    }//stop
    }
}

//END MOTOR CONTROL--------------------------------------------------------------------------------------------------------------------


void setup() {
  //total initialization time is 2 seconds + 1.2 seconds for starting buzzer
  Serial.begin(9600);
  while(!Serial); // wait for serial to be ready
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {    Serial.println(F("APDS-9960 initialization complete"));  } else {    Serial.println(F("Something went wrong during APDS-9960 init!"));  }
    // Wait for initialization and calibration to finish
  delay(500);
  // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(PGAIN_2X) ) {    Serial.println(F("Something went wrong trying to set PGAIN"));  }
    // Wait for initialization and calibration to finish
  delay(500);
  // Start running the APDS-9960 proximity sensor (no interrupts)
  if ( apds.enableProximitySensor(false) ) {    Serial.println(F("Proximity sensor is now running"));  } else {    Serial.println(F("Something went wrong during sensor init!"));  }
    // Wait for initialization and calibration to finish
  delay(500);
  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {    Serial.println(F("Light sensor is now running"));  } else {    Serial.println(F("Something went wrong during light sensor init!"));  }
    // Wait for initialization and calibration to finish
  delay(500);
  

  //Set Pins
  pinMode(MODE,OUTPUT);
  digitalWrite(MODE,HIGH); //set motor driver to PHASE ENABLE 
  pinMode(RIGHT_DIRECTION,OUTPUT);
  pinMode(RIGHT_MOTOR,OUTPUT);
  pinMode(LEFT_DIRECTION,OUTPUT);
  pinMode(LEFT_MOTOR,OUTPUT);
  
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
  //printIRDigital();
  // printErrorVal(); 
    update_PID();//check PID
    set_motors();//set speed

    delay(1000);
}
