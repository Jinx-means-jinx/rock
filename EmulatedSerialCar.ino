/*
 * Control the car through the virtual serial port!!
 * 
 */

#include <SoftwareSerial.h>

// Serial is a virtual port, Rx is port A10, Tx is port A1
SoftwareSerial serial(A0, A1); 

//Configure THE PWM control pin
const int PWM2A = 11;      //M1 motor
const int PWM2B = 3;       //M2 motor   
const int PWM0A = 6;       //M3 motor  
const int PWM0B = 5;       //M4 motor

const int DIR_CLK = 4;     //Data input clock line
const int DIR_EN = 7;      //Equip the L293D enabling pins
const int DATA = 8;        // cable
const int DIR_LATCH = 12;  //Output memory latch clock

//Define the pin of ultrasonic obstacle avoidance sensor
//A2 is the pin Trig connected to the ultrasonic sensor
//A3 is the pin Echo connected to the ultrasonic sensor
const int Trig = A2; 
const int Echo = A3;  

//Define motion state
const int Forward = 39;//39 is stored in the Forward variable
const int Back = 216;  //216 is stored in the Back variable
const int Left = 116;   //116 is stored in the Left variable 
const int Right = 139; //The right amount of change 139
const int Stop = 0;    //Parking variable

//Set the default speed between 1 and 255
int Speed1 = 190; //PWM0B -M3
int Speed2 = 190; //PWM0A -M4
int Speed3 = 190; //PWM2A -M3 
int Speed4 = 190; //PWM2B -M4

//Variables for storing ultrasonic sensor measurements
int distance = 0;       
// Store the data received from the serial port in serialData    
char serialData;            
char cmd;  //Store bluetooth receive commands                 

void setup() 
{
    Serial.begin(9600);//Set the serial port baud rate 9600
   // motor.setSpeed(MAX_SPEED);

   // Initialize the virtual serial port
  serial.begin(9600); 
  
    //Configure for pin mode
    pinMode(DIR_CLK,OUTPUT);
    pinMode(DATA,OUTPUT);
    pinMode(DIR_EN,OUTPUT);
    pinMode(DIR_LATCH,OUTPUT);
    pinMode(PWM0B,OUTPUT);
    pinMode(PWM0A,OUTPUT);
    pinMode(PWM2A,OUTPUT);
    pinMode(PWM2B,OUTPUT);

    //The Trig pin connected to the ultrasound is set to output mode
    //The Echo pin connected to the ultrasound is set to input mode
    pinMode(Trig,OUTPUT);
    pinMode(Echo,INPUT);

} 

void loop()
{
    distance = SR04(Trig,Echo);// Obtain ultrasonic distance       
     
    bluetooh();// Call bluetooth car control function 
   
} 
/* 
* function name：Motor(); 
* Function: Change the movement direction and speed of
* the car through the entrance parameters
* Entry parameter 1: Dri car movement direction
* Entry parameters 2~3: Speed1~Speed4 motor speed, value range 0~255
* Dri value description (forward :39;
* Back: 216;
* Left translation: 116;
* Right translation: 139;
* Stop: 0;
* Right rotation: 106;
* Left rotation: 149)
* Return value: None
 */
void Motor(int Dir,int Speed1,int Speed2,int Speed3,int Speed4)
{
    analogWrite(PWM2A,Speed1); //Motor1 PWM speed regulation
    analogWrite(PWM2B,Speed2); //Motor2 PWM speed regulation
    analogWrite(PWM0A,Speed3); //Motor3 PWM speed regulation
    analogWrite(PWM0B,Speed4); //Motor4 PWM speed regulation
    
  //Set the low level and write the direction of motion to prepare
    digitalWrite(DIR_LATCH,LOW); 
  //Dir motion direction value writes
    shiftOut(DATA,DIR_CLK,MSBFIRST,Dir);
    //Set the high level and output the direction of motion
    digitalWrite(DIR_LATCH,HIGH);
}

/*
Function name: SR04()
Function: Obtain ultrasonic ranging data
Entry parameters: Trig, Echo
Function return value: cm
*/
int SR04(int Trig,int Echo)
{
    float cm = 0;

    digitalWrite(Trig,LOW);     //Trig Set to low level
    delayMicroseconds(2);       //Wait 2 microseconds
    digitalWrite(Trig,HIGH);    //Trig Set to high level
    delayMicroseconds(15);      //Wait 15 microseconds
    digitalWrite(Trig,LOW);     //Trig Set to low level

    cm = pulseIn(Echo,HIGH)/58.8; //Convert the ranging time to CM
    cm = (int(cm * 100.0))/100.0; //Leave 2 as a decimal
     //Character Distance displayed in serial port monitor window:
  //  Serial.print("Distance:");   
   // Serial.print(cm); 
  //  Serial.println("cm"); 

    return cm;      //Returns cm value ranging data
}
/*
* Function name: bluetooh()
* Function: Receive Bluetooth data, control the car movement direction
* Entry parameters: None
* Return value: None
*/
void bluetooh()
{
  // Determine whether the received data is greater than 0
    if(Serial.available() > 0)      
    {
        serialData = Serial.read(); // Receive function
        Serial.println(serialData); 
        // If the serial port receives data as character F, 
        //save F to CMD
        if     ('F' == serialData )  cmd = 'F';     
        // If the data received by the serial port is character B,
        //save F to CMD
        else if('B' == serialData )  cmd = 'B';     
        // If the data received by the serial port is L,
        //save F to CMD
        else if('L' == serialData )  cmd = 'L';   
        // If the data received by the serial port is character R,
        //save F to CMD
        else if('R' == serialData )  cmd = 'R';    
        else if('S' == serialData )  cmd = 'S';    
        
        // Received string +, speed increases
        else if( serialData == '+' && Speed1 < 245)
        {
            Speed1 += 10;   //We're going to increase our velocity by 10
            Speed2 = Speed1;
            Speed3 = Speed1;
            Speed4 = Speed1;
            Serial.print("Speed = "); 
            Serial.println(Speed1); 
        }

        //When I receive a string -- the speed decreases
        else if( serialData == '-' && Speed1 > 30)
        {
          // The velocity decreases by 10 each time
            Speed1 -= 10;   
            Speed2 = Speed1;
            Speed3 = Speed1;
            Speed4 = Speed1;
            Serial.print("Speed = "); 
            Serial.println(Speed1); 
        }

        // When bluetooth receives the string R, dolly panalizes to the right
         else if('D' == serialData) 
        {
            Motor(106,Speed1,Speed2,Speed3,Speed4);     
        }

        // When bluetooth receives the string R, dolly panalizes to the right
        else if('C' == serialData) 
        {
            Motor(149,Speed1,Speed2,Speed3,Speed4);  
            
        }
    }
  //If Bluetooth receives the string F,
  //the dolly moves forward and enables obstacle avoidance
    if('F' == cmd)   //
    {      
       Motor(Forward,Speed1,Speed2,Speed3,Speed4);    
    }
    //The ultrasonic obstacle avoidance function is called to 
    //realize the obstacle avoidance function
    else if('B' == cmd)    
    {   
        Motor(Back,Speed1,Speed2,Speed3,Speed4);
    }
    // Bluetooth received string L, car left translation
    else if('L' == cmd)    
    {              
        Motor(Left,Speed1,Speed2,Speed3,Speed4); 
    }
    // When bluetooth receives the string R, dolly panalizes to the right
    else if('R' == cmd)    
    {
        Motor(Right,Speed1,Speed2,Speed3,Speed4);         
    }
    else if('A' == serialData)     
    { 
         AvoidingObstacles();
    }
   // The cart stops moving when the string S is received
    else if('S' == serialData)     
    { 
        Motor(Stop,0,0,0,0); 
    }
}

void AvoidingObstacles()
{
  // If the distance is greater than 20cm or

      if(distance > 20)
      {
        // Call forward function
          Motor(Forward,Speed1,Speed2,Speed3,Speed4);   
      }
      // Otherwise the distance is less than 20
      else 
      {
          Motor(Back,Speed1,Speed2,Speed3,Speed4);
          delay(500);
          // Turn left to change the direction of the car
          Motor(106,Speed1,Speed2,Speed3,Speed4);
          delay(500);
      } 
}
