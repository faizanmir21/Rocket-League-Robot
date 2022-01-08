 //################//IMPORTING LIBRARIES//##################
#include <Pixy2.h>   
Pixy2 pixy;          


//////////--------------Intislize pin setup--------------//////////

#define SPEED 250   
#define TURN_SPEED 100    
#define speedPinR 12   //  Front Wheel PWM pin connect Right MODEL-X ENA (a02 and a01)- orange
#define RightMotorDirPin1  48    //Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
#define RightMotorDirPin2  46   //Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)                                 
#define LeftMotorDirPin1  36    //Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
#define LeftMotorDirPin2  38   //Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)
#define speedPinL 11  //  Front Wheel PWM pin connect Right MODEL-X ENB (b01 and b02)- yellow

#define speedPinRB 10   //  Rear Wheel PWM pin connect Left MODEL-X ENA (a01 and a02)-green
#define RightMotorDirPin1B  28    //Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
#define RightMotorDirPin2B 30  //Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1) 
#define LeftMotorDirPin1B 22    //Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)(b02 and b01)- blue
#define LeftMotorDirPin2B 24  //Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)
#define speedPinLB 9   //  Rear Wheel PWM pin connect Left MODEL-X ENB
#define hitterPWM 4
#define hitterDIR 34
#define hitterSpeed 7 


//////////--------------Define variables used--------------//////////

float deadZone = 0.15;                   //dead zone tha defines where the object will not move at all equals to 15%
int baseSpeed = 250;                   // base speed that the robot will run

int cont = 0;
int signature, x, y, width, height;
float cx, cy, area;

char dataString[50] = {0};
int a =0;

//////////--------------motor control functions--------------//////////

void go_advance(int speed){
   RL_fwd(speed);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_fwd(speed); 
}
void go_back(int speed){
   RL_bck(speed);
   RR_bck(speed);
   FR_bck(speed);
   FL_bck(speed); 
}
void right_shift(int speed_fl_fwd,int speed_rl_bck ,int speed_rr_fwd,int speed_fr_bck)//side way  right 
{
  FL_fwd(speed_fl_fwd); 
  RL_bck(speed_rl_bck); 
  RR_fwd(speed_rr_fwd);
  FR_bck(speed_fr_bck);
}
void left_shift(int speed_fl_bck,int speed_rl_fwd ,int speed_rr_bck,int speed_fr_fwd)//side way left
{
   FL_bck(speed_fl_bck);
   RL_fwd(speed_rl_fwd);
   RR_bck(speed_rr_bck);
   FR_fwd(speed_fr_fwd);
}

void left_turn(int speed)//right front and right rear clockwise
{
   RL_bck(0);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_bck(0); 
}
void right_turn(int speed)//left rear and left front clockwise
{
   RL_fwd(speed);
   RR_bck(0);
   FR_bck(0);
   FL_fwd(speed); 
}
void left_back(int speed)//right front and right rear anticlockwise
{
   RL_fwd(0);
   RR_bck(speed);
   FR_bck(speed);
   FL_fwd(0); 
}
void right_back(int speed)//left rear and left front anticlockwise
{
   RL_bck(speed);
   RR_fwd(0);
   FR_fwd(0);
   FL_bck(speed); 
}
void clockwise(int speed) //360 degree toward right
{
   RL_fwd(speed);
   RR_bck(speed);
   FR_bck(speed);
   FL_fwd(speed); 
}
void countclockwise(int speed) //360 degree toward left
{
   RL_bck(speed);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_bck(speed); 
}
void diag_front_left(int speed)
{
   FR_fwd(speed);
   RL_fwd(speed);
   RR_fwd(0);
   FL_fwd(0); 
}
void diag_front_right(int speed)
{
   FR_fwd(0);
   RL_fwd(0);
   RR_fwd(speed);
   FL_fwd(speed); 
}
void diag_rear_left(int speed)
{
   FR_bck(0);
   RL_bck(0);
   RR_bck(speed);
   FL_bck(speed); 
}
void diag_rear_right(int speed)
{
   FR_bck(speed);
   RL_bck(speed);
   RR_bck(0);
   FL_bck(0); 
}
void right_abit(int speed,int x)
{
  for(int i=0;i<x;i++)
  {
   i++;
   RL_fwd(speed);
   RR_bck(speed);
   FR_bck(speed);
   FL_fwd(speed); 
  } 
  stop_Stop();
}
void left_abit(int speed,int x)
{
  for(int i=0;i<x;i++)
  {
   i++;
   RL_bck(speed);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_bck(speed);
  } 
  stop_Stop();
}
void FR_fwd(int speed)  //front-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1,HIGH);
  digitalWrite(RightMotorDirPin2,LOW); 
  analogWrite(speedPinR,speed);
}
void FR_bck(int speed) // front-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1,LOW);
  digitalWrite(RightMotorDirPin2,HIGH); 
  analogWrite(speedPinR,speed);
}
void FL_fwd(int speed) // front-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
}
void FL_bck(int speed) // front-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
}

void RR_fwd(int speed)  //rear-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B,LOW); 
  analogWrite(speedPinRB,speed);
}
void RR_bck(int speed)  //rear-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B,HIGH); 
  analogWrite(speedPinRB,speed);
}
void RL_fwd(int speed)  //rear-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1B,HIGH);
  digitalWrite(LeftMotorDirPin2B,LOW);
  analogWrite(speedPinLB,speed);
}
void RL_bck(int speed)    //rear-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1B,LOW);
  digitalWrite(LeftMotorDirPin2B,HIGH);
  analogWrite(speedPinLB,speed);
}
 
void stop_Stop()    //Stop
{
  analogWrite(speedPinLB,0);
  analogWrite(speedPinRB,0);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,0);
}
//////////--------------motor control functions ends here--------------//////////

//////////--------------Hitter setup--------------//////////
void hitter(int speed)
{
  digitalWrite(hitterPWM,speed);
  digitalWrite(hitterDIR,HIGH);
}

//////////--------------hitter setup ends here--------------//////////

//////////--------------Pins initialize--------------//////////
void init_GPIO()
{
  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT); 
  pinMode(RightMotorDirPin2B, OUTPUT); 
  pinMode(speedPinLB, OUTPUT);  
 
  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT); 
  pinMode(speedPinRB, OUTPUT);

  pinMode(hitterDIR, OUTPUT);
  pinMode(hitterPWM, OUTPUT); 
  //pinMode(hitterSpeed, OUTPUT);
  
  stop_Stop();
}
//////////--------------Pins initialization ends here--------------//////////

///////////------------ pixy control map function------------------------////////////////////////

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

///////////------------ pixy control map function ends here ------------------------////////////////////////


////////////----------------- RFM69 Code Setup ----------------------------------//////////////////
// Include the RFM69 and SPI libraries:
#include <SPI.h>
#include <RFM69.h>

// Addresses for this node. CHANGE THESE FOR EACH NODE!
#define NETWORKID     10  // Must be the same for all nodes
#define MYNODEID      2   // My node ID
#define TONODEID      1   // Origin node ID

// RFM69 frequency, uncomment the frequency of your module:
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ

// AES encryption (or not):
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):
#define USEACK        false // Request ACKs or not

// Create a library object for our RFM69HCW module:
RFM69 radio;

#define PACKSIZE 25 // Maximum Packetsize is 25,if 4 digit for x and y, and static characters
#define GrnID 1 // #defined constants for clearer code
#define BluID 2
#define YelID 3

int gx, gy, bx, by, yx, yy; //Global variables for storing x and y coordinates of green, blue and yellow ball locations

char buildInt[4];

///////-----------------------------------RFM69 Code Ends ----------------------------------------//////////////

//////////-----------%%%%%%%%%%%%%---------- RFM69 Code Setup --------------%%%%%%%%%%%%%%%--------------/////////////
void pullCoords(char packet[], int letterLoc, int color) 
{
    int BrS,Comma,BrE = 0; // Array locations of bracket start character, comma and bracket end character.
    int i=0;
  
    BrS=letterLoc+1; // The start bracket will be immediately after the color letter indicator
    for (i = BrS+1; i < PACKSIZE; i++) {
      if(packet[i]==',') Comma=i;  // Find the location of the comma that divides x and y coordinates
      if(packet[i]=='>') {
        BrE=i;  // Find the location of the end bracket
        break;  // And get out of for loop
      }
    }
    sprintf(buildInt, "    ");  // Clear the buildInt array      
    for (i = 0; i < (Comma-(BrS+1)); i++) {  // Work through the chars that represent the x coordinate
      buildInt[i]=packet[(BrS+1+i)];         // and copy them into the buildInt array
    }
    buildInt[(Comma-(BrS+1))]='\0';  // Terminate the buildInt array to make it easy on the converter
    switch (color) {                 // Convert the buildInt array to an integer and store in the correspoding color x coordinate
      case GrnID: 
        gx=atoi(buildInt); // Green x coord
        break;
      case BluID: 
        bx=atoi(buildInt); // Blue x coord
        break;
      case YelID: 
        yx=atoi(buildInt); // Yellow x coord
        break;
    }
    sprintf(buildInt, "    ");  // Clear the buildInt array 
    for (i = 0; i < (BrE-(Comma+1)); i++) {  // Work through the chars that represent the y coordinate
      buildInt[i]=packet[(Comma+1+i)];         // and copy them into the buildInt array
    }
    buildInt[(BrE-(Comma+1))]='\0';  // Terminate the buildInt array to make it easy on the converter
    switch (color) {                 // Convert the buildInt array to an integer and store in the correspoding color y coordinate
      case GrnID: 
        gy=atoi(buildInt); // Green y coord
        break;
      case BluID: 
        by=atoi(buildInt); // Blue y coord
        break;
      case YelID: 
        yy=atoi(buildInt); // Yellow y coord
        break;
    }
}
///////----------------%%%%%%%%%%%%--------------RFM69 Code Ends --------------------%%%%%%%%%%%----------------////////////// 

void setup()
{
  init_GPIO();
  pixy.init();   
//////////----------------- RFM69 Code Setup ----------------------------/////////////
 // Open a serial port so we can see the data we receive
  Serial.begin(115200);
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");  

  sprintf(buildInt, "    "); // Clear the buildInt array

  // Initialize the RFM69HCW:
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW

  // Turn on encryption if desired:
  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY); 
    
///////------------------------------RFM69 Code Ends ------------------------------------//////////////  

}

void loop() 
{
 static int i = 0;
 uint16_t blocks;
 char buf[32];
 blocks = pixy.ccc.getBlocks();
  if (blocks)
  {
    signature = pixy.ccc.blocks[0].m_signature;
    height = pixy.ccc.blocks[0].m_height; //height of the object
    width = pixy.ccc.blocks[0].m_width; //width of the object
    x = pixy.ccc.blocks[0].m_x;//x value of the object
    y = pixy.ccc.blocks[0].m_y;//y value of the object
    cx = mapfloat(x, 0, 320, -1, 1); // aplying normalization. If value is from 0-320 change from -1 to 1. This helps in the computation
    cy = mapfloat(y, 0, 200, 1, -1);
    // send to pi
    a++;  
    // send the cx,cy,height and width to PI from arduino                       
    Serial.print(cx);Serial.print(",");Serial.print(cy);Serial.print(",");Serial.print(width);Serial.print(",");Serial.println(height);   // send the data
    //Serial.print("Green:");Serial.print(',');Serial.print(108);Serial.print(',');Serial.println(17);
    //Serial.print("yellow:");Serial.print(',');Serial.print(208);Serial.print(',');Serial.println(245);
    //Serial.print("blue:");Serial.print(',');Serial.print(250);Serial.print(',');Serial.println(100);
    delay(1000);   
      if(Serial.available() > 0) {
        int data = Serial.read();
        switch(data){
        case 2:{  
          go_advance(180);
          Serial.print("fwd");
          break;
        }
        case 6:{
           right_turn(180);
           Serial.print("right");
           break;
        }
        case 4:{
          left_turn(180);
          Serial.print("left");
          break;
        }
        case 68://rotate right a little
        {
          right_abit(180,10);
          Serial.print("right a bit");
          break;
        }
        case 48://rotate left a little
        {
          left_abit(180,10);
          Serial.print("left a bit");
          break;
        }
        case 688://rotate right a bigger
        {
          right_abit(180,80);
          Serial.print("right a bigger bit");
          break;
        }
        case 488://rotate left a bigger
        {
          left_abit(180,80);
          Serial.print("left a bigger bit");
          break;
        }
        case 55:{
          stop_Stop();
          hitter(120);
          Serial.print("stop and hit");
          break;
        }
        default: { 
            // statements
            break;
        }
    }
    }
  }
  else
  {
      cont += 1;
      if (cont == 100) {
        cont = 0;
        cx = 0;
        clockwise(150);
        Serial.println("turn clock");}
 }
//////////--------------------- RFM69 Code Setup ----------------------------/////////////
// RECEIVING
// In this section, we'll check with the RFM69HCW to see// if it has received any packets:
  int j=0;
  char data[PACKSIZE];
  for (j=0; j<PACKSIZE; j++) data[j]=0; //Clear the data array to make sure there is no left over junk in it
  if (radio.receiveDone()) { // Got one!
    // Print out the information:
    Serial.print("received from node ");
    Serial.print(radio.SENDERID, DEC);
    Serial.print(", message [");

    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:
    for (byte i = 0; i < radio.DATALEN; i++){
      Serial.print((char)radio.DATA[i]);
      data[i]=(char)radio.DATA[i];
      j=i;
      }
    // RSSI is the "Receive Signal Strength Indicator",
    // smaller numbers mean higher power.
    Serial.print("], RSSI ");
    Serial.println(radio.RSSI);
    
    // Now parse the data received into coordinates as integers
    j=0;
    while (j<PACKSIZE) {  //Work through the data and look for color letter indicators
      if(data[j]=='G') {  //Found green
        pullCoords(data,j,GrnID); //Get the green x and y (as ints) and print them.
        Serial.print("Green:");Serial.print(',');Serial.print(gx);Serial.print(',');Serial.println(gy);
        break;                    // Assuming green is only color in its packet
      } else if (data[j]=='B') { //Found blue
        pullCoords(data,j,BluID); //Get the blue x and y (as ints) and print them.
        Serial.print("Blue:");Serial.print(',');Serial.print(bx);Serial.print(',');Serial.println(by);
        j++;                      //Asuming blue and yellow and together, blue first
      } else if (data[j]=='Y') { //Found yellow
        pullCoords(data,j,YelID); //Get the yellow x and y (as ints) and print them.
        Serial.print("Yellow:");Serial.print(',');Serial.print(yx);Serial.print(',');Serial.println(yy);
        break;                    // Assuming yellow is at the end of its packet      
      } else {
        j++;                      // Go to next character
      }
    }
  }
///////------------------------------RFM69 Code Ends ------------------------------------////////////// 

}
