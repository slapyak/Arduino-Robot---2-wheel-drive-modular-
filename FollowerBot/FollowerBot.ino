/* *******************************************************************************
* Refer to contributors.txt for authorship information
* ECE 450 - Semester Project Follower Robot
* ***************************************************************************** */
#include <Robot.h>
#define LOG 0 
#define DB  1
#define RPM (9.6/12*300) //RPM rating of current bot at operating voltage

/*--- function declarations ---*/
void serialEvent();
void serialCommand(char ch);
int PIDcalc(int distance);

/*--- sensor pins ---*/
//const int irPinBot   = 0;    //pin reserved for Sharp IR input (analog)
const int irTopPin   = 0;    //front IR sensor  
const int irBottomPin = 1;
//const int pingPinR  = 52;  //pin reserved for ping sensor input (digital)
//const int pingPinL  = 53;
const int cdsPinRt = 7;     //pin reserved for photoresistor input (analog)
const int cdsPinLt = 8; 

/*--- servo pins ---*/
//const int servPin = 8   //pin reserved for servo output (PWM)

/*--- motor pins ---*/
const int l_EnPin = 2;  //left motor enable pin
const int l_hPin1 = 10;  //left motor hbridge pins
const int l_hPin2 = 9;
const int r_EnPin = 3;  //right motor enable pin (for PWM speed ctrl)
const int r_hPin1 = 5;  //right motor hbridge pins
const int r_hPin2 = 6;

/*--- global variables ---*/
int mode = 1;		//holds the arbitration decision 
int speed = 180;       	//speed in PWM format [0 - 255]
//const int FILTER_SIZE = 10;
//static float irFilterLeft[FILTER_SIZE];
//static float irFilterRight[FILTER_SIZE];


/*--- intitialize ---*/
Robot robo;    //start the Robot, with Serial debugging ON
                  //refer to Robot class definition for capabilities and code

const long referenceMv = 5000;

/* --------------------------------------------------------------------
 * -----------                   SETUP                     ------------
 * -------------------------------------------------------------------- */
void setup() {
  
	//open serial connection
	Serial.begin(9600);
	//set the Robot class up with the pin info for each motor
	robo.setLeft(l_EnPin, l_hPin1, l_hPin2);
	robo.setRight(r_EnPin, r_hPin1, r_hPin2);
	if (DB) {Serial.println("- - - Follower Robot - - -");
  	Serial.println("   Enter 0 for Follower Robot");
	  Serial.println("   Enter s,S to stop all movement");  
  }
	//make sure the robot is stopped
	robo.stop();
	robo.setSpeed(speed); //set speed for turning/driving
	delay(2000);	//give some time to put the robot down after reset

 }

void loop(){
	//do stuff
  switch(mode){
    case 0: //stop mode - take no action
      break;
    case 1: //standard follower mode
      follower(); //call the follower function
      break;
    case 2:
      break;
    default:
      mode = 3;//Serial.println("shouldn't be here... Invalid Mode Selection");
  }
}


void follower(){
  int diff = 0; //variable to hold the differential turning
  float irRatio = 0;
  static int threshold = 700; //the minimum light level we will consider
  static int MaxTurn = 100;    //the maximum differential we wish to use
  static int MaxReading = 100;//the maximum delta between light sensor readings expected
  static int MaxSpeed = 90;
  static int MinSpeed = 35;
  
  float irTopDist  = robo.IRdistance_mm(irTopPin);    //getIrDist(irTopPin);
  float irBottomDist = robo.IRdistance_mm(irBottomPin);       //getIrDist(irBottomPin);
  
  
  if(irTopDist > 1000){
    irTopDist = 1000;
  }
  
  if(irBottomDist > 1000){
   irBottomDist = 1000; 
  }
  
  float irDelta = abs(irTopDist - irBottomDist);
  if(irTopDist < irBottomDist){
    diff = (int)(irDelta * -.3);
  } else {
    diff = (int)(irDelta * .3);
  }
  
  if (DB){
  Serial.print("IR TOP: ");
  Serial.print(irTopDist);
  Serial.print(" \tIR BOT: ");
  Serial.print(irBottomDist);
  Serial.print(" \tDif: ");
  Serial.print(diff);
  }
  
  float irMax = max(irTopDist, irBottomDist);
  float irMin = min(irTopDist, irBottomDist);
  
  if(irMax == 0){
    irMax = 1; 
  }
  
  irRatio = irMin / irMax;
  
  float distance;
  
  if(irRatio < 0.5){
    distance = irMin;
    diff = diff * 3;
    //speed = 75;
  } else {
    distance = (irTopDist + irBottomDist)/2;
  } 
  
  int correction = PIDcalc(distance);    
  if(distance < 200){
      speed = MinSpeed;
  } else {
    speed = speed + correction;
    speed = max(MinSpeed, speed);
    speed = min(MaxSpeed, speed);
   }

  diff = map(diff, -250, 250, -100, 100);
  diff = min(diff, 100);
  diff = max(diff,-100);
  Serial.print(" \tDI-2: ");
  Serial.print(diff);
  Serial.print(" \tSpeed: ");
  Serial.println(speed);
  robo.setSpeed(speed);
  robo.drive_dif(diff, -1, 65);
}

int PIDcalc(int distance){
  static int setPoint = 300; //distance to the bot in front in mm
  //setup & tuning variables, 
  //below are for 300 RPM motors at 80/255 speed setting.
  const float Ku = 0.8;
  const float Tu = 3400;
  const float Kp = Ku;// *0.45;
  const float Ki = 0; // Kp*2/Tu;
  const float Kd = 0; // Kp*Tu/3;
  //terms used each time
  static float lastError = 0;  //the value of the last error - stored between function calls
  static float errorSum = 0;   //sum of all errors - stored between calls
  static float dError = 0;         //change in error between this and last function call
  static unsigned long lastTime = 0;  //last time the function was called
  unsigned long now = millis(); //current time in milliseconds 
  float timeChange = (now - lastTime);  //self explanitory?
  float error;                 //current error
  float Pterm;                 //proportional term
  float Iterm;                 //integral term
  float Dterm;                 //derivative term
  int output;
  //Use PID function from Lab #6
  error = (float)(distance - setPoint);
  errorSum += (error*timeChange);       
  dError = (error - lastError)/timeChange;
  Pterm = (Kp*error);
  Iterm = (Ki*errorSum);
  Dterm = (Kd*dError);
  output = (Pterm + Iterm + Dterm);
  //
  lastError = error;
  lastTime = now;
  //
  if(LOG){ 
    Serial.print(" \t"); Serial.print(error);
    Serial.print(" \t"); Serial.print(Pterm);
    Serial.print(" \t"); Serial.print(Iterm);
    Serial.print(" \t"); Serial.print(Dterm);
    Serial.print(" \t"); Serial.print(output);
    Serial.print(" \t"); Serial.print(now); 
  }
  //
  return (int)output;
}

/* --------------- SerialEvent ---------------
 occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    serialCommand(inChar);
    } 
 }

/* --------------- serialCommand ---------------
 each character coming in the serial line is evaluated
 against known commands - commands listed in the setup()
 */
void serialCommand(char ch){
  if (ch == 's' || ch == 'S') {
    robo.stop();
    mode = -1;
    Serial.println("STOP COMMAND RECIEVED");
  } else if (ch == '0')  { //user selected wallfollower mode
    mode = 0;
    Serial.println("Go Forward! Move Ahead! It's not too late!");
  } else if (ch == '+')  { //user selected wallfollower mode
    speed = Serial.parseInt();
    robo.setSpeed(speed);
    Serial.print("SPEED updated to : ");  Serial.println(speed);
  } else {
    Serial.print(ch);
    Serial.println(" : unrecognized command");
  }
 }
 
float getIrDist(int pin){
  
  static float numerator = 177855.0;
  int val = analogRead(pin);
  int mV = (val * referenceMv) / 1023;
  
  float denominator = pow(mV, 1.275998);
  float dist = (numerator / denominator);
  return dist * 10; 
}