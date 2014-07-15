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
const int irPinL   = 14;    //pin reserved for Sharp IR input (analog)
const int irPinR  	= 15;    //front IR sensor  
const int pingPinR  = 52;  //pin reserved for ping sensor input (digital)
const int pingPinL  = 53;
const int cdsPin1 	= 0;     //pin reserved for photoresistor input (analog)
const int cdsPin2   = 0; 

/*--- servo pins ---*/
//const int servPin = 8   //pin reserved for servo output (PWM)

/*--- motor pins ---*/
const int l_EnPin = 7;  //left motor enable pin
const int l_hPin1 = 5;  //left motor hbridge pins
const int l_hPin2 = 4;
const int r_EnPin = 6;  //right motor enable pin (for PWM speed ctrl)
const int r_hPin1 = 3;  //right motor hbridge pins
const int r_hPin2 = 2;

/*--- global variables ---*/
int mode = 0;		//holds the arbitration decision 
int speed = 80;       	//speed in PWM format [0 - 255]


/*--- intitialize ---*/
Robot robo;    //start the Robot, with Serial debugging ON
                  //refer to Robot class definition for capabilities and code

/* --------------------------------------------------------------------
 * -----------                   SETUP                     ------------
 * -------------------------------------------------------------------- */
void setup() {
	//open serial connection
	Serial.begin(9600);
	//set the Robot class up with the pin info for each motor
	robo.setLeft(l_EnPin, l_hPin1, l_hPin2);
	robo.setRight(r_EnPin, r_hPin1, r_hPin2);
	Serial.println("- - - Follower Robot - - -");
  	Serial.println("   Enter 0 for Follower Robot");
	Serial.println("   Enter s,S to stop all movement");  
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
      Serial.println("shouldn't be here... Invalid Mode Selection");
  }
}

void follower(){
  int diff = 0; //variable to hold the differential turning 
  static int threshold = 750; //the minimum light level we will consider
  static int MaxTurn = 30;    //the maximum differential we wish to use
  static int MaxReading = 250;//the maximum delta between light sensor readings expected
  static int MaxSpeed = RPM*0.7;
  static int MinSpeed = RPM*0.3;
  //get Cds Sensor data
  int leftSens  = analogRead(cdsPin1);
  int rightSens = analogRead(cdsPin2);
  //set differential turning
  //..if at least one sensor is over the threshold
  if (leftSens > threshold || rightSens > threshold){
    //we know we found the bot in front,
    //so turn toward it, 
    //the higher reading indicates the direction the bot ahead is in
    //the difference in the two sensors is a rough indication of how far in that direction
    //negative differentials will turn left, positive to the right
    diff = rightSens - leftSens;  //find the difference
    diff = max(-MaxReading, diff);  //set a floor for the sensor differnce
    diff = min(MaxReading, diff);   //set a ceiling
    diff = map(diff, -MaxReading, MaxReading, -MaxTurn, MaxTurn); //map to the differential
  }
  //get front sensor data - distance returned as float, cast to int and *1000
  int leftIR  = robo.IRdistance_mm(irPinL);
  int rightIR = robo.IRdistance_mm(irPinR);
  int distance = (leftIR + rightIR)/2;  //dummy value for the time being
  //do some math to figure out where the bot is
    //need to check both, reject anything with too big of a difference
    //we could also use these readings to determine if we are pointed at the bot ahead properly
    //instead of the above section
  //calculate PID for distance with error above
  int correction = PIDcalc(distance);
    //too close will give a negative value, we want to add that to the current speed
    //too far will give a positive value, we want to add that to the current speed
    //then we need to set a floor and ceiling for how fast the bot can go
  speed = speed + correction;
  speed = max(MinSpeed, speed);
  speed = min(MaxSpeed, speed);
  //determine if we need to speed up or slow down
  //have the bot drive in the direction and speed necessary
  robo.setSpeed(speed);
  robo.drive_dif(diff);
}

int PIDcalc(int distance){
  static int setPoint = 500; //distance to the bot in front in mm
  //setup & tuning variables, 
  //below are for 300 RPM motors at 80/255 speed setting.
  const float Ku = 8.25;
  const float Tu = 3400;
  const float Kp = Ku*0.45;
  const float Ki = Kp*2/Tu;
  const float Kd = Kp*Tu/3;
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
  error = (float)(setPoint - distance);
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
