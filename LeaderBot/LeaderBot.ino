/* *******************************************************************************
* Refer to contributors.txt for authorship information
* ECE 450 - Semester Project Leader Robot
* ***************************************************************************** */
#include <Robot.h>
#define LOG 0
#define DB  1

/*--- function declarations ---*/
void serialEvent();
void serialCommand(char ch);
float PIDcalculate(float distance, int reset=0);

/*--- sensor pins ---*/
const int irPinLR   = 14;    //pin reserved for Sharp IR input (analog)
const int irPinLF   = 15;    //front IR sensor  
const int irPinR    = 13;

const int pingPinR  = 52;  //pin reserved for ping sensor input (digital)
const int pingPinF  = 53;
const int cdsPin  = 0;     //pin reserved for photoresistor input (analog)

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
int speed = 70;       	//speed in PWM format [0 - 255]
int mode  = 0;
float setPoint;
float leftFront;
float rightFront;

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
	Serial.println("- - - Leader Robot - - -");
  	Serial.println("   Enter 0 for Follower Robot");
	Serial.println("   Enter s,S to stop all movement");  
	//make sure the robot is stopped
	robo.stop();
	robo.setSpeed(speed); //set speed for turning/driving
  robo.swapWheels(); //we'll be driving 'backwards', toward the little wheel.
  robo.swapDirectionPins();
	delay(2000);	//give some time to put the robot down after reset
 }

void loop(){
  static int status = 0;
  switch (mode) {
    case -1:
      robo.stop();
      break;
    case 0: //calibrate & initialize
      setPoint = 77;
      //reset the PID
      //PIDcalculate(setPoint, 1);
      //tell the others to get redy (turn on lights?)
      //have the Robot start      
        robo.setSpeed(150);
        robo.drive();         //xx//robo.drive(speed);
        delay(350);
        speed = 55;
        robo.setSpeed(speed);
      mode += 1;//findCenter();
      break;
    case 1: //go straight
      status = centerLine();
      if (status != 0)
        mode = 20;
      break;
    case 20: // corner 
      //assume we are turning left
      Serial.println("Turning");
      robo.drive(30, 85);          //xx//robo.drive(0, speed)
      delay(2750);                //xx//delay(2500);
      mode += 1;
      break;
    case 21:  //drive straight to find next wall
      Serial.println("Continuing");
      robo.setSpeed(speed);
      robo.drive();
      delay(200);
      leftFront = robo.IRdistance(irPinLF);
      rightFront = robo.IRdistance(irPinLR);
      if (leftFront > 0 && leftFront < 100 && rightFront > 0 && rightFront <100){
        mode = 1;
        PIDcalculate(setPoint, 1);
      }
      break;
    case 22: //rotate until we find the next wall
      //rotate until we are centered again
      //once we're centered, drive straight again
      break;
    default:
      Serial.println("invalid mode selected");
  }
}

/* --------------- calibrateWidth -----------
 * gets the width of the hall from the two IR sensors
 * esitmates hall width based on sensor readings
 * returns the width in meters
 */
int findCenter(){
  static int last_diff;
  //get sensors
  int front = robo.IRdistance_mm(irPinLF);
  int rear  = robo.IRdistance_mm(irPinLR);
  //if we are outside of the acceptible 'parallel' range
  int difference = 100*abs(front - rear)/max(front, rear);  //percentage difference in the two readings
  Serial.print("F "); Serial.print(front); 
  Serial.print(" \tR "); Serial.print(rear);
  Serial.print(" \tD "); Serial.println(difference);
  if (difference > 2 ){
    int dir = rear - front; //if positive we want to rotate CW
    dir = max(-1, dir); //set floor
    dir = min( 1, dir); //set ceiling
    robo.pivot(dir, 90);      //pivot based on reading
  } else {
    Serial.println("AT ZERO");
    if (last_diff == difference) {
      robo.stop();
      int leftDist = front * 0.966;
      int rightDist = robo.IRdistance_mm(irPinR);
      setPoint = 76;//(rightDist+leftDist)*0.05;
      Serial.print("leftDist "); Serial.print(leftDist);
      Serial.print("riteDist "); Serial.print(rightDist);
      Serial.print("setPoint "); Serial.print(setPoint);
      return 1;
    }   
  }
  last_diff = difference;
  return 0;
  //rotate toward shorter until they are within 5% of each other
}
int centerLine(){
  const  int   numReadings=5;
  const  int   MaxErr = 100; //the highest error value expected
  static float readings_f[numReadings];
  static float average_f = 0;
  static float sum_f = 0;
  static float readings_r[numReadings];
  static float average_r = 0;
  static float sum_r = 0;
  static float last_reading = 0;
  static int   pointer = 0;
  static int   first_call = 1; //to initialize the static arrays

  //initialize the arrays
  if (first_call = 1) {
    for (int i = 0; i < numReadings; ++i)  {
      readings_f[i] = 0;
      readings_r[i] = 0;
    }
    first_call = 0;
  }
  //update sum
  //sum_f -= readings_f[pointer];
  //sum_r -= readings_r[pointer];
  //get sensor readings
  //readings_f[pointer] = robo.IRdistance(irPinLF);
  //readings_r[pointer] = robo.IRdistance(irPinLR);
  //update sum
  //sum_f += readings_f[pointer];
  //sum_r += readings_r[pointer];
  //figure out if we found a corner
  //if the left reading jumped, we'll want to turn left
  //if (readings_f[pointer] > average_f*1.5) //50% greater than the average
    //return -1;
  //if the right reading jumped we'll want to turn right
  //if (readings_r[pointer] > average_r*1.5) 
    //return 1;
  //update averages
  average_f = robo.IRdistance(irPinLF)*0.96; // = sum_f/numReadings;
  average_r = robo.IRdistance(irPinLR); // = sum_f/numReadings;
  //turn according to the average
  float distance;
  if (average_r != 0 && average_f != 0)
    distance = ( average_f + average_r ) * 0.485;
  else 
    distance = max(average_f, average_r) * 0.970;
  //corner detection:
  if (average_f == 0){
    return 1;
  }
  float error = setPoint - distance;
  float diff = PIDcalculate(error); //negative values mean too far right, turn left
  diff = min(diff, MaxErr); //limits error to max expected value
  diff = max(diff, -MaxErr);//limits error to min expected value
  diff = map(error, -MaxErr, MaxErr, -50, 50);
  robo.drive_dif((int)diff,speed);
    //print stuff
  Serial.print("DRIVING: F "); Serial.print(average_f);
  Serial.print(" \tR "); Serial.print(average_r);
  Serial.print(" \tD "); Serial.println(diff);
  return 0; //all systems normal
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
    Serial.print("Update speed, current : ");  Serial.println(speed);
    speed = Serial.parseInt();
    robo.setSpeed(speed);
    Serial.print("SPEED updated to : ");  Serial.println(speed);
  } else if (ch == 'c')  { //user selected wallfollower mode
    Serial.println("MODE select : ");
    mode = Serial.parseInt();
    robo.setSpeed(speed);
    Serial.print("MODE updated to : ");  Serial.println(mode);
  } else {
    Serial.print(ch);
    Serial.println(" : unrecognized command");
  }
 }

float PIDcalculate(float distance, int reset){
  //setup & tuning variables, 
  //below are for 300 RPM motors at 80/255 speed setting.
  const float Ku = 45;  //formerly 8.5
  const float Tu = 3400;
  const float Kp = Ku*0.65;
  const float Ki = Kp*0.4/Tu;
  const float Kd = Kp*Tu/3.4;
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
  float output;
  //Use PID function from Lab #6
  error = (float)(setPoint - distance);
  if (reset==1){
    errorSum = 0;
    lastError = error;
  }
  else
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
    Serial.print(" \t"); Serial.println(now); 
  }
  //
  return (output);
}