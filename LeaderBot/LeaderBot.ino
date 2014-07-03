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

/*--- sensor pins ---*/
//const int irPinL = 14;    //pin reserved for Sharp IR input (analog)
const int irPinR 	= 15;    //front IR sensor  
const int pingPinR  = 52;  //pin reserved for ping sensor input (digital)
const int pingPinF  = 53;
const int cdsPin 	= 0;     //pin reserved for photoresistor input (analog)

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
int speed = 80;       	//speed in PWM format [0 - 255]
int mode  = -1;
float setPoint;

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
  robo.switchDirection(); //we'll be driving 'backwards', toward the little wheel.
	if (DB) Serial.println(headers);
	delay(2000);	//give some time to put the robot down after reset
 }

void loop(){
  static int status = 0;
  switch (mode) {
    case 0: //calibrate
      //figure out how wide the hall is
      float hall_width = calibrateWidth();
      //set the setPoint for the centerline
      setPoint = hall_width/2*1.41421;
      //tell the others to get redy (turn on lights?)
      //have the Robot start
      mode = 1;
      break;
    case 1: //go straight
      status = centerline();
      if (status != 0)
        mode = 20;
    case 20: // corner approaching
      //log which way we are going to turn (toward the larger value)
      //if the values are more than 10% off, keep driving straight with the lower value
      //once they get close, switch mode to 21
    case 21:  //time to turn
      //stop & pivot toward the turn direction until we pick up the next wall
      //hopefully we're still close to the center
    case 22: //rotate until we find the next wall
      //rotate until we are centered again
      //once we're centered, drive straight again
    case default:
      Serial.println("invalid mode selected");
  }
}

/* --------------- calibrateWidth -----------
 * gets the width of the hall from the two IR sensors
 * esitmates hall width based on sensor readings
 * returns the width in meters
 */
float calibrateWidth(){
  float low_left  = 1000;    //lowest reading from the left sensor
  float low_right = 1000;    //lowest reading from the right sensor  
  float return_dest;
  //read the front 45-degree sensors
  //turn ccw until the right sensor reading goes to minimum and starts to go back up.
  return_dest = robo.IRdistance(irPinR)
  low_right = turn_until(irPinR, -1);
  //rotate back to center - should be 90 degrees ccw
  return_dest = turn_until(irPinR, return_dest);
  //turn cw until the left reading bottoms out
  low_left = turn_until(irPinL, -1);
  //turn the robot to a straight facing position 
  return_dest = low_left * 1.41421; //1.41421 = l/cos(45)
  return_dest = turn_until(irPinL, return_dest);
  //return the sum of those two readings as the hallway width
  float width = low_right + low_left + 22.8;  //each sensor is 11.4 cm from the center
}

float turn_until(int sensorPin, float stop_reading){
  float last_reading = 1000; //last reading recieved
  float current_reading = 1000;        //current reading of interest

  do {
    robo.turn(-1);

    if (stopReading == -1)
      last_reading = current_reading;
    else
      last_reading = stop_reading;

    current_reading = robo.IRdistance(sensorPin);

    if (current_reading < low_reading)
      low_reading = current_reading;
    if (DB) {
      Serial.print("last: ");   Serial.print(last_reading);
      Serial.print("\tcurr: "); Serial.print(current_reading);
      Serial.print("\tlow: ");  Serial.println(low_reading);
    }
  } while(current_reading <= last_reading);
  return low_reading;
 }

int centerLine(){
  const  int   numReadings
  static float readings_l[numReadings]={0,0,0,0,0};
  static float average_l = 0;
  static float sum_l = 0;
  static float readings_r[numReadings]={0,0,0,0,0};
  static float average_r = 0;
  static float sum_r = 0;
  static float last_reading = 0;
  static int   pointer = 0;

  //
  sum_l -= readings_l[pointer];
  sum_r -= readings_r[pointer];
  //get sensor readings
  readings_l[pointer] = robo.IRdistance(irPinL);
  readings_r[pointer] = robo.IRdistance(irPinR);
  //
  sum_l += readings_l[pointer];
  sum_r += readings_r[pointer];

  //figure out if we found a corner
  //if the left reading jumped, we'll want to turn left
  if (readings_l[pointer] > average_l*1.5) //50% greater than the average
    return -1;
  //if the right reading jumped we'll want to turn right
  if (readings_r[pointer] > average_r*1.5) 
    return 1;
  //update averages
  average_l = sum_l/numReadings;
  average_r = sum_r/numReadings;
  //turn according to the average
  int error = average_r - average_l;
  diff = map(error, -setPoint*2, setPoint*2, -20, 20);
  robo.drive_dif(diff);
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
