#include <Servo.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

//pins
//Linescan + servo
#define SI 3
#define CLOCK 2
#define AOUT 14
#define SERVO 5
//Motor
#define MOTOR_TOP 6
#define MOTOR_BOT 11
//Bluetooth
#define RX_PIN 0
#define TX_PIN 1
#define LED_PIN 13

//camera and line stuff
#define SATURATION_TIME 4500 //40 000 before, maybe change again?
#define NOISE 1
#define MAX_WIDTH 20
#define MIN_WIDTH 5

//servo rotaion limits
#define SMIN 60
#define SMAX 120
#define SMIDDLE 90

//pure pursuit geometry parameters
#define LOOK_AHEAD 60    //cm, distance from front axle
#define CAR_LENGTH 25  //cm
#define PIXEL_LENGTH  .28  //cm      

//debug
#define OG_DEBUG true
#define INDEX_PRINTING_DEBUG false
#define CAMERA_DEBUG true //filtered linescan data

//servo + linescan
Servo eric;
int scan[128];
int prevMiddle = 63;

//bluetooth setup
SoftwareSerial BTSerial(RX_PIN, TX_PIN);
boolean motorOn = false;
boolean shouldNL = true;

//bluetooth update vars
bool S_STATE;
bool P_STATE;
bool I_STATE;
bool D_STATE;
bool M_STATE;
int pidNum = 0;
LiquidCrystal_I2C lcd(0x3F, 16, 2);

//pid vars
float prevError = 0;
double dt;
unsigned long lastTime = 0;
unsigned long currentTime = 0;
//Look-ahead of camera from front axle to line: 55 mm
float Kp = 0.65;  //Original 0.65
float Kd = .25;  //Original 1
float Ki = 0;
double output = 0;
double Error = 0;
double dError = 0;
double iError = 0;
int motorDuty = 10;
int motorDutyPControl = 10;

void setup() {
  delay(1000); //aylmao, teensy sucks
  Serial.begin(9600);
  servoLineScanSetup();
  bluetoothSetup();
  lcdSetup();
}

void loop() {
  currentTime = millis(); //time at the beginning of the loop for PID
  dt = double(currentTime - lastTime); //difference in time

  chuck(); //linescan chuck to set exposure
  delayMicroseconds(SATURATION_TIME);
  readLinescan(); //read in linescan values

  //Apply filters to linescan data to get mid
  medianFilter(scan, 128);
  gradientFilter(scan, 128);
  int middle = findMiddle(scan, 128, NOISE);
  if (CAMERA_DEBUG) {
    //print final filtered linescan data
    Serial.print("Final: ");
    for (int i = 0; i < 128; i++) {
      Serial.print(scan[i]);
    }
    Serial.print(";\n");
  }

  //Set servo with middle + PID info
  //PIDControl(middle);

  purePursuit(middle);

  //Read in bluetooth and process input(kill switch, var changes)
  processBluetooth();
  
  //Go forward/brake according to kill switch
  if (motorOn) {
    forward(motorDutyPControl);
    //printInfo();
  }
  else {
    brake(motorDuty);
    printInfo();
  }
}

/*--------------------------------- Setup functions ---------------------------------*/
void servoLineScanSetup() {
  //setup servo
  eric.attach(SERVO);
  eric.write(SMIDDLE);
  //setup linescan
  pinMode(SI, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  digitalWrite(SI, LOW);
  digitalWrite(CLOCK, LOW);
}

void bluetoothSetup() {
  pinMode(LED_PIN, OUTPUT); //initialize digital pin as output
  digitalWrite(LED_PIN, HIGH); //turn on
  BTSerial.begin(9600);
}

void lcdSetup() {
  lcd.begin();
  lcd.backlight();
  delay(500);
  lcd.noBacklight(); //blink test
  delay(500);
  lcd.backlight();
  lcd.setCursor(4,0); 
  lcd.print("Waiting...");
  delay(1000);
}
/*-----------------------------------------------------------------------------------*/


/*--------------------------------------- Camera ------------------------------------*/
/* Read in the values on the linescan camera */
void readLinescan() {
  //Start linescan read
  digitalWrite(SI, HIGH);
  digitalWrite(CLOCK, HIGH);
  digitalWrite(SI, LOW);
  digitalWrite(CLOCK, LOW);
  for(int i = 0; i < 128; i++) {
    delayMicroseconds(20);
    readPixel(scan, i);
  }
}

void readPixel(int scan[], int pixel) {
  scan[pixel] = analogRead(AOUT);
  digitalWrite(CLOCK, HIGH);
  digitalWrite(CLOCK, LOW);
}

/* Used to start setting the camera exposure time */
void chuck(){
  digitalWrite(CLOCK, LOW);
  digitalWrite(SI, HIGH);
  digitalWrite(CLOCK, HIGH);
  digitalWrite(SI, LOW);
  digitalWrite(CLOCK, LOW);

 for (int j = 0; j < 128; j++) {
   digitalWrite(CLOCK, HIGH);
   digitalWrite(CLOCK, LOW);
 }
}
/*-----------------------------------------------------------------------------------*/

/*------------------------------- Filter functions ----------------------------------*/
int RANGE = 3; //set range here
int HALF = RANGE/2;

/* Filters the linescan data by changing values to the median within RANGE */
void medianFilter( int scan[], int length ) {
  int filtered[length];
  int temp[RANGE];
  for (int i = HALF; i < length - HALF; i++) {
    //copy over range number of values to temp (scan[i-HALF, i+HALF] to temp[])
    for (int j = 0; j < RANGE; j++) {
      temp[j] = scan[i - HALF + j];
    }
    //get median of the temp[] and put it in filtered array
    filtered[i] = getMedian(temp, RANGE);
  }
  //copy over filtered
  for (int i = HALF; i < length - HALF; i++) {
    scan[i] = filtered[i];
  }
  //the ends
  for(int i = 0; i < HALF; i++) {
    scan[i] = filtered[HALF];
  }
  for(int i = length-HALF; i < length; i++) {
    scan[i] = filtered[length-HALF-1];
  }
}

//sorts the temp array and returns middle index
int getMedian ( int array[], int length ) {
  int middle = length/2;
  sort(array, length);
  return array[middle];
}

//bubble sort for sorting temp array
void sort(int a[], int length) {
  for(int i=0; i<(length-1); i++) {
    for(int o=0; o<(length-(i+1)); o++) {
      if(a[o] > a[o+1]) {
          int t = a[o];
          a[o] = a[o+1];
          a[o+1] = t;
      }
    }
  }
}

/* Filters the linescan data by changing values to 0/1 based off of difference */
void gradientFilter(int a[], int length) {
  //CALCULATING DIFFERENCE ARRAY//
  int difference[length];
  //get differences
  for (int i = 2; i < length-2; i++) {
    difference[i] = a[i+1] - a[i]; 
  }
  difference[0] = 0;
  difference[1] = 0;
  difference[length-1] = 0;
  difference[length-2] = 0;
  difference[length-3] = 0;
  if (OG_DEBUG) {
    Serial.print("\nOG: ");
    for (int i = 0; i < length; i++) {
      Serial.print(a[i]);
      Serial.print(" ");
    }
    Serial.print("\nDIFF: ");
    for (int i = 0; i < length; i++) {
      Serial.print(difference[i]);
      Serial.print(" ");
    }
    Serial.print("\n");
  }
  //COPYING OVER DIFFERENCE ARRAY//
  for (int i = 0; i < length; i++) {
    a[i]  = difference[i];
  }
  int minDiff = a[0];
  int maxDiff = a[0];
  int minIndex = 0;
  int maxIndex = 0;
  for (int i = 2; i < length-2; i++) {
    if (a[i] < minDiff) {
      minDiff = a[i];
      minIndex = i;
    }
    else if (a[i] > maxDiff) {
      maxDiff = a[i];
      maxIndex = i;
    }
  }
  if (INDEX_PRINTING_DEBUG) {
    Serial.print("\nINDICES: ");
    Serial.print("MAXINDEX: ");
    Serial.print(maxIndex);
    Serial.print(" MININDEX: ");
    Serial.print(minIndex);
    int l = minIndex - maxIndex;
    Serial.print(" LENGTH: ");
    Serial.print(l);
  }
  //ERROR CHECKING: >MAX_WIDTH; <MIN_WIDTH; minIndex <= maxIndex
  //min index will have a higher index number
  //check for length
  if (minIndex <= maxIndex) {
    for (int i = 0; i < length; i++) {
      a[i] = 0;
    }
  }
  else if(minIndex - maxIndex > MAX_WIDTH) {
//    Serial.print("\nBIGGER THAN 20");
    for (int i = 0; i < length; i++) {
      a[i] = 0;
    }
  }
  else if (minIndex - maxIndex < MIN_WIDTH) {
//    Serial.print("\nLESS THAN 5");
    for (int i = 0; i < length; i++) {
      a[i] = 0;
    }
  }
  //passes cases
  //COPY OVER TO A[] WITH 0 AND 1's//
  else {
    for (int i = 0; i < length; i++) {
      if (i >= maxIndex && i <= minIndex) {
        a[i] = 1;
      }
      else {
        a[i] = 0;
      }
    }
  }
}

/* Filters for noise and returns the guess for mid of the line from data */
int findMiddle(int a[], int length, int noiseFilter) {
  int leftMost = -1;
  int rightMost = -2;
  int startCounter = 0;
  int endCounter = 0;
  boolean started = false;
  for (int i = 0; i < length; i++) {
    if (a[i] == 1) {
      startCounter++;
      endCounter = 0;
      if (startCounter > noiseFilter) {
        // set leftMost to current index - noise noiseFilter if not started yet
        if (started == false) {
          leftMost = i - noiseFilter;
          started = true;
        }
      }
      //say end of line is end if ending
      if (started) {
        if (i == length - 1) {
        rightMost = length - 1;
        }
      }
    }
    else {
      startCounter = 0;
      if (started) {
        //start counting for end
        endCounter++;
        //say end of line is end if ending
        if (i == length - 1) {
          rightMost = length - 1;
        }
        if (endCounter > noiseFilter) {
          rightMost = i - noiseFilter;
          started = false;
        }
      }
    }
  }
  int middle = (rightMost + leftMost)/2;
  return middle;
}
/*-----------------------------------------------------------------------------------*/

/*---------------------------- Motor + Servo functions ------------------------------*/
void PIDControl(int middle){
  //add offset in consideration of camera location relative to center
  int centerPos = middle;
  if(centerPos > 128){
    centerPos = 128;  
  }
  if(centerPos == -1){
    centerPos = prevMiddle;
  }
  Error = centerPos - 63; 
  dError = (Error - prevError)/dt;
  iError = (Error + prevError)*dt;
  // Output for the servo to control the turning
  output = -1*(Kp*Error + Kd*dError + Ki*iError);
  //center point of servo is 90
  double turnAngle = output + 90;
  if(turnAngle > 120) {
    turnAngle = 120;
  }
  else if(turnAngle < 60) {
    turnAngle = 60;
  }
  eric.write(turnAngle);
  prevError = Error; 
  lastTime = currentTime;
  prevMiddle = centerPos;


  

/*---------------------------- Proportional Speed Control ------------------------------*/

  motorDutyPControl = map(abs(turnAngle - 90), 0, 30, motorDuty + 5, motorDuty - 3);
}



void forward (int value) {
  int duty = 2.55*value;
  analogWrite(MOTOR_BOT, 0);
  //dead time to prevent short
  delayMicroseconds(5);
  analogWrite(MOTOR_TOP, duty);
}

void brake (int value) {
  int duty = 2.55*value;
  analogWrite(MOTOR_TOP, 0);
  //dead time to prevent short
  delayMicroseconds(5);
  analogWrite(MOTOR_BOT, duty);
}

void servoMiddle(int middle){
  if(middle == -1) {
    eric.write(SMIDDLE);
  }
  else{
    float ratio = ((128 - 2*NOISE)/87.0); 
    float pixelOffset = middle - 63; 
    float servoOffset = pixelOffset/ratio;
    eric.write(63+servoOffset);

    Serial.print(ratio);
    Serial.print(" : ");  
    Serial.print(pixelOffset);
    Serial.print(" : ");
    Serial.print(63+servoOffset);
    Serial.print(" : ");
  }
}
/*-----------------------------------------------------------------------------------*/

/*------------------------- Bluetooth + LCD functions -------------------------------*/
void processBluetooth() {
  //Output the bluetooth input onto serial monitor
  char charInput = ' ';
  if (BTSerial.available()) {
    charInput = BTSerial.read();
    Serial.write(charInput);

    //toggle motor
    if (charInput == 'e') {
      switchMotor();
    }

    //adjusting pid values and motorDuty
    if (!S_STATE & !motorOn) {
      //inc/dec pid
      if (charInput == 'p') {
        Kp += 0.01;
      }
      if (charInput == 'P') {
        Kp -= 0.01;
      }
      if (charInput == 'i') {
        Ki += 0.01;
      }
      if (charInput == 'I') {
        Ki -= 0.01;
      }
      if (charInput == 'd') {
        Kd += 0.01;
      }
      if (charInput == 'D') {
        Kd -= 0.01;
      }
      if (charInput == 'm') {
        motorDuty++;
      }
      if (charInput == 'M') {
        motorDuty--;
      }
    }

    //setting pid values
    if (charInput == 's') {
      S_STATE = true;
      pidNum = 0;
      P_STATE = false;
      I_STATE = false;
      D_STATE = false;
    }
    else if (S_STATE) {
      if (charInput == 'p') {
        Serial.print("P_STATE");
        P_STATE = true;
      }
      if (charInput == 'i') {
        Serial.print("I_STATE");
        I_STATE = true;
      }
      if (charInput == 'd') {
        Serial.print("D_STATE");
        D_STATE = true;
      }
      S_STATE = false;
    }
    
    if (P_STATE || I_STATE || D_STATE) {
      //add to pidNum
      if (charInput >= 48 && charInput <=57) {
        int intInput = charInput - '0';
        pidNum = pidNum*10 + intInput;
      }
      //set pid based on state to pidNum
      if (charInput == 13) {
        Serial.print("setting pid: ");
        Serial.print(pidNum);
        pidNum = pidNum/100;
        if (pidNum < 0) {
          pidNum = 0;
        }
        if (pidNum > 1) {
          pidNum = 1;
        }
        if (P_STATE) {
          Kp = pidNum;
        }
        if (I_STATE) {
          Ki = pidNum;
        }
        if (D_STATE) {
          Kd = pidNum;
        }
        P_STATE = false;
        I_STATE = false;
        D_STATE = false;
        pidNum = 0;
      }
    }

    if (charInput == 'x') {
      S_STATE = false;
      P_STATE = false;
      I_STATE = false;
      D_STATE = false;
      pidNum = 0;
    }
    
    if (BTSerial.available() == 0) { //NL if done reading
      Serial.write("\n");
    }
  }
}

void switchMotor() {
  if (motorOn) {
    digitalWrite(LED_PIN, LOW);
  }
  else {
    digitalWrite(LED_PIN, HIGH);
  }
  motorOn = !motorOn;
}

void printInfo()
{
  lcd.setCursor(0,0);
  lcd.print("Kp=");
  lcd.print(Kp);
  lcd.print(" Ki=");
  lcd.print(Ki);
  lcd.setCursor(0,1);
  lcd.print("Kd=");
  lcd.print(Kd);
  lcd.setCursor(8,1);
  lcd.print("M=");
  lcd.print(motorDuty);
}


void purePursuit(int middle){
  if (middle == -1) {
    middle = prevMiddle;
  }
    double delta = getAngle(middle);   
    double pos = SMIDDLE+delta;
    if(pos<SMIN){
      pos = SMIN;
    }   
    if(pos>SMAX){
      pos = SMAX; 
    }
    eric.write(pos); 
    //Serial.print(pos);
    //Serial.print(" : ");
    prevMiddle = middle; 

}

double getAngle(int middle){
  int pixelOffset = middle - 63;
   //Serial.print(pixelOffset);
   //Serial.print(" : ");
  double x = pixelOffset * PIXEL_LENGTH;//scaled pixel offset
  //Serial.print(x);
  //Serial.print(" : ");
  double hypotenuse = sqrt(pow(x,2) + pow(LOOK_AHEAD+CAR_LENGTH,2));
  //Serial.print(hypotenuse);
  //Serial.print(" : ");
  double delta = atan2(((2*CAR_LENGTH))*(x/hypotenuse), (CAR_LENGTH + LOOK_AHEAD))*(180/PI);
  //Serial.print(delta);
    //Serial.print(" :\n ");
  return 15*Kp*-delta;
}

// LOOK_AHEAD 60    //cm, distance from front axle
// CAR_LENGTH 25  //cm
// PIXEL_LENGTH  .28  //cm  
/*-----------------------------------------------------------------------------------*/
