/* ================================================================
  Name:       pHulkBuggy.ino
  Created:    2022/07/31
  Author:     Spoorstaaf

  This is the new code to the HulkBuggy which is a followup on AutoDrive.ino

  PINOUTS DESCRIPTION
  A0 is echoPin for Ultrasonic sensor
  A1 is trigPin for Ultrsonic sensor
  A2 is pin for servo
  A4 is SDA
  A5 is SCL

  2 is interupt pin
  3 is enB1
  4 is horn
  5 is enB2 
  6 is enA1
  9 is enA2
  13 is Led_pin

   NOTE: In addition to connection 5v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno this is digital I/O pin 2.

// ==============================================================*/

/* ================================================================
// ===                 PROJECT NAME AND VERSION                 ===
// ==============================================================*/
const char *p_project = "HulkBuggy v2";   // add project name
const uint8_t version_hi = 2;             // hi version
const uint8_t version_lo = 0;             // low version
const long version_date = 20220731;       // revision date


// ================================================================
// ===               INCLUDE FOLLOWING LIBRARIES                ===
// ================================================================
#include <LiquidCrystal_I2C.h>  // include the I2C LED screen library
//#include <Servo.h>              // include the Servo library
#include <NewPing.h>            // ultrasonic sensor function library
#include <I2Cdev.h>
//#include <MPU6050.h>
//#include "MPU6050_6Axis_MotionApps20.h"
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
/*#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif*/


// ================================================================
// ===                    PIN CONNECTIONS                       ===
// ================================================================

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno
#define LED_PIN 13       // Arduino is 13
#define trigPin A1       // define the sensor trigger pin
#define echoPin A0       // define the sensor echo pin
#define servoPin A2      // declare the Servo pin
#define enA1 3
#define enA2 5
#define enB1 9
#define enB2 6
#define Horn 4
bool blinkState = false;  // note used


// ================================================================
// ===                      GEN VARIABLES                       ===
// ================================================================
// General variables
const int BAUD_RATE PROGMEM = 9600;

// Ultrasonic sensor variables
int distance = 0;       // distance measured by sensor
int LongDistance = 0;   // this is the longest distance measured
int ReadDistance[10] = { 0, 0, 0, 0, 0 };     // range for each distance along each angle
#define maximum_distance 500                  // maximun range for sensor ping function
int SSD = 45;                 // declare the safe stop distance

// Servo variables
int j = 30;             // delay following servo movement
int LongAngle = 0;      // this is angle at the longest distance measured
int SetAngle[10] = { 10, 45, 90, 135, 170 };  // range on angles to be measured
signed int TurningAngle = 0;      // the value of the angular change required for car to follow longest path

// MPU6050 variables
int angle;     // angle on MPU
bool FirstAngle = true;       // boolean to determine if this is the first angle
int FirstAngleValue = 0;      // first angle from MPU, before user movements of MPU

// Stages variables
int StageDelay = 2000;        // delay between stages
bool Observing = false;       // boolean for whether or not the vehicle is observing
bool Turning = false;         // boolean for whether or not the vehicle is turning
bool Driving = false;         // boolean for whether or not the vehicle is driving
bool SelfDriveMode = false;   // Is the car in self drive or not
bool UnitTest = false;         // Create unit test section


// LCD variables
long LCD_last_update = 0;
long LCD_update_cycle = 1000;
long LCD_update_scale = 1;
byte connectedChar[] = {  B01110,  B01110,  B01110,  B00100,  B00100,  B00100,  B00100,  B00100};
byte not_connectedChar[] = {  B10101,  B01110,  B01110,  B10101,  B00100,  B00100,  B00100,  B00100};
byte manualChar[] = {  B10001,  B11011,  B11111,  B11111,  B10101,  B10101,  B10001,  B10001};
byte posChar[] = {  B00000,  B00000,  B00100,  B01010,  B10001,  B00100,  B01010,  B10001};
byte clear[] = {  B00000,  B00000,  B00000,  B00000,  B00000,  B00000, B00000,  B00000};
byte negChar[] = {  B10001,  B01010,  B00100,  B10001,  B01010,  B00100,  B00000, B00000};
byte UltraChar[] = {  B00000,  B10000,  B11110,  B11110,  B11110,  B11110,  B10000, B00000};
byte hornChar[] = {  B00110,  B01000,  B10010,  B10101,  B10101,  B10010,  B01000,  B00110};

// ================================================================
// ===                    CREATE OBJECTS                        ===
// ================================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
//Servo Servo1;                                       //servo object
NewPing sonar(trigPin, echoPin, maximum_distance);  //sensor function
//MPU6050 mpu;                                        // AD0 low = 0x68 (default)
MPU6050 mpu6050(Wire);
RF24 radio(7, 8);                                   // CE, CSN
const byte address[6] = "00001";

// ================================================================
// ===              H-BRIDGE AND STATUS VARIABLES               ===
// ================================================================

word xAxis = 512, yAxis = 512, SW = 1;
word receivedData[3] = { xAxis, yAxis, SW };

signed int xAxis100, yAxis100;
signed int xAxis100_inv;
float V, W, R, L;
int motorSpeedA = 0;
int motorSpeedB = 0;

int CountZero = 0;
int FirstZero = 0;

// ================================================================
// ===             MPU CONTROL AND STATUS VARIABLES             ===
// ================================================================

//NOT BROUGHT OVER FROM VERSION 1 YET



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

//NOT BROUGHT OVER FROM VERSION 1 YET



// ================================================================
// ===                    PROJECT INFORMATION                   ===
// ================================================================
void splash(void)
{
  Serial.print(F("Spoorstaaf: "));
  Serial.print(p_project);
  Serial.print(F(" V"));
  Serial.print(version_hi);
  Serial.print(F("."));
  Serial.print(version_lo);
  Serial.print(F(" "));
  Serial.println(version_date);

  Serial.println();
  Serial.println();
  Serial.println("=======================================================");
  Serial.println("===        WELCOME TO THE HULK BUGGY PROJECT        ===");
  Serial.println("===                  BY SPOORSTAAF                  ===");
  Serial.println("===                   16/05/2023                    ===");
  Serial.println("=======================================================");
  Serial.println();
  Serial.println();

  /*char buffer[40];
  sprintf(buffer, "The %d burritos are %s degrees F", 3.21, "hhh");
  Serial.println(buffer);*/
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  /* ==========================GENERAL INFORMATION==================================
     The following steps are being preformed at setup:
     1. Start serial out @ 115200 and print splash screen
     2. Start I2C bus
     3. General confiurgations 
		 	-	LCD
      - MPU
      - LED
      - Utrasonic
      - H-Bridge
      - RF24 Radio
      - Servo
     =============================================================================*/ 

  /* =============================================================================== 
     SETUP 1. Start serial out and print splash screen
  // initialize serial communication (115200 chosen because it is required for Teapot 
  // demo output, but it's really up to you depending on the project)
     =============================================================================*/ 
  Serial.begin(BAUD_RATE);          // setup serial output
  Serial.println("... Start program");
  while (!Serial)
    ;  // wait for Leonardo enumeration, others continue immediately

  splash();                         // splash project info to serial

  /* =============================================================================== 
     SETUP 2. Start I2C bus
  // join I2C bus (I2Cdev library doesn't do this automatically)    
     =============================================================================*/ 
  Serial.println(F("Initializing I2C devices..."));
  Wire.begin();
  Wire.setClock(400000);

  /*
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  */
  /* =============================================================================== 
     SETUP 3. General confirugations (MPU, LED, Utrasonic, H-Bridge, Radio, Servo)
     =============================================================================*/ 
	Serial.println(F("General configuration..."));
	lcd.init();                    
  lcd.backlight();
  lcd.setCursor(0,0);

	String msg = String(p_project); //p_project + F(" V") + version_hi + F(".") + version_lo + F(" ") + version_date;
	//lcd_scroll(msg, 50);

  //create special characters
  lcd.createChar(0, connectedChar);
  lcd.createChar(1, not_connectedChar);
  lcd.createChar(2, UltraChar);
  lcd.createChar(3, posChar);
  lcd.createChar(4, clear);
  lcd.createChar(5, negChar);
  lcd.createChar(6, hornChar);

	lcd.clear();
  lcd.home();
  lcd.write(1);
  lcd.setCursor(6,0);
  lcd.write(2);

  Serial.println("  ... LCD set");

  // ============  MPU6050  ============
  //NOT BROUGHT OVER FROM VERSION 1 YET
  
  //mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);
  //mpu6050.getAngleZ();
  /* new library*/

  // ============  LED  ============/ 
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("  ... LED pin set");

  // ============  Ultrasonic sensor  ============ 
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
  Serial.println("  ... Ultrasonic sensor set");

  // ============  H-Bridge  ============/ 
  pinMode(enA1, OUTPUT);
  pinMode(enA2, OUTPUT);
  pinMode(enB1, OUTPUT);
  pinMode(enB2, OUTPUT);
  pinMode(Horn, OUTPUT);
  Serial.println("  ... H-Bridge set");

  // ============  RF24 Radio  ============ 
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("  ... RF communication now started");
  delay(500);

  // ============  Servo  ============ 
  //Servo1.attach(servoPin);
  //move_servo(90, j);  // the starting point is at 90 degrees
  Serial.println("  ... Servo now at 90");
  delay(500);


}





// ================================================================
// ===                        MAIN LOOP                         ===
// ================================================================
void loop() {
  /* ==========================GENERAL INFORMATION==================================
     The following steps are being preformed in the loop of the program:
     0. Unit test function
     1. Manual drive logic
     2. Self drive logic
     =============================================================================*/ 

    if(UnitTest){
      //Serial.println("UTest"); 

      xAxis = 510;
      yAxis = 510;
      
      while (Serial.available() > 0) {
        // read the incoming byte:
        int incoming = Serial.parseInt();

        // say what you got:
        Serial.println();
        Serial.print("Selection: ");
        Serial.println(incoming);

        if(incoming == 1){
          Serial.println(" - Moving forward");
          drive_function(561, 1024);
          delay(2000);
          drive_function(510, 510);
        }
        else if(incoming == 2){
          Serial.println(" - Moving backward");
          drive_function(561, 0);
          delay(2000);
          drive_function(510, 510);
        }      
        else if(incoming == 3){
          Serial.println(" - Tenk turn left");
          drive_function(1024, 547);
          delay(2000);
          drive_function(510, 510);
        }
        else if(incoming == 4){
          Serial.println(" - Tenk turn right");
          drive_function(0, 547);
          delay(2000);
          drive_function(510, 510);
        }
        else if(incoming == 5){
          Serial.println(" - Turn left");
          drive_function(1024, 1024);
          delay(2000);
          drive_function(510, 510);
        }
        else if(incoming == 6){
          Serial.println(" - Turn right");
          drive_function(0, 0);
          delay(2000);
          drive_function(510, 510);
        }

        else if(incoming == 7){
          Serial.println(" - Moving Pin 9");
          set_motor_speeds_unit_test(255, enA1, 2000);
        }
        else if(incoming == 8){
          Serial.println(" - Moving Pin 6");
          set_motor_speeds_unit_test(255, enA2, 2000);
        }
        else if(incoming == 9){
          Serial.println(" - Moving Pin 3");
          set_motor_speeds_unit_test(255, enB1, 2000);
        }
        else if(incoming == 10){
          Serial.println(" - Moving Pin 5");
          set_motor_speeds_unit_test(255, enA1, 2000);
        }
        else if(incoming == 11){
          Serial.println(" -Variable speed test");
          set_motor_speeds_unit_test(255/4, enA1, 2000);
          set_motor_speeds_unit_test(255/2, enA1, 2000);
          set_motor_speeds_unit_test(255*3/4, enA1, 2000);
          set_motor_speeds_unit_test(255, enA1, 2000);
          
          set_motor_speeds_unit_test(255/4, enA2, 2000);
          set_motor_speeds_unit_test(255/2, enA2, 2000);
          set_motor_speeds_unit_test(255*3/4, enA2, 2000);
          set_motor_speeds_unit_test(255, enA2, 2000);

          set_motor_speeds_unit_test(255/4, enB1, 2000);
          set_motor_speeds_unit_test(255/2, enB1, 2000);
          set_motor_speeds_unit_test(255*3/4, enB1, 2000);
          set_motor_speeds_unit_test(255, enB1, 2000);
          
          set_motor_speeds_unit_test(255/4, enB2, 2000);
          set_motor_speeds_unit_test(255/2, enB2, 2000);
          set_motor_speeds_unit_test(255*3/4, enB2, 2000);
          set_motor_speeds_unit_test(255, enB2, 2000);
        }
        else if(incoming == 12){
          Serial.println(" -LED test");
          set_motor_speeds_unit_test(255, enA1, 2000);
          set_motor_speeds_unit_test(255, enA2, 2000);
          set_motor_speeds_unit_test(255, enB1, 2000);
          set_motor_speeds_unit_test(255, enB2, 2000);
        }

      }
    }

    if(!UnitTest){
      Serial.print("SD=");
      Serial.print(SelfDriveMode);

      if (!SelfDriveMode) {
        /* =============================================================================== 
        1. Manual drive logic
        // manual drive using the RF module joystick
        // during manual drive, the car MPU is not working >>>
        // SelfDriveMode = false; Observing = false;  Turning = false;  Driving = false
        =============================================================================*/
        lcd.setCursor(0,1);
        lcd.print("M");
        // listern for a new RF24 instruction
        listern_to_RF24();

        Serial.print("\t X=");
        Serial.print(xAxis);
        Serial.print("\t Y=");
        Serial.print(yAxis);

        // call tank model driving function based on the selection of xAxis, yAxis
        drive_function(xAxis, yAxis);

        // measure the ultrasonic distance and show on LCD
        distance = readPing();

        // sound horn or activate/deactivate selfdrive
        activate_the_horn();  

        Serial.println();
      }

    /* =============================================================================== 
      2. Self drive logic


      =============================================================================*/ 
    if (SelfDriveMode) {
      // SELF DRIVE LOGIC
      // ==========================STAGE 1 OBSERVING====================================
      // 1. first observe environment in a range by scanning around and measure distances,
      //    and determine longest pathway.
      // 2. turn through the angle that was determined.
      // 3. drive until too close to object.
      // ===============================================================================


      if (Observing) {
        // SelfDriveMode = true; Observing = true;  Turning = false;  Driving = false

      }

      if (Turning) {
        // SelfDriveMode = true; Observing = false;  Turning = true;  Driving = false


        //mpu6050.update();
        //start_angle = mpu6050.getAngleZ()

      }

      if (Driving) {
        // SelfDriveMode = true; Observing = false;  Turning = false;  Driving = true

      }

    } 
  }
}





// ================================================================
// ===                    GENERAL SUB PROGRAMS                  ===
// ================================================================

// ============  SERVO SUB PROGRAMS  ============
void move_servo(int a, int b) {
  // Make servo go to a degrees and apply b delay to the system
  //Servo1.write(a);
  delay(b);
}

// ============  ULTRASONIC SUB PROGRAMS  ============
int readPing() {
  delay(50);
  int cm = sonar.ping_cm();
  String cm_text = "";

  if (cm == 0) {
    cm = 0;
  }

  if (cm < 100) {
    cm_text = " " + String(cm);
  }
  else {
    cm_text = String(cm);
  }

  Serial.print("  ULTDIST=");
  Serial.print(cm_text);
  Serial.print("\t");

  if (millis() - LCD_last_update  >= (LCD_update_cycle * LCD_update_scale)) {
    LCD_last_update = millis();
    lcd.setCursor(7,0);
    lcd.print(cm_text);
  }

  return cm;
}


// ============  RF24 SUB PROGRAMS  ============
void listern_to_RF24() {
  lcd.setCursor(0,0);
  if (radio.available())  // If the NRF240L01 module received data
  {
		lcd.write(0);
    Serial.print("\t RF=1");
    radio.read(&receivedData, sizeof(receivedData));  // Read the data and put it into character array
    if (!SelfDriveMode) {
      xAxis = receivedData[0];
      yAxis = receivedData[1];
    }
    SW = receivedData[2];
    delay(10);
  }
  else{
    lcd.write(1);
    Serial.print("\t RF=0");
  }
}


void activate_the_horn() {
  if (SW == 0) {
    digitalWrite(Horn, HIGH);
    lcd.setCursor(11,0);
    lcd.write(6);
    Serial.print("\t HORN=1");
    CountZero++;
  } 
  else {
    digitalWrite(Horn, LOW);
    lcd.setCursor(11,0);
    lcd.print("-");
    Serial.print("\t HORN=0");
    if (millis() - FirstZero > 1000) {
      CountZero = 0;
      FirstZero = millis();
    }
  }
  // set up count which will determine for what period the joystick neends to be pressed before mode will change
  if (CountZero > 20000) {
    Observing = !Observing;
    SelfDriveMode = !SelfDriveMode;
    CountZero = 0;
    digitalWrite(Horn, LOW);
  }
}



// ============  H-BRIDGE SUB PROGRAMS  ============
void drive_function(int xAxis, int yAxis) {
  // top of the spectrum seems to be unstable
  if (xAxis > 1020) { xAxis == 1020; }
  if (yAxis > 1020) { yAxis == 1020; }

  // Step 1: map xAxis and yAxis to values between -100 and 100
  xAxis100 = map(xAxis, 0, 1020, -100, 100);
  yAxis100 = map(yAxis, 0, 1020, -100, 100);
  // Step 2: invert X
  xAxis100_inv = -xAxis100;
  // Step 3: V =(100-ABS(X)) * (Y/100) + Y
  V = (100 - abs(xAxis100_inv)) * (yAxis100 / 100) + yAxis100;
  // Step 4: W =(100-ABS(Y)) * (X/100) + X
  W = (100 - abs(yAxis100)) * (xAxis100_inv / 100) + xAxis100_inv;
  // Step 5 : Calculate R: R = (V+W) /2
  R = (V + W) / 2;
  // Step 6 : Calculate L: L = (V-W) /2
  L = (V - W) / 2;

  Serial.print("\t V=");
  Serial.print(V);
	Serial.print("  W=");
  Serial.print(W);

  Serial.print("  R=");
  Serial.print(R);
	Serial.print("  L=");
  Serial.print(L);

  set_motor_speeds(R, enA1, enA2, 2);
  set_motor_speeds(L, enB1, enB2, 4);

  delay(10);
}


void set_motor_speeds(int calc_speed, uint8_t pin1, uint8_t pin2, int side){
  int pin1_speed = 0;
  int pin2_speed = 0;
  if (calc_speed < -10) {
    pin2_speed = map(calc_speed, -10, -100, 20, 255);
    lcd.setCursor(side,0);
    lcd.write(4);     //clear block
    lcd.setCursor(side,1);
    lcd.write(5);
  } else if (calc_speed > 10) {
    pin1_speed = map(calc_speed, 10, 100, 20, 255);
    lcd.setCursor(side,0);
    lcd.write(3);
    lcd.setCursor(side,1);
    lcd.write(4);     //clear block
  } else {
    lcd.setCursor(side,0);
    lcd.write(4);     //clear block
    lcd.setCursor(side,1);
    lcd.write(4);     //clear block
  }

  Serial.print("\t p");
  Serial.print(pin1);
  Serial.print("=");
  Serial.print(pin1_speed);
  Serial.print("\t p");
  Serial.print(pin2);
  Serial.print("=");
  Serial.print(pin2_speed);

  analogWrite(pin1, pin1_speed);  // Send PWM signal to motor
  analogWrite(pin2, pin2_speed);  // Send PWM signal to motor
}

void set_motor_speeds_unit_test(int pin_speed, uint8_t pin, int pin_delay){

  Serial.print("\t p");
  Serial.print(pin);
  Serial.print("=");
  Serial.println(pin_speed);
  analogWrite(pin, pin_speed);  // Send PWM signal to motor
  delay(pin_delay);
  analogWrite(pin, 0);  // Send PWM signal to motor to stop
}






/*
void motor_speed_A_function(int a, int b) {
  analogWrite(enA1, a);  // Send PWM signal to motor A
  analogWrite(enA2, b);  // Send PWM signal to motor A
  analogWr
}

void motor_speed_B_function(int a, int b) {
  analogWrite(enB1, a);  // Send PWM signal to motor B
  analogWrite(enB2, b);  // Send PWM signal to motor B
}
*/

// ============  LCD SUB PROGRAMS  ============

void lcd_scroll(String txt, int t) {
	lcd.print(txt);
	int txt_length = txt.length();
	Serial.println(txt);
  // scroll 13 positions (string length) to the left
  // to move it offscreen left:
  for (int positionCounter = 0; positionCounter < txt_length; positionCounter++) {
    // scroll one position left:
    lcd.scrollDisplayLeft();
    // wait a bit:
    delay(t);
  }

  // scroll 29 positions (string length + display length) to the right
  // to move it offscreen right:
  for (int positionCounter = 0; positionCounter < (txt_length * 2) + 3 ; positionCounter++) {
    // scroll one position right:
    lcd.scrollDisplayRight();
    // wait a bit:
    delay(t);
  }

  // scroll 16 positions (display length + string length) to the left
  // to move it back to center:
  for (int positionCounter = 0; positionCounter < (txt_length + 3); positionCounter++) {
    // scroll one position left:
    lcd.scrollDisplayLeft();
    // wait a bit:
    delay(t);
  }

  // delay at the end of the full loop:
  delay(5 * t);

}

void lcd_update(int x, int y, int position_1, int position_2){

  int currentMillis = millis(); 
  if (currentMillis - LCD_last_update >= LCD_update_cycle){
    LCD_last_update = millis();
    lcd.setCursor(position_1,1);
    lcd.print(x);
    lcd.setCursor(position_2,1);
    lcd.print(y);
  }

}




