# pHulkBuggy
This is the readme file for the HulkBuggy, the second version of the self drive project.

## Pinout and connections
The following pinouts are being used in this project
  A0 is echoPin for Ultrasonic sensor
  A1 is trigPin for Ultrsonic sensor
  A2 is pin for servo
  A4 is SDA
  A5 is SCL

  2 is interupt pin
  3 is enA1
  4 is horn
  5 is enA2
  6 is enB2 
  9 is enB1
  13 is Led_pin

   NOTE: In addition to connection 5v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno this is digital I/O pin 2.

## The program
The following steps are being preformed at **setup**:
1. Start serial out @ 9600 and print splash screen
2. Start I2C bus
3. General confiurgations 
    - MPU
    - LED
    - Utrasonic
    - H-Bridge
    - RF24 Radio
    - Servo

The following steps are being preformed in the **loop** of the program:
1. Manual drive logic
2. Self drive logic

Unit tests are available to test various senarios in the code. For running unit test set **UnitTest == True**

# Tank TurnMovement Calculations
Get X and Y from the Joystick, do whatever scaling and calibrating you need to do based on your hardware.
Invert X
Calculate R+L (Call it V): V=(100-ABS(X)) * (Y/100) + Y
Calculate R-L (Call it W): W= (100-ABS(Y)) * (X/100) + X
Calculate R: R = (V+W) /2
Calculate L: L= (V-W)/2
Do any scaling on R and L your hardware may require.
Send those values to your Robot.
Go back to 1.

# GIT notes
https://git-scm.com/book/en/v2/Git-Branching-Basic-Branching-and-Merging

  Switched to branch 'master'
  git merge <branch>

  Delete the branch
    git branch -d <branch>

https://nvie.com/posts/a-successful-git-branching-model/


# MARKDOWN notes
https://www.markdownguide.org/cheat-sheet/

# Useful links
  **Changing floats to strings** https://www.programmingelectronics.com/dtostrf/
  **Insert variables to strings** https://www.programmingelectronics.com/sprintf-arduino/
  **LCD display** coding https://docs.arduino.cc/learn/electronics/lcd-displays#custom-character
  **Special characters** https://maxpromer.github.io/LCD-Character-Creator/
  **Arduino variable** https://roboticsbackend.com/arduino-variable-types-complete-guide/
                      https://randomnerdtutorials.com/
  **Tank joystick** https://home.kendra.com/mauser/joystick.html
  **MPU6050** https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/