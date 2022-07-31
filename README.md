# pHulkBuggy
This is the readme file for the HulkBuggy, the second version of the self drive project.

## Pinout and connections
The following pinouts are being used in this project
  A0 is echoPin for Ultrasonic sensor
  A1 is trigPin for Ultrsonic sensor
  A2 is pin for servo
  A4 is SDA
  A5 is SDL

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

## The program

The following steps are being preformed at **setup**:
1. Start serial out @ 115200 and print splash screen
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





# Useful links
**Changing floats to strings** https://www.programmingelectronics.com/dtostrf/
**Insert variables to strings** https://www.programmingelectronics.com/sprintf-arduino/

# GIT notes
https://git-scm.com/book/en/v2/Git-Branching-Basic-Branching-and-Merging



# MARKDOWN notes
https://www.markdownguide.org/cheat-sheet/