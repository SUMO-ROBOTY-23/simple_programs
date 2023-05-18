/*******************************************************************************
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTY AND SUPPORT
 * IS APPLICABLE TO THIS SOFTWARE IN ANY FORM. CYTRON TECHNOLOGIES SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 ********************************************************************************
 * DESCRIPTION:
 *
 * This example shows how to drive 2 motors using 4 PWM pins (2 for each motor)
 * with 2-channel motor driver.
 * 
 * 
 * CONNECTIONS:
 * 
 * Stm32 A1  - Motor Driver PWM D5 Input
 * Stm32 A2  - Motor Driver PWM D6 Input
 * Stm32 A5  - Motor Driver PWM D3 Input
 * Stm32 A6  - Motor Driver PWM D4 Input
 * Stm32 GND - Motor Driver GND
 *
 *
 * AUTHOR   : Kong Wai Weng
 * COMPANY  : Cytron Technologies Sdn Bhd
 * WEBSITE  : www.cytron.io
 * EMAIL    : support@cytron.io
 *
 *******************************************************************************/

 #include "CytronMotorDriver.h"

#include <IRremote.hpp>
#define IR_RECEIVE_PIN D11


#define MINUS 0x7
#define PLUS 0x15
#define CHM	69
#define CHP	71
#define ZERO 22

// Configure the motor driver.
CytronMD motor1(PWM_PWM, D5, D6);
CytronMD motor2(PWM_PWM, D3, D4);

int16_t speed1 = 0;
int16_t speed2 = 0;

// The setup routine runs once when you press reset.
void setup() {
  Serial.begin(9600);
  pinMode(IR_RECEIVE_PIN, INPUT);
  // initialize digital pin LED_BUILTIN as an output.
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  pinMode(D13, OUTPUT);
  digitalWrite(D13, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("Ready to read");
}


// The loop routine runs over and over again forever.
void loop() {
    int data = 0;
    if (IrReceiver.decode()) {
      data = IrReceiver.decodedIRData.command;

      // Serial.println(data);
      

      if (data == MINUS) {
        speed1 -= 1;
      } else if (data == PLUS) {
        speed1 += 1;
      } else if (data == ZERO) {
        speed1 = 0;
        speed2 = 0;
      } else if (data == CHP) {
        speed2 += 1;
      } else if (data == CHM) {
        speed2 -= 1;
      } else {
        Serial.println(data);
      }

      if (data == MINUS || data == PLUS || data == ZERO || data == CHM || data == CHP) {
        Serial.println("Speeds:");
        Serial.println(speed1);
        Serial.println(speed2);
        motor1.setSpeed(speed1);
        motor2.setSpeed(speed2);
      }        

      IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
    // IrReceiver.resume();
  }

      
      

    // for (int i = -255 / 2; i < 256 / 2; ++i){
    //   motor1.setSpeed(2 * i);
    //   delay(100);
    // }

    // for (int i = 256 / 2; i > -256 / 2; --i){
    //   motor1.setSpeed(2 * i);
    //   delay(100);
    // }
}


