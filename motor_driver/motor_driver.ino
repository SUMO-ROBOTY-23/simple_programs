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
 * Stm32 A1  - Motor Driver PWM 1A Input
 * Stm32 A2  - Motor Driver PWM 1B Input
 * Stm32 A5  - Motor Driver PWM 2A Input
 * Stm32 A6  - Motor Driver PWM 2B Input
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

// Configure the motor driver.
CytronMD motor1(PWM_PWM, A1, A2);
CytronMD motor2(PWM_PWM, A5, A6);

int speed = 0;

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
  // 
  
  int data;
    if (IrReceiver.decode()) {
      data = IrReceiver.decodedIRData.command;

      Serial.println(data);
      IrReceiver.resume();
  }


      if (data == MINUS) {
        speed -= 10;
      } else if (data == PLUS) {
        speed += 10;
      }

      if (speed >= 256) {
        speed = 250;
      } else if (speed <= -256) {
        speed = -250;
      }

      if (data == MINUS || data == PLUS) {
        Serial.println(speed);
        motor1.setSpeed(speed);
        IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
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


