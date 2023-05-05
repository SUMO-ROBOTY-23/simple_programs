
#include <IRremote.hpp>
#define IR_RECEIVE_PIN D6

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  pinMode(IR_RECEIVE_PIN, INPUT);
  // initialize digital pin LED_BUILTIN as an output.
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  pinMode(D13, OUTPUT);
  digitalWrite(D13, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("Ready to read");
  
}

// the loop function runs over and over again forever
void loop() {
    
    if (IrReceiver.decode()) {
      Serial.println(IrReceiver.decodedIRData.decodedRawData); // Print "old" raw data
      Serial.println("OK");
      IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
      IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data
      IrReceiver.resume(); // Enable receiving of the next value
  }
}
