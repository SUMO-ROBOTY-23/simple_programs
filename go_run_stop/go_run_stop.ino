#include <QTRSensors.h>
#include <IRremote.hpp>
#include "pilot_names.h"
#define IR_RECEIVE_PIN D11
#define IR_SENSOR_PIN PA1
#define IR_SEN_NUM 1
QTRSensorsAnalog qtra((unsigned char[]) {IR_SENSOR_PIN}, IR_SEN_NUM);
unsigned int sensors[IR_SEN_NUM]; 
bool off = 1;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  pinMode(IR_RECEIVE_PIN, INPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  // initialize digital pin LED_BUILTIN as an output.
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  pinMode(D13, OUTPUT);
  digitalWrite(D13, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("Ready to read");
  qtra.emittersOn();
}

// the loop function runs over and over again forever
void loop() { 
    while (off) {
      if (IrReceiver.decode()) {
        int a = IrReceiver.decodedIRData.command;
        if (a == p_ch_p){
          off = 0; 
          Serial.println("On");
        }
        IrReceiver.resume();
      }
    }
    while (!off) {

      if(IrReceiver.decode()) {
        int rec = IrReceiver.decodedIRData.command;
        switch(rec) {
          case p_ch_m:
            off = 1;
            Serial.println("Off");
            break;
          case p_eq:
            int i;
            for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
            {
              qtra.calibrate();
              delay(20);
            }
            break;
          case p_play:
            qtra.read(sensors);
            Serial.println("Sensors readings");
            Serial.println(sensors[0]);
            break;
          default:
            Serial.println(rec);
            break;
        }
        IrReceiver.resume();
      }

    }

}
