#include <RedBotSoftwareSerial.h>
#include <RedBot.h>

#define trigPin 12
#define echoPin 11

RedBotSensor sen = RedBotSensor(A0);
const int buttonPin = 2;     // the number of the pushbutton pin
// variables will change:
int buttonState = 0;


//QTRSensors qtr;
void setup() {
  //qtr.setTypeAnalog();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonPin, INPUT);
    // initialize digital pin LED_BUILTIN as an output.
    // put your setup code here, to run once:
  Serial.begin(9600);



  /while(!digitalRead(buttonPin)){
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    int i;
    for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
    {
      //qtr.calibrate();
      delay(20);
    }                       // wait for a second
  }/


  digitalWrite(LED_BUILTIN, LOW);


  pinMode(trigPin,OUTPUT);//Pin, do którego podłączymy trig jako wyjście

  pinMode(echoPin,INPUT);//a echo, jako wejście



}

void loop() {
long czas;

digitalWrite(trigPin,LOW);

delayMicroseconds(2);

digitalWrite(trigPin,HIGH);

delayMicroseconds(10);

digitalWrite(trigPin,LOW);

czas=pulseIn(echoPin,HIGH);
String a = String(czas);
String b = String(sen.read());
String toPrint = a + " " + b + "\n";
Serial.print(toPrint);
Serial.print("\n");
delay(100);

}
