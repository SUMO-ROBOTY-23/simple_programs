#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// set the pins to shutdown
#define SHT_LOX1 PB12
#define SHT_LOX2 PB13
#define SHT_LOX3 PB14

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2 and LOX3
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1);
  }
  delay(10);

  Serial.println(F("First activated"));


  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1);
  }

  Serial.println(F("Second activated"));

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX2
  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while (1);
  }

  Serial.println(F("Third activated"));

  lox1.startRangeContinuous();
  lox2.startRangeContinuous();
  lox3.startRangeContinuous();

  Serial.println(F("Continous activated"));
}

void read_dual_sensors() {

  if(lox1.waitRangeComplete()) {
    Serial.print("Distance 1 in mm: ");
    Serial.print(lox1.readRange());
  }
  if(lox2.waitRangeComplete()) {
    Serial.print(" Distance 2 in mm: ");
    Serial.print(lox2.readRange());
  }
  if(lox3.waitRangeComplete()) {
    Serial.print(" Distance 3 in mm: ");
    Serial.print(lox3.readRange());
  }

  Serial.println();
}

void setup() {
  pinMode(PC13, OUTPUT);
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (!Serial) { delay(1); }

  Wire.begin();  // TODO Necessary?

  digitalWrite(PC13, HIGH);

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));


  Serial.println(F("Starting..."));
  setID();
}

void loop() {
  read_dual_sensors();

  // int time1 = micros();
  // for (int i = 0; i < 100; ++i) {
  //   read_dual_sensors();
  // }
  // int time2 = micros();

  // Serial.println(s);
  // Serial.println((time2 - time1) / 100000.0);
}
