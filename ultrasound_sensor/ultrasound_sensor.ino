#define  ULTRASOUND_READ_PIN  PA7
#define  MAX_RANGE            520 // The max measurement vaule of the module is 520cm (a little bit longer than the effective max range)
#define  ADC_RESOLUTION       ((1 << 12) - 1) //ADC accuracy of STM32 F411 is 12bit
#define  CALIBRATION_CONST    (3300.0 / 5000.0) 
// Not sure why needed. There are some problems with ADC/Voltage supply, maybe the detector operates on 3.3V.
// The results is that the max voltage is not 5V but sth around 3.3 V, even for a supply from a 5V converter. 


void setup() {
  Serial.begin(9600);
  pinMode(ULTRASOUND_READ_PIN, INPUT);
  analogReadResolution(12);
}

float dist, sensity;

void loop() {
  // read the value from the sensor
  sensity = analogRead(ULTRASOUND_READ_PIN);

  dist = CALIBRATION_CONST * sensity * MAX_RANGE / ADC_RESOLUTION;

  Serial.print(sensity);
  Serial.print(" ");
  Serial.print(sensity / ADC_RESOLUTION);
  Serial.print(" ");
  Serial.print(dist, 0);
  Serial.println("cm");

  delay(50);
}