#include "robot_state.h"
#include "pilot_names.h"
#include "IRremote.hpp"


#define  ADC_BITS                        12
#define  ADC_MAX_VALUE                   ((1 << ADC_BITS) - 1) // ADC accuracy of STM32 F411 is 12bit
#define  ULTRASOUND_MAX_RANGE            520                   // The max measurement value of the module is 520cm
#define  ULTRASOUND_CALIBRATION_CONST    (3300.0 / 5000.0)
#define  WAIT_TIME                       5000
#define  MAX_SPEED                       255


void UltrasoundSensor::setup() {
  pinMode(pin, INPUT);
}

uint16_t UltrasoundSensor::get_distance() {
  float sensity = analogRead(pin);
  float dist = ULTRASOUND_CALIBRATION_CONST * ULTRASOUND_MAX_RANGE * sensity / ADC_MAX_VALUE;
  return static_cast<uint16_t>(dist * 10); // Returns distance in millimeters
}

void DistanceSensor::pin_output() {
  pinMode(pin, OUTPUT);
}

void DistanceSensor::write_pin(uint32_t value) {
  digitalWrite(pin, value);
}

bool DistanceSensor::begin() {
  return sensor.begin(address);
}

bool DistanceSensor::start_continuous() {
  return sensor.startRangeContinuous();
}

bool DistanceSensor::wait_ready() {
  return sensor.waitRangeComplete();
}

uint16_t DistanceSensor::get_distance() {
  return sensor.readRange();
}

ReflectanceSensors::ReflectanceSensors(uint8_t const pins[REFLECTANCE_SENSOR_COUT]) {
  memcpy(reflectance_pins, pins, REFLECTANCE_SENSOR_COUT * sizeof(uint8_t));
}

void ReflectanceSensors::setup_sensors() {
  sensors.setSensorPins(reflectance_pins, REFLECTANCE_SENSOR_COUT);
  sensors.setTypeAnalog();
  sensors.emittersOn();
}

void ReflectanceSensors::read_reflectance(uint16_t* sensor_values) {
  sensors.read(sensor_values);
}


Robot::Robot(Pins pins) :
      ultrasound_sensor(pins.ultrasound_pin),
      distance_sensor1(pins.distance_pin1, pins.distance_address1),
      distance_sensor2(pins.distance_pin2, pins.distance_address2),
      distance_sensor3(pins.distance_pin3, pins.distance_address3),
      motor1(PWM_PWM, pins.motor1_pin1, pins.motor1_pin2), 
      motor2(PWM_PWM, pins.motor2_pin1, pins.motor2_pin2),
      IR_pin(pins.IR_pin) {
  uint8_t const reflectance_pins[4] = {pins.reflectance_pin1, pins.reflectance_pin2, pins.reflectance_pin3, pins.reflectance_pin4};
  reflectance_sensors = ReflectanceSensors(reflectance_pins);

  motor_state = {0, 0};
}

void Robot::setup_distance_sensors() {
  Wire.begin();  // TODO Necessary?

  distance_sensor1.pin_output();
  distance_sensor2.pin_output();
  distance_sensor3.pin_output();

  // TODO Remove those three lines?
  distance_sensor1.write_pin(LOW);
  distance_sensor2.write_pin(LOW);
  distance_sensor3.write_pin(LOW);

  distance_sensor1.write_pin(LOW);
  distance_sensor2.write_pin(LOW);
  distance_sensor3.write_pin(LOW);
  delay(10);

  distance_sensor1.write_pin(HIGH);
  distance_sensor2.write_pin(LOW);
  distance_sensor3.write_pin(LOW);

  if (!distance_sensor1.begin()) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1); // TODO Restart program
  }
  delay(10);

  distance_sensor2.write_pin(HIGH);
  delay(10);

  if (!distance_sensor2.begin()) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1); // TODO Restart program
  }

  distance_sensor3.write_pin(HIGH);
  delay(10);

  if (!distance_sensor3.begin()) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while (1); // TODO Restart program
  }

  distance_sensor1.start_continuous();
  distance_sensor2.start_continuous();
  distance_sensor3.start_continuous();
}

void Robot::setup_sensors() {
  pinMode(IR_pin, INPUT);
  IrReceiver.begin(IR_pin, ENABLE_LED_FEEDBACK);


  ultrasound_sensor.setup();
  setup_distance_sensors();
  reflectance_sensors.setup_sensors();
}

void Robot::read_sensors() {
  ultrasound_measurement = ultrasound_sensor.get_distance();

  distance_sensor1.wait_ready();
  distance_measurements[0] = distance_sensor1.get_distance();
  distance_sensor2.wait_ready();
  distance_measurements[1] = distance_sensor2.get_distance();
  distance_sensor3.wait_ready();
  distance_measurements[2] = distance_sensor3.get_distance();

  reflectance_sensors.read_reflectance(reflectance_measurements);

  if (IrReceiver.decode()) {
        IR_measurement = IrReceiver.decodedIRData.command;
        IrReceiver.resume();
  }
}


void Robot::print_measurements() {
  Serial.print("Ultrasound: ");
  Serial.print(ultrasound_measurement);

  Serial.print(" mm  Distance1: ");
  Serial.print(distance_measurements[0]);
  Serial.print(" mm  Distance2: ");
  Serial.print(distance_measurements[1]);
  Serial.print(" mm  Distance3: ");
  Serial.print(distance_measurements[2]);

  Serial.print(" mm  Reflectance1: ");
  Serial.print(reflectance_measurements[0]);
  Serial.print(" Reflectance2: ");
  Serial.print(reflectance_measurements[1]);
  Serial.print(" Reflectance3: ");
  Serial.print(reflectance_measurements[2]);
  Serial.print(" Reflectance4: ");
  Serial.println(reflectance_measurements[3]);

}

void Robot::make_decision() {
  if (!started && IR_measurement == P_ON_OFF) {
    started = true;
    current_instructions.push_back(new Wait(WAIT_TIME));
    current_instructions.push_back(new Go(UINT16_MAX, MAX_SPEED)); // TODO change distance or speed
  } else if (started && IR_measurement == P_FUNC) {
    started = false;
    for (auto ins_ptr : current_instructions) {
      delete ins_ptr;
    } 
    current_instructions.clear();
  }


  // TODO rest of decision making

}
