#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <cstdint>
#include "Adafruit_VL53L0X.h"
#include "CytronMotorDriver.h"
#include "QTRSensors.h"

#define  REFLECTANCE_SENSOR_COUT 4

typedef struct Pins_t {
  uint8_t motor1_pin1;
  uint8_t motor1_pin2;
  uint8_t motor2_pin1;
  uint8_t motor2_pin2;
  uint8_t distance_pin1;
  uint8_t distance_pin2;
  uint8_t distance_pin3;
  uint8_t distance_address1;
  uint8_t distance_address2;
  uint8_t distance_address3;
  uint8_t reflectance_pin1;
  uint8_t reflectance_pin2;
  uint8_t reflectance_pin3;
  uint8_t reflectance_pin4;
  uint8_t ultrasound_pin;
  uint8_t IR_pin;
} Pins;

class Instruction {};

class Wait : public Instruction {
public:
  Wait(uint32_t milliseconds) : milliseconds(milliseconds) {}
private:
  uint32_t milliseconds;
};

class Rotate : public Instruction {
private:
  float angle;
};

class Go : public Instruction {
private:
  float distance;
};

class UltrasoundSensor {
public:
  UltrasoundSensor();
  UltrasoundSensor(uint8_t pin) : pin(pin) {}
  void setup();
  uint16_t get_distance();
private:
  uint8_t pin;
};

class DistanceSensor {
public:
  DistanceSensor();
  DistanceSensor(uint8_t pin, uint8_t address) : pin(pin), address(address) {}
  void pin_output();
  void write_pin(uint32_t value);
  bool begin();
  bool start_continuous();
  bool wait_ready();
  uint16_t get_distance();

private:
  uint8_t pin;
  uint8_t address;
  Adafruit_VL53L0X sensor;
};

class ReflectanceSensors {
public:
  ReflectanceSensors() {};
  ReflectanceSensors(uint8_t const pins[REFLECTANCE_SENSOR_COUT]);
  void setup_sensors();
  void read_reflectance(uint16_t* sensorValues);
private:
  QTRSensors sensors;
  uint8_t reflectance_pins[REFLECTANCE_SENSOR_COUT]; 
};

typedef struct MotorState_t {
  int16_t left_speed;
  int16_t right_speed;
} MotorState;


class Robot {
public:
  Robot(Pins pins);
  void setup_sensors();
  void read_sensors();

private:
  void setup_distance_sensors();

  UltrasoundSensor ultrasound_sensor;
  DistanceSensor distance_sensor1;
  DistanceSensor distance_sensor2;
  DistanceSensor distance_sensor3;
  ReflectanceSensors reflectance_sensors;
  uint8_t IR_pin;

  uint16_t ultrasound_measurement;
  uint16_t distance_measurements[3]; 
  uint16_t reflectance_measurements[4]; 
  uint16_t IR_measurement;

  Instruction current_instruction;
  MotorState motor_state;
  CytronMD motor1;
  CytronMD motor2;
};


#endif
