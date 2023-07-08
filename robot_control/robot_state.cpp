#include "robot_state.h"
#include "pilot_names.h"
#include "IRremote.hpp"


#define  ADC_BITS                        12
#define  ADC_MAX_VALUE                   ((1 << ADC_BITS) - 1) // ADC accuracy of STM32 F411 is 12bit
#define  ULTRASOUND_MAX_RANGE            520                   // The max measurement value of the module is 520cm
#define  ULTRASOUND_CALIBRATION_CONST    (3300.0 / 5000.0) // TODO Change to number without division for faster evaluation
#define  WAIT_TIME                       5000
#define  MAX_SPEED                       255
#define  REFRESH_TIME                    35.7 // TODO adjust
#define  ROTATION_CONSTANT               0.063 // TODO adjust
#define  ROBOT_SPEED                     0.11 // TODO adjust
#define  RIGHT_MOTOR_CORRECTION          0.96 // TODO adjust
#define  MAX_WHITE_REFLECTANCE           75 // TODO adjust

void Wait::execute_instruction(MotorState* motor_state) {
  motor_state->left_speed = 0;
  motor_state->right_speed = 0;
  milliseconds -= REFRESH_TIME;
  finished = milliseconds <= 0; 
}

void Rotate::execute_instruction(MotorState* motor_state) {
  if (clockwise) {
    motor_state->left_speed = MAX_SPEED;
    motor_state->right_speed = -MAX_SPEED;
  } else {
    motor_state->left_speed = -MAX_SPEED;
    motor_state->right_speed = MAX_SPEED;
  }

  angle -= ROTATION_CONSTANT * REFRESH_TIME;
  finished = angle <= 0;
}

void Go::execute_instruction(MotorState* motor_state) {
  if (forward) {
    motor_state->left_speed = speed;
    motor_state->right_speed = speed;
  } else {
    motor_state->left_speed = -speed;
    motor_state->right_speed = -speed;
  }

  distance -= REFRESH_TIME * ROBOT_SPEED * speed / MAX_SPEED;
  finished = distance <= 0;
}

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
  // uint8_t const reflectance_pins[REFLECTANCE_SENSOR_COUT] = {pins.reflectance_pin1, pins.reflectance_pin2, pins.reflectance_pin3, pins.reflectance_pin4};
  uint8_t const reflectance_pins[REFLECTANCE_SENSOR_COUT] = {pins.reflectance_pin1, pins.reflectance_pin2};
  reflectance_sensors = ReflectanceSensors(reflectance_pins);

  motor_state = {0, 0};
}

void Robot::setup_distance_sensors() {
  Wire.begin();  // TODO Necessary?

  distance_sensor1.pin_output();
  // distance_sensor2.pin_output();
  distance_sensor3.pin_output();

  // TODO Remove those three lines?
  distance_sensor1.write_pin(LOW);
  // distance_sensor2.write_pin(LOW);
  distance_sensor3.write_pin(LOW);

  distance_sensor1.write_pin(LOW);
  // distance_sensor2.write_pin(LOW);
  distance_sensor3.write_pin(LOW);
  delay(10);

  distance_sensor1.write_pin(HIGH);
  // distance_sensor2.write_pin(LOW);
  distance_sensor3.write_pin(LOW);

  if (!distance_sensor1.begin()) {
    Serial.println(F("Failed to boot first VL53L0X"));
    Serial.flush();
    while (1); // TODO Restart program
  }
  delay(10);

  // distance_sensor2.write_pin(HIGH);
  // delay(10);

  // if (!distance_sensor2.begin()) {
  //   Serial.println(F("Failed to boot second VL53L0X"));
  //   Serial.flush();
  //   while (1); // TODO Restart program
  // }

  distance_sensor3.write_pin(HIGH);
  delay(10);

  if (!distance_sensor3.begin()) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while (1); // TODO Restart program
  }

  distance_sensor1.start_continuous();
  // distance_sensor2.start_continuous();
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
  // distance_sensor2.wait_ready();
  // distance_measurements[1] = distance_sensor2.get_distance();
  distance_sensor3.wait_ready();
  distance_measurements[2] = distance_sensor3.get_distance();

  reflectance_sensors.read_reflectance(reflectance_measurements);

  if (IrReceiver.decode()) {
        IR_measurement = IrReceiver.decodedIRData.command;
        IrReceiver.resume();
  }
}


void Robot::print_measurements() {
  if (started) {
    Serial.print("Started ");
  } else {
    Serial.print("Stopped ");

  }

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

void Robot::clear_queue() {
  for (auto ins_ptr : current_instructions) {
      delete ins_ptr;
    } 
    current_instructions.clear();
}

void Robot::make_decision() {
  if (!started && IR_measurement == P_ON_OFF) {
    started = true;
//    digitalWrite(PC13, HIGH);
    delay(5000);
//     current_instructions.push_back(new Wait(WAIT_TIME));
//     wait = true;
    current_instructions.push_back(new Go(true, MAX_SPEED, UINT16_MAX)); // TODO change distance or speed
  } else if (started && IR_measurement == P_FUNC) {
//    digitalWrite(PC13, LOW);
    started = false;
    clear_queue();
  }
  IR_measurement = 0;

//  if (wait) {
//    return;
//  }

   if (ultrasound_measurement <= 500) {
     clear_queue();
     current_instructions.push_back(new Go(true, MAX_SPEED, UINT16_MAX));
   } else if (reflectance_measurements[0] <= MAX_WHITE_REFLECTANCE) {
     clear_queue();
     current_instructions.push_back(new Go(false, MAX_SPEED, 50));
     current_instructions.push_back(new Rotate(150, true));
     current_instructions.push_back(new Go(true, MAX_SPEED, UINT16_MAX));
   } else if (reflectance_measurements[1] <= MAX_WHITE_REFLECTANCE) {
     clear_queue();
     current_instructions.push_back(new Go(false, 50, MAX_SPEED));
     current_instructions.push_back(new Rotate(150, false));
     current_instructions.push_back(new Go(true, MAX_SPEED, UINT16_MAX));
   } else if (distance_measurements[0] <= 800 || distance_measurements[2] <= 800) {
     if (distance_measurements[0] < distance_measurements[2]) {
       clear_queue();
       current_instructions.push_back(new Rotate(30, false));
     } else {
       clear_queue();
       current_instructions.push_back(new Rotate(30, true));
     }
   } else if (current_instructions.empty()) {
     current_instructions.push_back(new Rotate(180, true));
     current_instructions.push_back(new Go(true, MAX_SPEED, UINT16_MAX));
   }

  
  

  
  // najpierw sprawdzamy, czy coś jest przed nami na odległość ok 10 cm, jeśli tak - jazda na przód

  //jeśli nie, to sprawdzamy, czy najeżdżamy na białą linię - jeśli tak - jazda w tył

  //jeśli przed nami nic nie ma na odległość 10 cm, to sprawdzamywyniki czujników laserowych i jeszcze raz dźwiękowych
  //jeśli coś przed nami jest, to  korygujemy trasę na czujnik, który wykrywa najbliżej i jedziemy w tą stornę (dźwiękowy - 0 stopni, lewy -15 stopni, prawy 15 stopni (lub inna wartość)
  //jeśli czujniki nic nie wykryły- obróć się
  
  // TODO rest of decision making
  // TODO push back next instruction or clear the queue and push new instruction
}

void Robot::set_speed(int16_t new_left_speed, int16_t new_right_speed) {
  if (motor_state.left_speed != new_left_speed) {
    motor_state.left_speed = new_left_speed;
    motor1.setSpeed(new_left_speed * RIGHT_MOTOR_CORRECTION);
  }

  if (motor_state.right_speed != new_right_speed) {
    motor_state.right_speed = new_right_speed;
    motor2.setSpeed(-new_right_speed);
  }
}

void Robot::run_decision() {
  if (!started) {
    set_speed(0, 0);
  } else if (!current_instructions.empty()) {
//    set_speed(MAX_SPEED, MAX_SPEED);

     MotorState new_motor_state;
     current_instructions.front()->execute_instruction(&new_motor_state);
     set_speed(new_motor_state.left_speed, new_motor_state.right_speed);
     if (current_instructions.front()->is_finished()) {
//        wait = false;
       delete current_instructions.front();
       current_instructions.pop_front();
     }
  }
}
