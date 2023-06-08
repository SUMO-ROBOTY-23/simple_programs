#include "robot_state.h"

#define MOTOR1_PIN1         PA0
#define MOTOR1_PIN2         PA1
#define MOTOR2_PIN1         PA2
#define MOTOR2_PIN2         PA3
#define DISTANCE_PIN1       PB12
#define DISTANCE_PIN2       PB13
#define DISTANCE_PIN3       PB14
#define DISTANCE_ADDRESS1   0x30
#define DISTANCE_ADDRESS2   0x31
#define DISTANCE_ADDRESS3   0x32
#define REFLECTANCE_PIN1    PA4
#define REFLECTANCE_PIN2    PA5
#define REFLECTANCE_PIN3    PA6
#define REFLECTANCE_PIN4    PA7
#define ULTRASOUND_PIN      PA7
#define IR_PIN              PA8

Pins pins = {
  MOTOR1_PIN1,
  MOTOR1_PIN2,
  MOTOR2_PIN1,
  MOTOR2_PIN2,
  DISTANCE_PIN1,
  DISTANCE_PIN2,
  DISTANCE_PIN3,
  DISTANCE_ADDRESS1,
  DISTANCE_ADDRESS2,
  DISTANCE_ADDRESS3,
  REFLECTANCE_PIN1,
  REFLECTANCE_PIN2,
  REFLECTANCE_PIN3,
  REFLECTANCE_PIN4,
  ULTRASOUND_PIN,
  IR_PIN
};

Robot robot = Robot(pins);

void setup() {
  pinMode(PC13, OUTPUT);
  robot.setup_sensors();
}


void loop() {
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(1000);                     
  digitalWrite(LED_BUILTIN, LOW);  
  delay(1000);                    
}