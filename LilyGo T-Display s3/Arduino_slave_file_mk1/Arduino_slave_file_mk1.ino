#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>

//pin for sequential state transition recieving
#define COMMS_PIN 1

//left motor pins
#define LM_In1 21
#define LM_In2 16
#define LM_En 43 //PWM pin

//right Motor pins
#define RM_In1 18
#define RM_In2 17
#define RM_En 44 //PWM pin

//Linear Actuator pins
#define LA_In1 2
#define LA_In2 3

//drive speeds (0-255)
#define APPROACH_SPEED 30
#define TURN_SPEED 15
#define DRIVE_SPEED 15
#define RAMP_DELAY 1000 //delay before slowing down motors (milliseconds)

// Motor struct
struct Motor {
  int in1, in2, en;
};

struct LinearAct {
  int in1, in2;
};


// Declare motors and actuator
Motor leftMotor  = { LM_In1, LM_In2, LM_En };
Motor rightMotor = { RM_In1, RM_In2, RM_En };
LinearAct LinearActuator = {LA_In1, LA_In2};

//function to set pinmode of motors
void motorSetup(Motor m) {
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
  pinMode(m.en,  OUTPUT);
}
void LinearActSetup(LinearAct l) { //thats a lowecase L
  pinMode(l.in1, OUTPUT);
  pinMode(l.in2, OUTPUT);
}

// Speed: 0–255 | Direction: 1 = forward, -1 = reverse, 0 = stop
void setMotor(Motor m, int direction, int speed) {
  analogWrite(m.en, speed);

  if (direction == 1) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
  } else if (direction == -1) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
  } else {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
    analogWrite(m.en, 0);
  }
}
//linear actuator control function
void setLinearAct(LinearAct l, int direction) {
  if (direction == 1) {
    digitalWrite(l.in1, HIGH);
    digitalWrite(l.in2, LOW);
  }else if (direction == -1) {
    digitalWrite(l.in1, LOW);
    digitalWrite(l.in2, HIGH);
  } else {
      digitalWrite(l.in1, LOW);
      digitalWrite(l.in2, LOW);
  }
  
}
//stop motor function
void stopMotors() {
  setMotor(leftMotor, 0, 0);
  setMotor(rightMotor, 0, 0);
}
//stop actuator
void stopActuator(){
  setLinearAct(LinearActuator, 0);
}


//States: 
bool lastPinState = LOW;

enum Level {
  WAITING,
  CALIBRATION,
  DRIVE,
  STOPA,
  TURN,
  DRIVERAMP,
  STOPB,
  DOWNRAMP,
  FINISH
};
Level currentState = WAITING;



void setup() {
  Serial.begin(115200);

  motorSetup(leftMotor);
  motorSetup(rightMotor);
  LinearActSetup(LinearActuator);
  stopMotors();
  stopActuator();
  
   
  Serial.println("SLAVE ready for you master");
  Serial.println("Waiting for pulses on GPIO1...");
  
}

void loop() {

  bool pinNow = digitalRead(COMMS_PIN);

  if (pinNow == HIGH && lastPinState == LOW) {
    currentState++;
    Serial.print("PULSE DETECTED -> STATE: ")
    Serial.println(currentState);

    delay(150);
  }

  lastPinState = pinNow;

  //== WAITING STATE ==
  if (currentState == WAITING) {
    stopMotors();
    stopActuator();

  }
  // == CALIBRATION STATE ==
  else if (currentState == CALIBRATION) {
    stopMotors();
    stopActuator();
  }

  // == DRIVE STATE == (slow reverse)
  else if (currentState == DRIVE) {
    setMotor(leftMotor, -1, DRIVE_SPEED);
    setMotor(rightMotor, -1, DRIVE_SPEED);
    stopActuator();
  }

  // == STOPA STATE == 
  else if (currentState == STOPA){
      stopMotors();
      stopActuator();
  }

  // == TURN STATE ==

  else if (currentState == TURN) {
    setMotor(leftMotor, -1, TURN_SPEED);
    setMotor(rightMotor, 1, TURN_SPEED);
    stopActuator();
  }

  // == DRIVERAMP STATE ==

  else if (currentState == DRIVERAMP) {
    setMotor(leftMotor, 1, APPROACH_SPEED);
    setMotor(rightMotor, 1, APPROACH_SPEED);
    stopActuator();
    delay(RAMP_DELAY);// small delay to let the robot 'mount' the ramp
    setMotor(leftMotor, 1, DRIVE_SPEED);
    setMotor(rightMotor, 1, DRIVE_SPEED);   
  }

// == STOPB STATE ==

  else if (currentState == STOPB) {
    stopActuator();
    stopMotors();

  }
}





