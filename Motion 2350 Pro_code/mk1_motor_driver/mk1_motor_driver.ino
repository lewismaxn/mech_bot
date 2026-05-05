#include "CytronMotorDriver.h"
 #define MLPWM 10
 #define MRPWM 8
 #define MLDIR 11
 #define MRDIR 9
// ===== Input from ESP32 =====
const int statePin = 16;
 
// ===== Buzzer =====
const int buzzerPin = 22;
 
// ===== Motors — PWM_DIR mode =====
CytronMD motorRight(PWM_DIR, 8, 9);  // M2
CytronMD motorLeft(PWM_DIR, 10, 11);   // M4

 
// ===== Speed settings =====
const int driveSpeed = 15;
const int turnSpeed = 15;
 
// ===== State tracking =====
// 0=waiting, 1=calibration, 2=drive, 3=stopA, 4=turn, 5=stop
int currentState = 0;
bool lastPinState = LOW;

enum Level {
  WAITING,
  CALIBRATION,
  DRIVE,
  STOPA,
  TURN,
  STOPB
};


void stopMotors() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}
 
void playMusic() {
  int melody[] = { 262, 330, 392, 523, 392, 440, 494, 523, 587, 659, 784 };
  int durations[] = { 150, 150, 150, 300, 150, 150, 150, 300, 150, 150, 400 };
  int notes = sizeof(melody) / sizeof(melody[0]);
 
  for (int i = 0; i < notes; i++) {
    tone(buzzerPin, melody[i], durations[i]);
    delay(durations[i] + 30);
    noTone(buzzerPin);
  }
}

 void setMotorSpeed(CytronMD &motor, int pwmPin, int dirPin, int speed) {
  if (speed >= 0) {
    // Positive
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, speed);
  } else {
    // Negative
    digitalWrite(dirPin, HIGH);          // set direction to reverse
    analogWrite(pwmPin, speed);         // apply PWM
  }
}

void setup() {
  Serial.begin(115200);
 
  pinMode(statePin, INPUT_PULLDOWN);
  pinMode(buzzerPin, OUTPUT);
 
  stopMotors();
 
  Serial.println("MOTION 2350 Pro ready");
  Serial.println("Waiting for pulses on GP16...");
}
 
void loop() {
  // Detect rising edge on GP16
  bool pinNow = digitalRead(statePin);
 
  if (pinNow == HIGH && lastPinState == LOW) {
    // Pulse detected — advance state
    currentState++;
    if (currentState == 6) {currentState = 0;}
    Serial.print("PULSE DETECTED -> STATE: ");
    Serial.println(currentState);
 
    // Brief delay to let pulse finish
    delay(150);
  }
 
  lastPinState = pinNow;
 
  // === STATE 0: WAITING ===
  if (currentState == WAITING) {
    stopMotors();
  }
 
  // === STATE 1: CALIBRATION — motors off, play music ===
  else if (currentState == CALIBRATION) {
    stopMotors();
    // Music plays once when entering
    static bool musicPlayed = false;
    if (!musicPlayed) {
      Serial.println("STATE 1: CALIBRATION — PLAYING MUSIC");
      //playMusic();
      musicPlayed = true;
      Serial.println("STATE 1: CALIBRATION — WAITING");
    }
  }
 
  // === STATE 2: DRIVE — both motors slow reverse ===
  else if (currentState == DRIVE) {
    static bool driveStarted = false;
    if (!driveStarted) {
      Serial.println("STATE 2: DRIVE — SLOW REVERSE");
      driveStarted = true;
    }
   // motorLeft.setSpeed(-driveSpeed);
   // motorRight.setSpeed(-driveSpeed);
    setMotorSpeed(motorLeft, MLPWM, MLDIR, -turnSpeed);
    setMotorSpeed(motorRight, MRPWM, MRDIR, -turnSpeed);
  }
 
  // === STATE 3: STOP A — motors off ===
  else if (currentState == STOPA) {
    static bool stopAStarted = false;
    if (!stopAStarted) {
      Serial.println("STATE 3: STOP A — MOTORS OFF");
      stopAStarted = true;
    }
    stopMotors();
  }
 
  // === STATE 4: TURN — M4 reverse, M2 forward ===
  else if (currentState == TURN) {
    static bool turnStarted = false;
    if (!turnStarted) {
      Serial.println("STATE 4: TURN — M4 REVERSE, M2 FORWARD");
      turnStarted = true;
    }
    motorRight.setSpeed(turnSpeed);
    setMotorSpeed(motorRight, MRPWM, MRDIR, turnSpeed);  // M2 forward
    setMotorSpeed(motorLeft, MLPWM, MLDIR, -turnSpeed);   // M4 reverse
    
  }
 
  // === STATE 5: STOP B — motors off, done ===
  else if (currentState == STOPB) {
    static bool stopBStarted = false;
    if (!stopBStarted) {
      Serial.println("STATE 5: STOP B — COMPLETE");
      stopBStarted = true;
    }
    stopMotors();
  }
 
  delay(20);
}