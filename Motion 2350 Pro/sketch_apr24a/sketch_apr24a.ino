#include "CytronMotorDriver.h"
 
// ===== Input from ESP32 =====
const int statePin = 16;
 
// ===== Buzzer =====
const int buzzerPin = 22;
 
// ===== Motors — PWM_DIR mode =====
CytronMD motorRight(PWM_DIR, 10, 11);  // M2
CytronMD motorLeft(PWM_DIR, 14, 15);   // M4
 
// ===== Speed settings =====
const int driveSpeed = 5;
const int turnSpeed = 5;
 
// ===== State tracking =====
// 0=waiting, 1=calibration, 2=drive, 3=stop1, 4=turn, 5=stop2
int currentState = 0;
bool lastPinState = LOW;
 
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
    Serial.print("PULSE DETECTED -> STATE: ");
    Serial.println(currentState);
 
    // Brief delay to let pulse finish
    delay(150);
  }
 
  lastPinState = pinNow;
 
  // === STATE 0: WAITING ===
  if (currentState == 0) {
    stopMotors();
  }
 
  // === STATE 1: CALIBRATION — motors off, play music ===
  else if (currentState == 1) {
    stopMotors();
    // Music plays once when entering
    static bool musicPlayed = false;
    if (!musicPlayed) {
      Serial.println("STATE 1: CALIBRATION — PLAYING MUSIC");
      playMusic();
      musicPlayed = true;
      Serial.println("STATE 1: CALIBRATION — WAITING");
    }
  }
 
  // === STATE 2: DRIVE — both motors slow reverse ===
  else if (currentState == 2) {
    static bool driveStarted = false;
    if (!driveStarted) {
      Serial.println("STATE 2: DRIVE — SLOW REVERSE");
      driveStarted = true;
    }
    motorLeft.setSpeed(-driveSpeed);
    motorRight.setSpeed(-driveSpeed);
  }
 
  // === STATE 3: STOP 1 — motors off ===
  else if (currentState == 3) {
    static bool stop1Started = false;
    if (!stop1Started) {
      Serial.println("STATE 3: STOP 1 — MOTORS OFF");
      stop1Started = true;
    }
    stopMotors();
  }
 
  // === STATE 4: TURN — M4 reverse, M2 forward ===
  else if (currentState == 4) {
    static bool turnStarted = false;
    if (!turnStarted) {
      Serial.println("STATE 4: TURN — M4 REVERSE, M2 FORWARD");
      turnStarted = true;
    }
    motorLeft.setSpeed(-turnSpeed);   // M4 reverse
    motorRight.setSpeed(turnSpeed);   // M2 forward
  }
 
  // === STATE 5: STOP 2 — motors off, done ===
  else if (currentState == 5) {
    static bool stop2Started = false;
    if (!stop2Started) {
      Serial.println("STATE 5: STOP 2 — COMPLETE");
      stop2Started = true;
    }
    stopMotors();
  }
 
  delay(20);
}