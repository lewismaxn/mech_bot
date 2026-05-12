#include "CytronMotorDriver.h"
 
// ===== Serial input from ESP32 =====
// GP16 = RX pin for Serial1
 
// ===== Buzzer =====
const int buzzerPin = 22;
 
// ===== Motors — PWM_DIR mode =====
// M2 = Right motor (GP10=PWM, GP11=DIR)
CytronMD motorRight(PWM_DIR, 10, 11);
 
// M4 = Left mot#include "CytronMotorDriver.h"
 
// ===== Serial input from ESP32 =====
// GP16 = RX pin for Serial1
 
// ===== Buzzer =====

 
// M4 = Left motor (GP14=PWM, GP15=DIR)
CytronMD motorLeft(PWM_DIR, 14, 15);
 
// ===== Speed settings =====
const int slowSpeed = 150;
const int turnSpeed = 150;
 
// ===== State tracking =====
int currentState = 0;
int lastState = -1;
bool turnPauseComplete = false;
unsigned long turnPauseStart = 0;
bool emergencyStopped = false;
bool driveMusicPlayed = false;
bool turnMusicPlayed = false;
 
// ===== Music notes =====
#define NOTE_C4  262
#define NOTE_E4  330
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_G5  784
 
void stopMotors() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}
 
void playDriveMusic() {
  int melody[] = {
    NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5,
    NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5
  };
  int durations[] = {
    150, 150, 150, 300,
    150, 150, 150, 300
  };
  int notes = sizeof(melody) / sizeof(melody[0]);
 
  for (int i = 0; i < notes; i++) {
    tone(buzzerPin, melody[i], durations[i]);
    delay(durations[i] + 30);
    noTone(buzzerPin);
  }
}
 
void playTurnMusic() {
  int melody[] = {
    NOTE_D5, NOTE_E5, NOTE_G5
  };
  int durations[] = {
    150, 150, 400
  };
  int notes = sizeof(melody) / sizeof(melody[0]);
 
  for (int i = 0; i < notes; i++) {
    tone(buzzerPin, melody[i], durations[i]);
    delay(durations[i] + 30);
    noTone(buzzerPin);
  }
}
 
void readSerialState() {
  while (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    line.trim();
 
    if (line.length() == 0) return;
 
    int newState = line.toInt();
 
    if (newState >= 0 && newState <= 4) {
      if (newState == 0) {
        emergencyStopped = true;
        stopMotors();
        currentState = 0;
        Serial.println("RX: EMERGENCY STOP");
        return;
      }
 
      if (newState != currentState) {
        currentState = newState;
        if (currentState == 3) {
          turnPauseComplete = false;
          turnPauseStart = 0;
        }
        Serial.print("RX STATE: ");
        Serial.println(currentState);
      }
    }
  }
}
 
void setup() {
  Serial.begin(115200);
 
  Serial1.setRX(16);
  Serial1.begin(9600);
 
  pinMode(buzzerPin, OUTPUT);
 
  stopMotors();
 
  Serial.println("MOTION 2350 Pro ready");
  Serial.println("Waiting for serial on GP16...");
}
 
void loop() {
  readSerialState();
 
  if (emergencyStopped) {
    stopMotors();
    delay(50);
    return;
  }
 
  // === STATE 0: NO SIGNAL ===
  if (currentState == 0) {
    stopMotors();
  }
 
  // === STATE 1: GATHER — motors off ===
  else if (currentState == 1) {
    stopMotors();
  }
 
  // === STATE 2: DRIVE — play music then slow reverse ===
  else if (currentState == 2) {
    if (!driveMusicPlayed) {
      playDriveMusic();
      driveMusicPlayed = true;
      Serial.println("STATE 2: DRIVE — MUSIC DONE, REVERSING");
    }
 
    motorLeft.setSpeed(-slowSpeed);
    motorRight.setSpeed(-slowSpeed);
 
    if (lastState != 2) {
      lastState = 2;
    }
  }
 
  // === STATE 3: TURN — stop 3 sec, play music, then right motor ===
  else if (currentState == 3) {
    if (!turnPauseComplete) {
      if (turnPauseStart == 0) {
        stopMotors();
        turnPauseStart = millis();
        Serial.println("STATE 3: TURN — 3 SEC PAUSE");
      }
 
      if (millis() - turnPauseStart >= 3000) {
        if (!turnMusicPlayed) {
          playTurnMusic();
          turnMusicPlayed = true;
          Serial.println("STATE 3: TURN — MUSIC DONE, TURNING");
        }
        turnPauseComplete = true;
      } else {
        stopMotors();
      }
    } else {
      motorLeft.setSpeed(0);
      motorRight.setSpeed(turnSpeed);
    }
 
    if (lastState != 3) {
      lastState = 3;
    }
  }
 
  // === STATE 4: PAUSED — motors off ===
  else if (currentState == 4) {
    stopMotors();
  }
 
  delay(50);
}