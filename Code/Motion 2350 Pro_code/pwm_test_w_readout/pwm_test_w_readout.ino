#include "CytronMotorDriver.h"

// Motor on M2 — PWM pin 10, DIR pin 11
CytronMD motorRight(PWM_DIR, 10, 11);
// Motor on M4 — PWM pin 14, DIR pin 15
CytronMD motorLeft(PWM_DIR, 14, 15);

const int START_SPEED = 5;
const int MAX_SPEED   = 255;
const int STEP        = 10;
const int INTERVAL_MS = 2000;

int currentSpeed = START_SPEED;
bool motorsStopped = false;

void setup() {
  Serial.begin(115200);
  Serial.println("PWM Ramp Test — Starting");

  motorLeft.setSpeed(currentSpeed);
  motorRight.setSpeed(currentSpeed);

  Serial.print("Speed set to: ");
  Serial.println(currentSpeed);
}

void loop() {
  if (motorsStopped) return;

  delay(INTERVAL_MS);

  currentSpeed += STEP;

  if (currentSpeed >= MAX_SPEED) {
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);
    motorsStopped = true;
    Serial.println("Max reached — Motors STOPPED");
    return;
  }

  motorLeft.setSpeed(currentSpeed);
  motorRight.setSpeed(currentSpeed);

  Serial.print("Speed set to: ");
  Serial.println(currentSpeed);
}