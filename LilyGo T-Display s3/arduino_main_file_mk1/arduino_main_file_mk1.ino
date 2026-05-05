#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
 
// HC-SR04 pins
#define TRIG_PIN 2
#define ECHO_PIN 3
 
// MPU-6050 I2C
#define MPU_SDA 17
#define MPU_SCL 18
#define MPU_ADDR 0x68
 
// Button for state advance
#define BUTTON_PIN 43
 
// State transfer to Motion 2350 Pro and Slave TTGO
#define STATE_OUT_PIN 1
#define SLAVE_PIN 10
 
// Create display object
TFT_eSPI tft = TFT_eSPI();
 
// States: 1=Measurements, 2=Calibration, 3=Drive, 4=Stop1, 5=Turn, 6=Stop2
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
 
// MPU-6050 data
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float temperature;
 
// Calibration offsets
float accOffX = 0, accOffY = 0, accOffZ = 0;
float gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;
bool calibrated = false;
 
// Yaw tracking
float yawAngle = 0;
unsigned long lastGyroTime = 0;
 
// Distance
long distance = 0;
 
// Pulse count
int pulseCount = 0;
 
void initMPU6050() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}
 
void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
 
  int16_t rawAccX = Wire.read() << 8 | Wire.read();
  int16_t rawAccY = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ = Wire.read() << 8 | Wire.read();
  int16_t rawTemp = Wire.read() << 8 | Wire.read();
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();
 
  accX = rawAccX / 16384.0;
  accY = rawAccY / 16384.0;
  accZ = rawAccZ / 16384.0;
  temperature = (rawTemp / 340.0) + 36.53;
  gyroX = rawGyroX / 131.0;
  gyroY = rawGyroY / 131.0;
  gyroZ = rawGyroZ / 131.0;
 
  if (calibrated) {
    accX -= accOffX;
    accY -= accOffY;
    accZ -= accOffZ;
    gyroX -= gyroOffX;
    gyroY -= gyroOffY;
    gyroZ -= gyroOffZ;
  }
 
  if (lastGyroTime > 0 && calibrated) {
    unsigned long now = millis();
    float dt = (now - lastGyroTime) / 1000.0;
    yawAngle += gyroZ * dt;
    lastGyroTime = now;
  } else {
    lastGyroTime = millis();
  }
}
 
void sendPulse() {
  digitalWrite(STATE_OUT_PIN, HIGH);
  digitalWrite(SLAVE_PIN, HIGH);
  delay(100);
  digitalWrite(STATE_OUT_PIN, LOW);
  digitalWrite(SLAVE_PIN, LOW);
  pulseCount++;
  Serial.print("PULSE SENT #");
  Serial.println(pulseCount);
}
 
bool buttonPressed() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      while (digitalRead(BUTTON_PIN) == LOW) delay(10);
      delay(50);
      return true;
    }
  }
  return false;
}
 
void runCalibration() {
  int samples = 200;
  float sumAX = 0, sumAY = 0, sumAZ = 0;
  float sumGX = 0, sumGY = 0, sumGZ = 0;
 
  tft.fillScreen(TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString("CALIBRATING", tft.width() / 2, 30);
  tft.setTextSize(1);
  tft.drawString("KEEP ROBOT STILL", tft.width() / 2, 55);
 
  int barX = 20;
  int barY = 75;
  int barW = tft.width() - 40;
  int barH = 16;
  tft.drawRect(barX, barY, barW, barH, TFT_WHITE);
 
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);
 
    int16_t rawAccX = Wire.read() << 8 | Wire.read();
    int16_t rawAccY = Wire.read() << 8 | Wire.read();
    int16_t rawAccZ = Wire.read() << 8 | Wire.read();
    int16_t rawTemp = Wire.read() << 8 | Wire.read();
    int16_t rawGyroX = Wire.read() << 8 | Wire.read();
    int16_t rawGyroY = Wire.read() << 8 | Wire.read();
    int16_t rawGyroZ = Wire.read() << 8 | Wire.read();
 
    sumAX += rawAccX / 16384.0;
    sumAY += rawAccY / 16384.0;
    sumAZ += rawAccZ / 16384.0;
    sumGX += rawGyroX / 131.0;
    sumGY += rawGyroY / 131.0;
    sumGZ += rawGyroZ / 131.0;
 
    int fillW = map(i, 0, samples - 1, 0, barW - 4);
    tft.fillRect(barX + 2, barY + 2, fillW, barH - 4, TFT_GREEN);
 
    delay(10);
  }
 
  accOffX = sumAX / samples;
  accOffY = sumAY / samples;
  accOffZ = (sumAZ / samples) - 1.0;
  gyroOffX = sumGX / samples;
  gyroOffY = sumGY / samples;
  gyroOffZ = sumGZ / samples;
 
  calibrated = true;
  yawAngle = 0;
  lastGyroTime = millis();
 
  tft.fillScreen(TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString("CALIBRATED", tft.width() / 2, 15);
 
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  int y = 40;
  tft.drawString("OFFSETS:", 5, y); y += 15;
  tft.drawString("Acc X: " + String(accOffX, 4), 5, y); y += 13;
  tft.drawString("Acc Y: " + String(accOffY, 4), 5, y); y += 13;
  tft.drawString("Acc Z: " + String(accOffZ, 4), 5, y); y += 13;
  tft.drawString("Gyr X: " + String(gyroOffX, 4), 5, y); y += 13;
  tft.drawString("Gyr Y: " + String(gyroOffY, 4), 5, y); y += 13;
  tft.drawString("Gyr Z: " + String(gyroOffZ, 4), 5, y); y += 18;
 
  tft.setTextDatum(MC_DATUM);
  tft.drawString("PRESS BUTTON TO DRIVE", tft.width() / 2, tft.height() - 10);
 
  while (true) {
    if (buttonPressed()) break;
    delay(10);
  }
}
 
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
 
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  long dist = duration * 0.034 / 2;
  return dist;
}
 
void displayState1() {
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  int y = 5;
 
  tft.fillRect(0, 0, tft.width(), 15, TFT_BLUE);
  tft.drawString("STATE 1: MEASUREMENTS (RAW)", 5, y);
  y += 18;
 
  tft.drawLine(0, y, tft.width(), y, TFT_WHITE);
  y += 5;
 
  tft.fillRect(0, y, tft.width(), 20, TFT_BLUE);
  if (distance > 0 && distance < 400) {
    tft.drawString("DIST: " + String(distance) + " cm", 5, y);
  } else {
    tft.drawString("DIST: NO ECHO", 5, y);
  }
  y += 18;
 
  tft.drawLine(0, y, tft.width(), y, TFT_WHITE);
  y += 5;
 
  tft.fillRect(0, y, tft.width(), 55, TFT_BLUE);
  tft.drawString("ACCEL (g):", 5, y);
  y += 14;
  tft.drawString("X:" + String(accX, 2) + " Y:" + String(accY, 2) + " Z:" + String(accZ, 2), 5, y);
  y += 18;
 
  tft.drawString("GYRO (deg/s):", 5, y);
  y += 14;
  tft.drawString("X:" + String(gyroX, 1) + " Y:" + String(gyroY, 1) + " Z:" + String(gyroZ, 1), 5, y);
  y += 18;
 
  tft.drawLine(0, y, tft.width(), y, TFT_WHITE);
  y += 5;
 
  tft.fillRect(0, y, tft.width(), 15, TFT_BLUE);
  tft.drawString("TEMP: " + String(temperature, 1) + " C", 5, y);
 
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.drawString("PULSES: " + String(pulseCount), tft.width() / 2, tft.height() - 25);
  tft.drawString("PRESS BUTTON TO CALIBRATE", tft.width() / 2, tft.height() - 10);
  tft.setTextDatum(TL_DATUM);
}
 
void displayDrive() {
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextDatum(MC_DATUM);
 
  tft.fillRect(0, 0, tft.width(), 25, TFT_BLUE);
  tft.setTextSize(1);
  tft.drawString("STATE: DRIVE  |  PULSES: " + String(pulseCount), tft.width() / 2, 10);
 
  tft.drawLine(0, 22, tft.width(), 22, TFT_WHITE);
 
  tft.fillRect(0, 25, tft.width(), 60, TFT_BLUE);
  tft.setTextSize(4);
  tft.drawString("DRIVE", tft.width() / 2, 55);
 
  tft.drawLine(0, 85, tft.width(), 85, TFT_WHITE);
 
  tft.fillRect(0, 88, tft.width(), 70, TFT_BLUE);
  tft.setTextSize(2);
  if (distance > 0 && distance < 400) {
    tft.drawString("DIST: " + String(distance) + " cm", tft.width() / 2, 100);
  } else {
    tft.drawString("DIST: NO ECHO", tft.width() / 2, 100);
  }
 
  tft.setTextSize(1);
  tft.drawString("AUTO: DIST > 3cm -> STOP", tft.width() / 2, 130);
 
  if (distance > 3 && distance < 400) {
    tft.setTextColor(TFT_GREEN, TFT_BLUE);
    tft.drawString("THRESHOLD MET!", tft.width() / 2, 145);
  } else {
    tft.setTextColor(TFT_YELLOW, TFT_BLUE);
    tft.drawString("BELOW THRESHOLD", tft.width() / 2, 145);
  }
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
}
 
void displayStop(int stopNum) {
  tft.setTextColor(TFT_WHITE, TFT_RED);
  tft.setTextDatum(MC_DATUM);
 
  tft.fillRect(0, 0, tft.width(), 25, TFT_RED);
  tft.setTextSize(1);
  tft.drawString("STATE: STOP " + String(stopNum) + "  |  PULSES: " + String(pulseCount), tft.width() / 2, 10);
 
  tft.drawLine(0, 22, tft.width(), 22, TFT_WHITE);
 
  tft.fillRect(0, 25, tft.width(), 60, TFT_RED);
  tft.setTextSize(4);
  tft.drawString("STOP", tft.width() / 2, 55);
 
  tft.drawLine(0, 85, tft.width(), 85, TFT_WHITE);
 
  tft.fillRect(0, 88, tft.width(), 50, TFT_RED);
  tft.setTextSize(2);
  if (distance > 0 && distance < 400) {
    tft.drawString("DIST: " + String(distance) + " cm", tft.width() / 2, 105);
  } else {
    tft.drawString("DIST: NO ECHO", tft.width() / 2, 105);
  }
 
  if (stopNum == 1) {
    tft.setTextSize(1);
    tft.drawString("PRESS BUTTON -> TURN", tft.width() / 2, 140);
  } else {
    tft.setTextSize(1);
    tft.drawString("COMPLETE", tft.width() / 2, 140);
  }
}
 
void displayTurn() {
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextDatum(MC_DATUM);
 
  tft.fillRect(0, 0, tft.width(), 25, TFT_BLUE);
  tft.setTextSize(1);
  tft.drawString("STATE: TURN  |  PULSES: " + String(pulseCount), tft.width() / 2, 10);
 
  tft.drawLine(0, 22, tft.width(), 22, TFT_WHITE);
 
  tft.fillRect(0, 25, tft.width(), 60, TFT_BLUE);
  tft.setTextSize(4);
  tft.drawString("TURN", tft.width() / 2, 55);
 
  tft.drawLine(0, 85, tft.width(), 85, TFT_WHITE);
 
  tft.fillRect(0, 88, tft.width(), 70, TFT_BLUE);
  tft.setTextSize(2);
  tft.drawString("YAW: " + String(yawAngle, 1) + " deg", tft.width() / 2, 100);
 
  tft.setTextSize(1);
  tft.drawString("AUTO: YAW <= -90 -> STOP", tft.width() / 2, 130);
 
  float progress = abs(yawAngle) / 90.0;
  if (progress > 1.0) progress = 1.0;
  int barW = tft.width() - 40;
  tft.drawRect(20, 142, barW, 10, TFT_WHITE);
  tft.fillRect(22, 144, (int)((barW - 4) * progress), 6, TFT_GREEN);
}
 
void setup() {
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);
 
  Serial.begin(115200);
 
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATE_OUT_PIN, OUTPUT);
  pinMode(SLAVE_PIN, OUTPUT);
  digitalWrite(STATE_OUT_PIN, LOW);
  digitalWrite(SLAVE_PIN, LOW);
 
  initMPU6050();
 
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLUE);
}
 
void loop() {
  distance = readDistanceCM();
  readMPU6050();
 
  // === STATE 1: MEASUREMENTS ===
  if (currentState == WAITING) {
    displayState1();
 
    if (buttonPressed()) {
      sendPulse();
      currentState++;
      runCalibration();
      sendPulse();
      currentState++
      tft.fillScreen(TFT_BLUE);
    }
  }
 
  // === STATE 3: DRIVE ===
  else if (currentState == DRIVE) {
    displayDrive();
 
    if (distance > 6 && distance < 400) {
      sendPulse();
      currentState++;
      tft.fillScreen(TFT_RED);
    }
  }
 
  // === STATE 4: STOP 1 ===
  else if (currentState == STOPA) {
    displayStop(1);
 
    if (buttonPressed()) {
      sendPulse();
      yawAngle = 0;
      lastGyroTime = millis();
      currentState++;
      tft.fillScreen(TFT_BLUE);
    }
  }
 
  // === STATE 5: TURN ===
  else if (currentState == TURN) {
    displayTurn();
 
    // Turn left = negative yaw
    if (yawAngle <= -90.0) {
      sendPulse();
      currentState++;
      tft.fillScreen(TFT_RED);
    }
  }
 
  // === STATE 6: STOP 2 ===
  else if (currentState == DRIVERAMP) {
    displayStop(2);
  }
 
  delay(50);
}