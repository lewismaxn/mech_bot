// ===================================================
// T-DISPLAY S3 — UNIFIED MASTER / SLAVE FIRMWARE
//
// Role is selected at compile time via platformio.ini build_flags:
//   -DIS_MASTER  →  state machine, sensors, display
//   -DIS_SLAVE   →  motors, actuator, display
//
// Peer MAC bytes (PEER_MAC_0..5) are also injected via build_flags
// so swapping a board only requires editing platformio.ini.
//
// Communication:
//   Master ↔ Slave  : ESP-NOW  (replaces the old GPIO 17→18 pulse)
//   Master → Motion : GPIO 1 pulse (RP2350 has no Wi-Fi / ESP-NOW)
//
// ESP-NOW API note:
//   This file targets arduino-esp32 v3.x (ESP-IDF 5.x).
//   On v2.x change the onDataRecv signature to:
//     void onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
// ===================================================

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

#if !defined(IS_MASTER) && !defined(IS_SLAVE)
  #error "Set IS_MASTER or IS_SLAVE via build_flags in platformio.ini"
#endif

// ===================================================
// ESP-NOW — peer MAC from platformio.ini build_flags
// ===================================================
uint8_t peerMAC[6] = {
  PEER_MAC_0, PEER_MAC_1, PEER_MAC_2,
  PEER_MAC_3, PEER_MAC_4, PEER_MAC_5
};

struct StateMsg {
  uint8_t state;
};

// ===================================================
// DISPLAY
// ===================================================
#ifdef TFT_LIGHTGREY
  #undef TFT_LIGHTGREY
#endif
#define TFT_LIGHTGREY 0xC618

TFT_eSPI tft = TFT_eSPI();

// ===================================================
// STATE MACHINE — shared enum, both boards mirror each other
// ===================================================
enum Level {
  WAITING      =  0,
  CALIBRATION  =  1,
  FORWARD1     =  2,
  BACKWARD     =  3,
  FORWARD2     =  4,
  EXTEND       =  5,
  RAISEARM     =  6,
  TURN         =  7,
  FORWARD3     =  8,
  DEPOSIT      =  9,
  GOHOME       = 10,
  DONE         = 11
};

Level currentState = WAITING;

// ===================================================
// PIN DEFINITIONS
// ===================================================
#ifdef IS_MASTER
  // Ultrasonic A (front-facing)
  #define TRIG_A          2
  #define ECHO_A          3
  // Ultrasonic B (second)
  #define TRIG_B         43
  #define ECHO_B         44
  // MPU-6050 IMU
  #define MPU_SDA        16
  #define MPU_SCL        21
  #define MPU_ADDR     0x68
  // Button
  #define BUTTON_PIN     10
  // GPIO pulse out to Motion 2350 Pro (RP2350 — no ESP-NOW)
  #define STATE_OUT_PIN   1
  // Encoder motor (arm extend)
  #define EM_In1         11
  #define EM_In2         12
  #define EM_En          13

  // Timing constants
  #define FORWARD1_TIME   2000   // ms
  #define BACKWARD_TIME   2000
  #define FORWARD2_TIME   2000
  #define EXTEND_TIME     1500
  #define RAISE_ARM_TIME  8000
  #define TURN_SETTLE     500
  #define GOHOME_TIME     4000
  #define DEPOSIT_STOP_CM    6   // cm — ultrasonic stop threshold
  #define EXTEND_SPEED     255
  #define ENC_PWM_FREQ    1000
  #define ENC_PWM_RES        8

#else // IS_SLAVE

  // Left wheel motor — MD1 Secondary (U4), A1/2
  #define LM_In1  43
  #define LM_In2  44
  #define LM_En   10   // LEDC ch 0
  // Right wheel motor — MD2 Primary (U3)
  #define RM_In1  16
  #define RM_In2  21
  #define RM_En    3   // LEDC ch 2
  // Linear actuator — MD1 Secondary (U4), B1/2
  #define LA_In1   1
  #define LA_In2   2
  #define LA_En   11

  #define DRIVE_SPEED    700
  #define TURN_SPEED     500
  #define RM_CORRECTION  0.75f   // right motor slip compensation
  #define MTR_PWM_FREQ  1000
  #define MTR_PWM_RES     10     // 10-bit: 0–1023

#endif

// ===================================================
// MASTER-ONLY CODE
// ===================================================
#ifdef IS_MASTER

// ── Variables ──────────────────────────────────────
float accX, accY, accZ, gyroX, gyroY, gyroZ, temperature;
float accOffX, accOffY, accOffZ, gyroOffX, gyroOffY, gyroOffZ;
bool  calibrated    = false;
float yawAngle      = 0;
unsigned long lastGyroTime  = 0;
long  distA         = 0;
long  distB         = 0;
int   pulseCount    = 0;
unsigned long stateStartTime = 0;

// ── MPU-6050 ───────────────────────────────────────
void initMPU6050() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);

  int16_t rAX = Wire.read()<<8|Wire.read(), rAY = Wire.read()<<8|Wire.read(), rAZ = Wire.read()<<8|Wire.read();
  int16_t rT  = Wire.read()<<8|Wire.read();
  int16_t rGX = Wire.read()<<8|Wire.read(), rGY = Wire.read()<<8|Wire.read(), rGZ = Wire.read()<<8|Wire.read();

  accX  = rAX/16384.0f; accY  = rAY/16384.0f; accZ  = rAZ/16384.0f;
  temperature = rT/340.0f + 36.53f;
  gyroX = rGX/131.0f;   gyroY = rGY/131.0f;   gyroZ = rGZ/131.0f;

  if (calibrated) {
    accX-=accOffX; accY-=accOffY; accZ-=accOffZ;
    gyroX-=gyroOffX; gyroY-=gyroOffY; gyroZ-=gyroOffZ;
  }
  if (lastGyroTime > 0 && calibrated)
    yawAngle += gyroZ * (millis() - lastGyroTime) / 1000.0f;
  lastGyroTime = millis();
}

// ── Ultrasonic ────────────────────────────────────
long readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return pulseIn(echo, HIGH, 30000) * 0.034f / 2;
}

// ── Encoder motor ─────────────────────────────────
void encoderMotorSetup() {
  pinMode(EM_In1, OUTPUT); pinMode(EM_In2, OUTPUT);
  ledcSetup(4, ENC_PWM_FREQ, ENC_PWM_RES);
  ledcAttachPin(EM_En, 4);
  ledcWrite(4, 0);
}

void setEncoderMotor(int dir, int spd) {
  if      (dir ==  1) { digitalWrite(EM_In1, HIGH); digitalWrite(EM_In2, LOW);  }
  else if (dir == -1) { digitalWrite(EM_In1, LOW);  digitalWrite(EM_In2, HIGH); }
  else                { digitalWrite(EM_In1, LOW);  digitalWrite(EM_In2, LOW);  spd = 0; }
  ledcWrite(4, spd);
}

// ── Button ────────────────────────────────────────
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

// ── Display helpers ───────────────────────────────
void drawTopBar(String label, uint32_t colour) {
  tft.fillRect(0, 0, tft.width(), 25, colour);
  tft.setTextColor(TFT_WHITE, colour); tft.setTextDatum(MC_DATUM); tft.setTextSize(1);
  tft.drawString(label + "  |  PULSES: " + String(pulseCount), tft.width()/2, 10);
  tft.drawLine(0, 22, tft.width(), 22, TFT_BLACK);
}
void drawBigLabel(String label, uint32_t colour) {
  tft.fillRect(0, 25, tft.width(), 60, TFT_LIGHTGREY);
  tft.setTextColor(colour, TFT_LIGHTGREY); tft.setTextDatum(MC_DATUM); tft.setTextSize(4);
  tft.drawString(label, tft.width()/2, 55);
  tft.drawLine(0, 85, tft.width(), 85, TFT_BLACK);
}
void drawInfoLines(String l1, String l2) {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY); tft.setTextDatum(MC_DATUM); tft.setTextSize(1);
  tft.drawString(l1, tft.width()/2, 108);
  tft.drawString(l2, tft.width()/2, 128);
}
void refreshTimerBar(unsigned long dur) {
  float prog = constrain((float)(millis()-stateStartTime)/dur, 0.0f, 1.0f);
  int bw = tft.width()-40;
  unsigned long rem = dur - min((unsigned long)(millis()-stateStartTime), dur);
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY); tft.setTextDatum(MC_DATUM); tft.setTextSize(1);
  tft.drawString(String(rem/1000.0f, 1)+" s remaining", tft.width()/2, 108);
  tft.drawRect(20, 130, bw, 12, TFT_BLACK);
  tft.fillRect(22, 132, (int)((bw-4)*prog), 8, TFT_NAVY);
}
void refreshDistanceData(int threshold) {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY); tft.setTextDatum(MC_DATUM); tft.setTextSize(2);
  tft.drawString((distA>0&&distA<400)?"A: "+String(distA)+"cm":"A: NO ECHO", tft.width()/2, 100);
  tft.drawString((distB>0&&distB<400)?"B: "+String(distB)+"cm":"B: NO ECHO", tft.width()/2, 118);
  tft.setTextSize(1);
  tft.drawString("Stop (A) at: < "+String(threshold)+" cm", tft.width()/2, 138);
}
void refreshTurnData() {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY); tft.setTextDatum(MC_DATUM); tft.setTextSize(2);
  tft.drawString("YAW: "+String(yawAngle,1)+" deg", tft.width()/2, 105);
  tft.setTextSize(1); tft.drawString("Target: <= -90 deg", tft.width()/2, 130);
  float prog = constrain(abs(yawAngle)/90.0f, 0.0f, 1.0f);
  int bw = tft.width()-40;
  tft.drawRect(20, 142, bw, 10, TFT_BLACK);
  tft.fillRect(22, 144, (int)((bw-4)*prog), 6, TFT_DARKGREEN);
}
void refreshWaitingData() {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY); tft.setTextDatum(TL_DATUM); tft.setTextSize(1);
  int y = 92;
  tft.drawString("A: "+((distA>0&&distA<400)?String(distA)+" cm":String("NO ECHO")), 5, y);
  tft.drawString("B: "+((distB>0&&distB<400)?String(distB)+" cm":String("NO ECHO")), tft.width()/2, y);
  y += 16;
  tft.drawString("Acc X:"+String(accX,2)+" Y:"+String(accY,2)+" Z:"+String(accZ,2), 5, y); y+=16;
  tft.drawString("Gyr X:"+String(gyroX,1)+" Y:"+String(gyroY,1)+" Z:"+String(gyroZ,1), 5, y); y+=16;
  tft.drawString("Temp: "+String(temperature,1)+" C", 5, y);
}

// ── Calibration ───────────────────────────────────
void runCalibration() {
  const int SAMPLES = 200;
  float sAX=0,sAY=0,sAZ=0,sGX=0,sGY=0,sGZ=0;
  tft.fillScreen(TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK,TFT_LIGHTGREY); tft.setTextDatum(MC_DATUM); tft.setTextSize(2);
  tft.drawString("CALIBRATING", tft.width()/2, 30);
  tft.setTextSize(1); tft.drawString("KEEP ROBOT STILL", tft.width()/2, 55);
  int bx=20, by=75, bw=tft.width()-40, bh=16;
  tft.drawRect(bx, by, bw, bh, TFT_BLACK);
  for (int i = 0; i < SAMPLES; i++) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14,(bool)true);
    int16_t rAX=Wire.read()<<8|Wire.read(), rAY=Wire.read()<<8|Wire.read(), rAZ=Wire.read()<<8|Wire.read();
    Wire.read(); Wire.read();
    int16_t rGX=Wire.read()<<8|Wire.read(), rGY=Wire.read()<<8|Wire.read(), rGZ=Wire.read()<<8|Wire.read();
    sAX+=rAX/16384.0f; sAY+=rAY/16384.0f; sAZ+=rAZ/16384.0f;
    sGX+=rGX/131.0f;   sGY+=rGY/131.0f;   sGZ+=rGZ/131.0f;
    tft.fillRect(bx+2, by+2, map(i,0,SAMPLES-1,0,bw-4), bh-4, TFT_DARKGREEN);
    delay(10);
  }
  accOffX=sAX/SAMPLES; accOffY=sAY/SAMPLES; accOffZ=sAZ/SAMPLES-1.0f;
  gyroOffX=sGX/SAMPLES; gyroOffY=sGY/SAMPLES; gyroOffZ=sGZ/SAMPLES;
  calibrated=true; yawAngle=0; lastGyroTime=millis();

  tft.fillScreen(TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK,TFT_LIGHTGREY); tft.setTextDatum(TL_DATUM); tft.setTextSize(1);
  int y=10;
  tft.drawString("CALIBRATED OK",5,y); y+=18;
  tft.drawString("AccX:"+String(accOffX,4),5,y); y+=14;
  tft.drawString("AccY:"+String(accOffY,4),5,y); y+=14;
  tft.drawString("AccZ:"+String(accOffZ,4),5,y); y+=14;
  tft.drawString("GyrX:"+String(gyroOffX,4),5,y); y+=14;
  tft.drawString("GyrY:"+String(gyroOffY,4),5,y); y+=14;
  tft.drawString("GyrZ:"+String(gyroOffZ,4),5,y);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("PRESS BUTTON TO START", tft.width()/2, tft.height()-10);
  while (!buttonPressed()) delay(10);
}

// ── Comms ─────────────────────────────────────────
void sendStateToSlave(Level s) {
  StateMsg msg = { (uint8_t)s };
  esp_now_send(peerMAC, (uint8_t*)&msg, sizeof(msg));
}

void pulseMotionBoard() {
  digitalWrite(STATE_OUT_PIN, HIGH);
  delay(100);
  digitalWrite(STATE_OUT_PIN, LOW);
  pulseCount++;
  Serial.print("PULSE #"); Serial.println(pulseCount);
}

void advanceState() {
  pulseMotionBoard();
  currentState = (Level)(currentState + 1);
  sendStateToSlave(currentState);
  stateStartTime = millis();
  Serial.print("-> STATE "); Serial.println((int)currentState);
}

// ── ESP-NOW send callback ─────────────────────────
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS)
    Serial.println("ESP-NOW TX fail");
}

#endif // IS_MASTER

// ===================================================
// SLAVE-ONLY CODE
// ===================================================
#ifdef IS_SLAVE

// ── Motor structs ─────────────────────────────────
struct Motor     { int in1, in2, en, ch; };
struct LinearAct { int in1, in2, en; };

Motor     leftMotor  = { LM_In1, LM_In2, LM_En, 0 };
Motor     rightMotor = { RM_In1, RM_In2, RM_En, 2 };
LinearAct actuator   = { LA_In1, LA_In2, LA_En    };

// ── Motor functions ───────────────────────────────
void motorSetup(Motor m) {
  pinMode(m.in1, OUTPUT); pinMode(m.in2, OUTPUT);
  ledcSetup(m.ch, MTR_PWM_FREQ, MTR_PWM_RES);
  ledcAttachPin(m.en, m.ch);
  ledcWrite(m.ch, 0);
}
void setMotor(Motor m, int dir, int spd) {
  if      (dir ==  1) { digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW);  }
  else if (dir == -1) { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH); }
  else                { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, LOW);  spd = 0; }
  ledcWrite(m.ch, spd);
}
void setActuator(LinearAct l, int dir) {
  if      (dir ==  1) { digitalWrite(l.in1,HIGH); digitalWrite(l.in2,LOW);  digitalWrite(l.en,HIGH); }
  else if (dir == -1) { digitalWrite(l.in1,LOW);  digitalWrite(l.in2,HIGH); digitalWrite(l.en,HIGH); }
  else                { digitalWrite(l.in1,LOW);  digitalWrite(l.in2,LOW);  digitalWrite(l.en,LOW);  }
}
void stopMotors()   { setMotor(leftMotor,0,0); setMotor(rightMotor,0,0); }
void stopActuator() { setActuator(actuator,0); }
void stopAll()      { stopMotors(); stopActuator(); }

// Right motor physically inverted — direction always opposite to left
void driveForward(int spd)  { setMotor(leftMotor,-1,spd); setMotor(rightMotor, 1,(int)(spd*RM_CORRECTION)); }
void driveBackward(int spd) { setMotor(leftMotor, 1,spd); setMotor(rightMotor,-1,(int)(spd*RM_CORRECTION)); }
void driveTurn(int spd)     { setMotor(leftMotor, 1,(int)(spd*0.5f)); setMotor(rightMotor,1,(int)(spd*RM_CORRECTION)); }

// ── Display ───────────────────────────────────────
void showState(String label, uint32_t colour) {
  tft.fillRect(0, 0, tft.width(), 25, colour);
  tft.setTextColor(TFT_WHITE, colour); tft.setTextDatum(MC_DATUM); tft.setTextSize(1);
  tft.drawString("SLAVE  STATE: "+String((int)currentState), tft.width()/2, 12);
  tft.fillRect(0, 25, tft.width(), 145, TFT_LIGHTGREY);
  tft.setTextColor(colour, TFT_LIGHTGREY); tft.setTextSize(4);
  tft.drawString(label, tft.width()/2, 95);
}

// ── State entry ───────────────────────────────────
void enterState(Level s) {
  Serial.print("STATE -> "); Serial.println((int)s);
  switch (s) {
    case WAITING:     showState("WAIT",  TFT_DARKGREY);  stopAll();                                  break;
    case CALIBRATION: showState("CAL",   TFT_DARKCYAN);  stopAll();                                  break;
    case FORWARD1:    showState("FWD1",  TFT_NAVY);      stopActuator(); driveForward(DRIVE_SPEED);  break;
    case BACKWARD:    showState("BACK",  TFT_NAVY);      stopActuator(); driveBackward(DRIVE_SPEED); break;
    case FORWARD2:    showState("FWD2",  TFT_NAVY);      stopActuator(); driveForward(DRIVE_SPEED);  break;
    case EXTEND:      showState("EXTND", TFT_PURPLE);    stopAll();                                  break;
    case RAISEARM:    showState("RAISE", TFT_PURPLE);    stopMotors(); setActuator(actuator,1);      break;
    case TURN:        showState("TURN",  TFT_NAVY);      stopActuator(); driveTurn(TURN_SPEED);      break;
    case FORWARD3:    showState("FWD3",  TFT_NAVY);      stopActuator(); driveForward(DRIVE_SPEED);  break;
    case DEPOSIT:     showState("DUMP",  TFT_ORANGE);    stopAll();                                  break;
    case GOHOME:      showState("HOME",  TFT_DARKGREEN); stopActuator(); driveBackward(DRIVE_SPEED); break;
    case DONE:
    default:          showState("DONE",  TFT_DARKGREEN); stopAll();                                  break;
  }
}

// ── ESP-NOW state buffer (written from WiFi task, read from loop) ─
volatile bool  newStateFlag = false;
volatile Level pendingState = WAITING;

// ESP-NOW receive callback — arduino-esp32 v3.x signature
// For v2.x replace with: void onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(StateMsg)) return;
  StateMsg msg;
  memcpy(&msg, data, sizeof(msg));
  pendingState = (Level)msg.state;
  newStateFlag = true;
}

#endif // IS_SLAVE

// ===================================================
// ESP-NOW INIT — shared
// ===================================================
void initESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed — halting");
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setTextDatum(MC_DATUM); tft.setTextSize(2);
    tft.drawString("ESP-NOW FAIL", tft.width()/2, tft.height()/2);
    while (true) delay(1000);
  }

#ifdef IS_MASTER
  esp_now_register_send_cb(onDataSent);
#else
  esp_now_register_recv_cb(onDataRecv);
#endif

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK)
    Serial.println("ESP-NOW add peer failed");

  Serial.printf("ESP-NOW ready — peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
    peerMAC[0],peerMAC[1],peerMAC[2],peerMAC[3],peerMAC[4],peerMAC[5]);
}

// ===================================================
// SETUP
// ===================================================
void setup() {
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);   // TFT backlight

  Serial.begin(115200);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_LIGHTGREY);

  initESPNow();

#ifdef IS_MASTER
  pinMode(TRIG_A,        OUTPUT);
  pinMode(ECHO_A,        INPUT);
  pinMode(TRIG_B,        OUTPUT);
  pinMode(ECHO_B,        INPUT);
  pinMode(BUTTON_PIN,    INPUT_PULLUP);
  pinMode(STATE_OUT_PIN, OUTPUT);
  digitalWrite(STATE_OUT_PIN, LOW);

  initMPU6050();
  encoderMotorSetup();

  drawTopBar("WAITING", TFT_DARKGREY);
  drawBigLabel("WAIT",  TFT_DARKGREY);
  drawInfoLines("Sensors live", "PRESS BUTTON to calibrate");

  stateStartTime = millis();
  Serial.println("MASTER v2 ready — WAITING  [ESP-NOW active]");

#else // IS_SLAVE
  motorSetup(leftMotor);
  motorSetup(rightMotor);
  pinMode(LA_In1, OUTPUT); pinMode(LA_In2, OUTPUT); pinMode(LA_En, OUTPUT);
  stopAll();

  enterState(WAITING);
  Serial.println("SLAVE v2 ready — awaiting ESP-NOW from master");
#endif
}

// ===================================================
// LOOP
// ===================================================
void loop() {

// ─── MASTER ────────────────────────────────────────
#ifdef IS_MASTER

  distA = readUltrasonic(TRIG_A, ECHO_A);
  distB = readUltrasonic(TRIG_B, ECHO_B);
  readMPU6050();

  if (currentState == WAITING) {
    refreshWaitingData();
    if (buttonPressed()) {
      advanceState();           // → CALIBRATION (1)
      runCalibration();
      advanceState();           // → FORWARD1 (2)
      tft.fillScreen(TFT_LIGHTGREY);
      drawTopBar("FORWARD 1", TFT_NAVY);
      drawBigLabel("FWD",     TFT_NAVY);
    }
  }
  else if (currentState == FORWARD1) {
    refreshTimerBar(FORWARD1_TIME);
    if (millis()-stateStartTime >= FORWARD1_TIME) {
      advanceState();           // → BACKWARD (3)
      tft.fillScreen(TFT_LIGHTGREY); drawTopBar("BACKWARD",TFT_NAVY); drawBigLabel("BACK",TFT_NAVY);
    }
  }
  else if (currentState == BACKWARD) {
    refreshTimerBar(BACKWARD_TIME);
    if (millis()-stateStartTime >= BACKWARD_TIME) {
      advanceState();           // → FORWARD2 (4)
      tft.fillScreen(TFT_LIGHTGREY); drawTopBar("FORWARD 2",TFT_NAVY); drawBigLabel("FWD",TFT_NAVY);
    }
  }
  else if (currentState == FORWARD2) {
    refreshTimerBar(FORWARD2_TIME);
    if (millis()-stateStartTime >= FORWARD2_TIME) {
      advanceState();           // → EXTEND (5)
      tft.fillScreen(TFT_LIGHTGREY); drawTopBar("EXTEND ARM",TFT_PURPLE); drawBigLabel("EXTND",TFT_PURPLE);
    }
  }
  else if (currentState == EXTEND) {
    setEncoderMotor(1, EXTEND_SPEED);
    refreshTimerBar(EXTEND_TIME);
    if (millis()-stateStartTime >= EXTEND_TIME) {
      setEncoderMotor(0, 0);
      advanceState();           // → RAISEARM (6)
      tft.fillScreen(TFT_LIGHTGREY); drawTopBar("RAISE ARM",TFT_PURPLE); drawBigLabel("RAISE",TFT_PURPLE);
      drawInfoLines("Raising arm", String(RAISE_ARM_TIME/1000)+" s");
    }
  }
  else if (currentState == RAISEARM) {
    refreshTimerBar(RAISE_ARM_TIME);
    if (millis()-stateStartTime >= RAISE_ARM_TIME) {
      advanceState();           // → TURN (7)
      yawAngle=0; lastGyroTime=millis();
      tft.fillScreen(TFT_LIGHTGREY); drawTopBar("TURN",TFT_NAVY); drawBigLabel("TURN",TFT_NAVY);
    }
  }
  else if (currentState == TURN) {
    refreshTurnData();
    if (yawAngle <= -90.0f) {
      delay(TURN_SETTLE);
      advanceState();           // → FORWARD3 (8)
      tft.fillScreen(TFT_LIGHTGREY); drawTopBar("FORWARD 3",TFT_NAVY); drawBigLabel("FWD",TFT_NAVY);
    }
  }
  else if (currentState == FORWARD3) {
    refreshDistanceData(DEPOSIT_STOP_CM);
    if (distA > 0 && distA <= DEPOSIT_STOP_CM) {
      advanceState();           // → DEPOSIT (9)
      tft.fillScreen(TFT_LIGHTGREY); drawTopBar("DEPOSIT",TFT_ORANGE); drawBigLabel("DUMP",TFT_ORANGE);
      drawInfoLines("Depositing", "TODO: mechanism");
    }
  }
  else if (currentState == DEPOSIT) {
    // TODO: run deposit servo/motor here
    delay(500);
    advanceState();             // → GOHOME (10)
    tft.fillScreen(TFT_LIGHTGREY); drawTopBar("GO HOME",TFT_DARKGREEN); drawBigLabel("HOME",TFT_DARKGREEN);
  }
  else if (currentState == GOHOME) {
    refreshTimerBar(GOHOME_TIME);
    if (millis()-stateStartTime >= GOHOME_TIME) {
      pulseMotionBoard();       // final pulse to Motion board
      sendStateToSlave(DONE);   // tell slave to stop motors
      tft.fillScreen(TFT_LIGHTGREY);
      drawTopBar("COMPLETE", TFT_DARKGREEN);
      drawBigLabel("DONE",   TFT_DARKGREEN);
      drawInfoLines("Mission complete", "Pulses sent: "+String(pulseCount));
      Serial.println("MISSION COMPLETE");
      while (true) delay(1000);
    }
  }

  delay(50);

// ─── SLAVE ─────────────────────────────────────────
#else

  // All state changes arrive via ESP-NOW callback.
  // The flag+value pattern avoids calling enterState() from the WiFi task.
  if (newStateFlag) {
    newStateFlag = false;
    currentState = pendingState;
    enterState(currentState);
  }

  delay(5);

#endif
}
