// ===================================================
// MOTION 2350 PRO — SERVO CONTROL PANEL
//
// Debug / servo-tuning sketch. Flash this to the MOTION
// board instead of the main controller when you want to
// manually drive servos from your phone.
//
// *** Requires WiFi — only works on the Pico 2W variant ***
// *** (RP2350 + CYW43439 wireless chip)                 ***
//
// 1. Flash this sketch to the Motion board.
// 2. Connect your phone to WiFi "Robot-Motion" / "robot1234"
// 3. Open a browser and go to 192.168.4.1
//
// Controls available on the webpage:
//   - Extender servo (GP0): slider + EXTEND / STOP / RETRACT buttons
//   - Flippers (GP1 + GP3): single slider, GP3 auto-inverts
//   - Raw µs override: set each servo independently
//   - RGB LED colour picker
//   - ALL NEUTRAL button: stops everything
//   - Live IMU display: accel, gyro, temperature (auto-refresh)
//   - Action log: IMU snapshot before and after every command,
//                 downloadable as robot_actions.txt
// ===================================================

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <WebServer.h>

// ── WiFi access point credentials ───────────────────
const char* SSID = "Robot-Motion";
const char* PASS = "robot1234";

WebServer server(80);

// ── Pin assignments ──────────────────────────────────
const int EXTENDER_PIN = 0;    // GP0
const int FLIPPER_R    = 1;    // GP1
const int FLIPPER_L    = 3;    // GP3 — mounted inverted, always mirrored
const int BUZZER_PIN   = 22;
#define   RGB_PIN       28
#define   RGB_COUNT     1

// ── MPU-6050 IMU ─────────────────────────────────────
// Wiring per motion_2350_pro.md: SDA=GP20, SCL=GP21, addr=0x68
#define MPU_SDA   20
#define MPU_SCL   21
#define MPU_ADDR  0x68

// Scaled sensor readings (updated by readIMU())
float accX, accY, accZ;           // g
float gyroX, gyroY, gyroZ;        // °/s
float imuTemp;                     // °C
bool  imuReady = false;

// ── Servo pulse width limits (microseconds) ──────────
const int US_MIN = 1000;   // full speed one way
const int US_MID = 1500;   // stopped (neutral)
const int US_MAX = 2000;   // full speed other way

// ── Servo and LED objects ────────────────────────────
Servo extender, flipR, flipL;
Adafruit_NeoPixel rgb(RGB_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);

// ── Current servo positions (tracked for the UI) ────
int usExt   = 1500;
int usFlipR = 1500;
int usFlipL = 1500;

// ── Action log ───────────────────────────────────────
// Timestamped record of every command plus IMU snapshots.
// Capped at 6 KB — oldest lines are dropped when full.
#define LOG_MAX_BYTES 6144
String actionLog = "";

// ── IMU functions ─────────────────────────────────────

void initIMU() {
    Wire.begin(MPU_SDA, MPU_SCL);
    // Wake the chip (register 0x6B PWR_MGMT_1 = 0x00)
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
    // Accel full-scale ±2g  (register 0x1C = 0x00)
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
    // Gyro full-scale ±250°/s (register 0x1B = 0x00)
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
    imuReady = true;
}

// Read all 14 registers in one burst and convert to SI units.
// Accel LSB sensitivity: 16384 counts/g  (±2g range)
// Gyro  LSB sensitivity: 131  counts/°/s (±250°/s range)
// Temp formula from datasheet: T(°C) = raw/340 + 36.53
void readIMU() {
    if (!imuReady) return;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);   // start at ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);

    int16_t rAX = Wire.read() << 8 | Wire.read();
    int16_t rAY = Wire.read() << 8 | Wire.read();
    int16_t rAZ = Wire.read() << 8 | Wire.read();
    int16_t rT  = Wire.read() << 8 | Wire.read();
    int16_t rGX = Wire.read() << 8 | Wire.read();
    int16_t rGY = Wire.read() << 8 | Wire.read();
    int16_t rGZ = Wire.read() << 8 | Wire.read();

    accX    = rAX / 16384.0f;
    accY    = rAY / 16384.0f;
    accZ    = rAZ / 16384.0f;
    gyroX   = rGX / 131.0f;
    gyroY   = rGY / 131.0f;
    gyroZ   = rGZ / 131.0f;
    imuTemp = (rT  / 340.0f) + 36.53f;
}

// ── Logging helpers ───────────────────────────────────

void logAction(const String& description) {
    unsigned long ms   = millis();
    unsigned long secs = ms / 1000;
    unsigned long frac = ms % 1000;

    char timestamp[18];
    snprintf(timestamp, sizeof(timestamp), "[%4lus.%03lu] ", secs, frac);

    String entry = String(timestamp) + description + "\n";

    while (actionLog.length() + entry.length() > LOG_MAX_BYTES) {
        int cut = actionLog.indexOf('\n');
        if (cut < 0) { actionLog = ""; break; }
        actionLog = actionLog.substring(cut + 1);
    }

    actionLog += entry;
    Serial.print("[LOG] "); Serial.print(entry);
}

// Read the IMU and append a formatted snapshot line to the log.
// label should be "PRE " or "POST" (4 chars for alignment).
void logIMU(const char* label) {
    readIMU();

    char buf[120];
    snprintf(buf, sizeof(buf),
        "  %s IMU  — Accel: X=%+5.2fg Y=%+5.2fg Z=%+5.2fg"
        "  |  Gyro: X=%+6.1f Y=%+6.1f Z=%+6.1f °/s"
        "  |  Temp: %.1f°C",
        label,
        accX, accY, accZ,
        gyroX, gyroY, gyroZ,
        imuTemp);

    logAction(String(buf));
}

// ── Servo helpers ─────────────────────────────────────

void applyServos() {
    extender.writeMicroseconds(usExt);
    flipR.writeMicroseconds(usFlipR);
    flipL.writeMicroseconds(usFlipL);
}

// GP3 is back-to-back with GP1, so its direction is reversed.
// Mirroring around 1500 corrects: 1800↔1200, 1500 stays 1500.
int invertFlip(int us) {
    return 3000 - us;
}

String describeUs(int us) {
    if (us > 1550) return String(us) + "µs (fwd)";
    if (us < 1450) return String(us) + "µs (rev)";
    return "1500µs (stop)";
}

// ── Web page ─────────────────────────────────────────
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>Motion Control</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: Arial, sans-serif;
      background: #0d0d1a;
      color: #e0e0e0;
      padding: 12px;
    }
    h1 { color: #a29bfe; text-align: center; margin-bottom: 12px; font-size: 1.4em; }
    .card {
      background: #16213e;
      border-radius: 12px;
      padding: 14px;
      margin-bottom: 10px;
    }
    .card h2 {
      color: #a29bfe;
      font-size: 0.9em;
      text-transform: uppercase;
      margin-bottom: 10px;
      letter-spacing: 1px;
    }
    .row { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 8px; }
    .btn {
      border: none;
      border-radius: 8px;
      padding: 14px 10px;
      cursor: pointer;
      font-size: 0.9em;
      font-weight: bold;
      flex: 1;
    }
    .purple { background: #6c5ce7; color: #fff; }
    .orange { background: #e17055; color: #fff; }
    .grey   { background: #636e72; color: #fff; }
    .blue   { background: #0984e3; color: #fff; }
    .red    { background: #d63031; color: #fff; width: 100%; padding: 16px; font-size: 1.1em; }

    label { font-size: 0.85em; color: #aaa; margin-bottom: 3px; display: block; }
    input[type=range] { width: 100%; height: 32px; accent-color: #a29bfe; margin-bottom: 4px; }
    .status-bar {
      background: #0a0a1a;
      border-radius: 8px;
      padding: 8px;
      text-align: center;
      font-size: 0.85em;
      color: #a29bfe;
      margin-top: 6px;
    }
    .note { font-size: 0.75em; color: #636e72; margin-top: 4px; text-align: center; }
    input[type=number] {
      width: 100%;
      padding: 8px;
      background: #0a0a1a;
      color: #fff;
      border: 1px solid #444;
      border-radius: 6px;
    }

    /* IMU card */
    .imu-grid {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 8px;
      margin-bottom: 8px;
    }
    .imu-cell {
      background: #0a0a1a;
      border-radius: 8px;
      padding: 8px;
      text-align: center;
    }
    .imu-cell .axis { font-size: 0.72em; color: #636e72; margin-bottom: 2px; }
    .imu-cell .val  { font-size: 1.1em; color: #fdcb6e; font-weight: bold; font-family: monospace; }
    .imu-section-label {
      font-size: 0.78em;
      color: #a29bfe;
      text-transform: uppercase;
      letter-spacing: 1px;
      margin: 8px 0 4px 0;
    }
    .imu-temp {
      text-align: center;
      font-size: 0.85em;
      color: #aaa;
      margin-top: 6px;
    }

    /* Action log */
    #log_box {
      background: #0a0a1a;
      border-radius: 8px;
      padding: 10px;
      height: 260px;
      overflow-y: auto;
      font-family: monospace;
      font-size: 0.75em;
      color: #a8e6cf;
      white-space: pre-wrap;
      word-break: break-all;
      margin-bottom: 10px;
      border: 1px solid #2d3561;
    }
    .log-controls { display: flex; gap: 8px; }
    .log-note { font-size: 0.72em; color: #636e72; margin-top: 6px; text-align: center; }
  </style>
</head>
<body>

<h1>&#9881; Motion Control</h1>

<!-- Extender servo card -->
<div class='card'>
  <h2>Extender — GP0</h2>
  <label>Position: <span id='ext_label'>1500</span> µs</label>
  <input type='range' min='1000' max='2000' value='1500' id='ext_slider'
    oninput="document.getElementById('ext_label').innerText = this.value"
    onchange="setServo('ext', this.value)">
  <div class='row'>
    <button class='btn purple' onclick="setUs('ext', 1800)">EXTEND</button>
    <button class='btn grey'   onclick="setUs('ext', 1500)">STOP</button>
    <button class='btn orange' onclick="setUs('ext', 1200)">RETRACT</button>
  </div>
</div>

<!-- Flipper servos card -->
<div class='card'>
  <h2>Flippers — GP1 + GP3 (auto-inverted)</h2>
  <p class='note'>GP3 always mirrors GP1 — one command moves both flippers correctly.</p>
  <br>
  <label>
    Flipper: <span id='flip_label'>1500</span> µs (GP1)
           | <span id='flip_inv_label'>1500</span> µs (GP3 inverted)
  </label>
  <input type='range' min='1000' max='2000' value='1500' id='flip_slider'
    oninput="updateFlipLabels(this.value)"
    onchange="setServo('flip', this.value)">
  <div class='row'>
    <button class='btn purple' onclick="setUs('flip', 1650)">RAISE</button>
    <button class='btn grey'   onclick="setUs('flip', 1500)">STOP</button>
    <button class='btn orange' onclick="setUs('flip', 1350)">DUMP</button>
  </div>
</div>

<!-- Raw µs override card -->
<div class='card'>
  <h2>Individual Override (raw µs)</h2>
  <div class='row'>
    <div style='flex:1'>
      <label>GP0 — Extender</label>
      <input type='number' id='raw_ext'   value='1500' min='1000' max='2000'>
    </div>
    <div style='flex:1'>
      <label>GP1 — Flipper R</label>
      <input type='number' id='raw_flipR' value='1500' min='1000' max='2000'>
    </div>
    <div style='flex:1'>
      <label>GP3 — Flipper L</label>
      <input type='number' id='raw_flipL' value='1500' min='1000' max='2000'>
    </div>
  </div>
  <button class='btn purple' style='width:100%; margin-top:4px' onclick='sendRaw()'>SEND RAW</button>
</div>

<!-- LED colour picker card -->
<div class='card'>
  <h2>LED Colour</h2>
  <input type='color' id='led_colour' value='#282828'
    style='width:100%; height:40px; border:none; border-radius:8px; cursor:pointer'
    onchange='setLED(this.value)'>
</div>

<!-- All neutral + status bar -->
<div class='card'>
  <button class='btn red' onclick='neutral()'>ALL NEUTRAL (1500 µs)</button>
  <div class='status-bar' id='status'>Ready</div>
</div>

<!-- Live IMU card -->
<div class='card'>
  <h2>&#127916; Live IMU — MPU-6050</h2>

  <div class='imu-section-label'>Accelerometer (g)</div>
  <div class='imu-grid'>
    <div class='imu-cell'><div class='axis'>X</div><div class='val' id='aX'>—</div></div>
    <div class='imu-cell'><div class='axis'>Y</div><div class='val' id='aY'>—</div></div>
    <div class='imu-cell'><div class='axis'>Z</div><div class='val' id='aZ'>—</div></div>
  </div>

  <div class='imu-section-label'>Gyroscope (°/s)</div>
  <div class='imu-grid'>
    <div class='imu-cell'><div class='axis'>X</div><div class='val' id='gX'>—</div></div>
    <div class='imu-cell'><div class='axis'>Y</div><div class='val' id='gY'>—</div></div>
    <div class='imu-cell'><div class='axis'>Z</div><div class='val' id='gZ'>—</div></div>
  </div>

  <div class='imu-temp' id='imu_temp'>Temperature: —</div>
  <p class='note' style='margin-top:6px'>Auto-refreshes every 2 seconds</p>
</div>

<!-- Action log card -->
<div class='card'>
  <h2>&#128221; Action Log</h2>
  <div id='log_box'>(no actions yet)</div>
  <div class='log-controls'>
    <button class='btn blue' onclick='downloadLog()'>&#11015; Download .txt</button>
    <button class='btn grey' onclick='clearLog()'>Clear Log</button>
  </div>
  <p class='log-note'>
    Auto-refreshes every 3 s &bull; IMU snapshot taken before and after each command &bull;
    timestamps = seconds since boot
  </p>
</div>

<script>
  // ── Servo controls ──────────────────────────────────

  function setServo(servo, us) {
    us = parseInt(us);
    if (servo === 'ext') {
      document.getElementById('ext_slider').value = us;
      document.getElementById('ext_label').innerText = us;
    }
    if (servo === 'flip') {
      document.getElementById('flip_slider').value = us;
      updateFlipLabels(us);
    }
    fetch('/servo?s=' + servo + '&us=' + us)
      .then(r => r.text())
      .then(t => setStatus(t));
  }

  function setUs(servo, us) { setServo(servo, us); }

  function updateFlipLabels(us) {
    document.getElementById('flip_label').innerText = us;
    document.getElementById('flip_inv_label').innerText = 3000 - parseInt(us);
  }

  function sendRaw() {
    const ext   = document.getElementById('raw_ext').value;
    const flipR = document.getElementById('raw_flipR').value;
    const flipL = document.getElementById('raw_flipL').value;
    fetch('/raw?e=' + ext + '&r=' + flipR + '&l=' + flipL)
      .then(r => r.text())
      .then(t => setStatus(t));
  }

  function neutral() {
    fetch('/neutral').then(r => r.text()).then(t => setStatus(t));
  }

  function setLED(hex) {
    const r = parseInt(hex.slice(1, 3), 16);
    const g = parseInt(hex.slice(3, 5), 16);
    const b = parseInt(hex.slice(5, 7), 16);
    fetch('/led?r=' + r + '&g=' + g + '&b=' + b);
  }

  function setStatus(text) {
    document.getElementById('status').innerText = text;
  }

  // ── Live IMU display ────────────────────────────────
  // Polls /imu every 2 seconds and updates the six axis cells.
  // Response format (one value per line): aX\naY\naZ\ngX\ngY\ngZ\ntemp

  function refreshIMU() {
    fetch('/imu')
      .then(r => r.text())
      .then(text => {
        const parts = text.split('\n');
        if (parts.length < 7) return;
        document.getElementById('aX').innerText = parseFloat(parts[0]).toFixed(3);
        document.getElementById('aY').innerText = parseFloat(parts[1]).toFixed(3);
        document.getElementById('aZ').innerText = parseFloat(parts[2]).toFixed(3);
        document.getElementById('gX').innerText = parseFloat(parts[3]).toFixed(1);
        document.getElementById('gY').innerText = parseFloat(parts[4]).toFixed(1);
        document.getElementById('gZ').innerText = parseFloat(parts[5]).toFixed(1);
        document.getElementById('imu_temp').innerText =
          'Temperature: ' + parseFloat(parts[6]).toFixed(1) + ' °C';
      });
  }

  setInterval(refreshIMU, 2000);
  refreshIMU();

  // ── Action log ──────────────────────────────────────

  function refreshLog() {
    fetch('/log')
      .then(r => r.text())
      .then(text => {
        const box = document.getElementById('log_box');
        const wasAtBottom = box.scrollHeight - box.clientHeight <= box.scrollTop + 5;
        box.innerText = text;
        if (wasAtBottom) box.scrollTop = box.scrollHeight;
      });
  }

  function downloadLog() {
    fetch('/log')
      .then(r => r.text())
      .then(text => {
        const blob = new Blob([text], { type: 'text/plain' });
        const url  = URL.createObjectURL(blob);
        const a    = document.createElement('a');
        a.href     = url;
        a.download = 'robot_actions.txt';
        a.click();
        URL.revokeObjectURL(url);
      });
  }

  function clearLog() {
    fetch('/clearlog')
      .then(r => r.text())
      .then(t => {
        document.getElementById('log_box').innerText = '(log cleared)';
        setStatus(t);
      });
  }

  setInterval(refreshLog, 3000);
  refreshLog();
</script>

</body>
</html>
)rawliteral";

// ── HTTP route handlers ───────────────────────────────

void handleRoot() {
    server.send(200, "text/html", HTML);
}

// GET /servo?s=ext&us=1800  or  /servo?s=flip&us=1200
void handleServo() {
    String servo = server.arg("s");
    int    us    = constrain(server.arg("us").toInt(), US_MIN, US_MAX);

    // IMU snapshot before the command
    logIMU("PRE ");
    logAction("CMD  " + servo + "     — " +
              (servo == "ext" ? "Extender (GP0) → " + describeUs(us)
                              : "Flipper GP1 → " + describeUs(us) +
                                "  GP3 → " + describeUs(invertFlip(us)) + " (auto-inv)"));

    if (servo == "ext") {
        usExt = us;
    } else if (servo == "flip") {
        usFlipR = us;
        usFlipL = invertFlip(us);
    }
    applyServos();

    // Short pause then sample IMU so motion shows up in the reading
    delay(300);
    logIMU("POST");

    server.send(200, "text/plain",
        servo + " = " + String(us) + " µs  |  GP3 = " + String(usFlipL) + " µs");
}

// GET /raw?e=1500&r=1800&l=1200
void handleRaw() {
    int e = constrain(server.arg("e").toInt(), US_MIN, US_MAX);
    int r = constrain(server.arg("r").toInt(), US_MIN, US_MAX);
    int l = constrain(server.arg("l").toInt(), US_MIN, US_MAX);

    logIMU("PRE ");
    logAction("CMD  raw      — GP0: " + String(e) +
              "µs  GP1: " + String(r) +
              "µs  GP3: " + String(l) + "µs (no auto-invert)");

    usExt = e; usFlipR = r; usFlipL = l;
    applyServos();

    delay(300);
    logIMU("POST");

    server.send(200, "text/plain",
        "Raw — ext: " + String(usExt) +
        "  flipR: "   + String(usFlipR) +
        "  flipL: "   + String(usFlipL));
}

// GET /neutral
void handleNeutral() {
    logIMU("PRE ");
    logAction("CMD  neutral  — all servos → 1500µs");

    usExt = usFlipR = usFlipL = US_MID;
    applyServos();

    delay(300);
    logIMU("POST");

    server.send(200, "text/plain", "All servos → 1500 µs (neutral)");
}

// GET /led?r=255&g=0&b=128
void handleLED() {
    int r = server.arg("r").toInt();
    int g = server.arg("g").toInt();
    int b = server.arg("b").toInt();
    rgb.setPixelColor(0, rgb.Color(r, g, b));
    rgb.show();
    logAction("LED         — R:" + String(r) + " G:" + String(g) + " B:" + String(b));
    server.send(200, "text/plain", "LED OK");
}

// GET /imu  →  current IMU readings, one value per line
// aX, aY, aZ (g), gX, gY, gZ (°/s), temp (°C)
void handleIMU() {
    readIMU();
    char buf[120];
    snprintf(buf, sizeof(buf),
        "%.4f\n%.4f\n%.4f\n%.2f\n%.2f\n%.2f\n%.2f",
        accX, accY, accZ,
        gyroX, gyroY, gyroZ,
        imuTemp);
    server.send(200, "text/plain", buf);
}

// GET /log
void handleLog() {
    server.send(200, "text/plain",
        actionLog.length() ? actionLog : "(no actions recorded yet)");
}

// GET /clearlog
void handleClearLog() {
    actionLog = "";
    server.send(200, "text/plain", "Log cleared");
}

// ── Setup ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(BUZZER_PIN, OUTPUT);

    // Attach and centre all servos
    extender.attach(EXTENDER_PIN);
    flipR.attach(FLIPPER_R);
    flipL.attach(FLIPPER_L);
    applyServos();

    // LED: dim blue while starting up
    rgb.begin();
    rgb.setBrightness(100);
    rgb.setPixelColor(0, rgb.Color(10, 10, 40));
    rgb.show();

    // Initialise IMU
    initIMU();
    readIMU();   // first read to populate globals before any log entry

    // Start WiFi access point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, PASS);
    IPAddress ip = WiFi.softAPIP();

    // Register URL routes
    server.on("/",         handleRoot);
    server.on("/servo",    handleServo);
    server.on("/raw",      handleRaw);
    server.on("/neutral",  handleNeutral);
    server.on("/led",      handleLED);
    server.on("/imu",      handleIMU);
    server.on("/log",      handleLog);
    server.on("/clearlog", handleClearLog);
    server.begin();

    logAction("Board booted    — control panel ready at 192.168.4.1");
    logIMU("BOOT");

    // Two-tone startup beep
    tone(BUZZER_PIN, 880,  150); delay(200);
    tone(BUZZER_PIN, 1100, 150);

    Serial.print("AP SSID : "); Serial.println(SSID);
    Serial.print("AP IP   : "); Serial.println(ip);
    Serial.println("Control panel ready — open 192.168.4.1 in your browser");
}

// ── Loop ──────────────────────────────────────────────
void loop() {
    server.handleClient();
}
