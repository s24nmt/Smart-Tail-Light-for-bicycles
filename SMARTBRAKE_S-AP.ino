#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"

// Pin Definitions matching XIAO ESP32C6 layout
#define BATTERY_PIN  D0      // D0 (ADC) पर 150k + 150k वोल्टेज डिवाइडर
#define BUTTON3_PIN  D1      // D1 लाइट को ON/OFF टॉगल (Deep Sleep) करने के लिए (LP GPIO1)
#define LED_PIN      D2      // D2 WS2812B डेटा के लिए
#define BUTTON2_PIN  D3      // D3 On-Demand Battery Gauge (Short Press) / SmartBrake Soft AP (Long Press)
#define BUTTON_PIN   D6      // D6 पुश बटन (मोड बदलने) के लिए
#define BUTTON4_PIN  D7      // D7 ब्राइटनेस लेवल (Low, Medium, High) बदलने के लिए
#define NUM_LEDS     8       // 8 LEDs

// MPU6050 I2C Address
const int MPU_addr = 0x68;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
Preferences prefs;

// Soft AP & Captive Portal Objects
WebServer server(80);
DNSServer dnsServer;
const IPAddress apIP(192, 168, 4, 1);
const IPAddress netMsk(255, 255, 255, 0);
const char* AP_SSID = "SmartBrake";
bool isSoftAPActive = false;
bool webRoutesInitialized = false;
bool pendingStopAP = false;
unsigned long stopAPRequestTime = 0;

// Web UI से Brightness और Battery LED गेज चलाने के लिए फ्लैग्स
bool pendingBrightnessGauge = false;
bool pendingBatteryGauge = false;
unsigned long gaugeRequestTime = 0;

// Web UI Mode बटन दबाने पर 2 सेकंड के रियल-टाइम LED प्रीव्यू के लिए
bool isModePreviewActive = false;
unsigned long modePreviewStartTime = 0;
const unsigned long modePreviewDuration = 2000;

// Lighting Modes: 1 to 10 - Flash (NVS) में सेव
int currentMode = 1; 
const int totalModes = 10; // कुल 10 मोड्स

// Brightness Levels: 1 = Low, 2 = Medium, 3 = High - Flash (NVS) में सेव
int brightnessLevel = 3; // डिफ़ॉल्ट High

// Motion, Timeout & Power Control
unsigned long lastMotionTime = 0;
const int DEFAULT_TIMEOUT_SEC = 30;
int timeoutSeconds = DEFAULT_TIMEOUT_SEC;  // Web UI से एडजस्टेबल (0 = Off, 1-30s)
unsigned long timeoutDuration = 30000;
bool lightsActive = true;
bool systemPower = true; // D1 बटन के लिए ON/OFF फ्लैग

// --- उन्नत ब्रेकिंग डिटेक्शन वेरिएबल्स (Balanced Normal + Hard Braking) ---
const float BRAKE_DIRECTION = -1.0;        
const float DEFAULT_BRAKE_THRESHOLD = 0.18;
const float DEFAULT_BRAKE_RELEASE_THRESHOLD = 0.06;
const int DEFAULT_REQUIRED_BRAKE_COUNT = 7;

float brakeThreshold = DEFAULT_BRAKE_THRESHOLD;               // Web UI से एडजस्टेबल (0.10 to 0.22)
float brakeReleaseThreshold = DEFAULT_BRAKE_RELEASE_THRESHOLD; // Web UI से एडजस्टेबल (0.04 to 0.08)
int requiredBrakeCount = DEFAULT_REQUIRED_BRAKE_COUNT;         // Web UI से एडजस्टेबल (4 to 10)
const float maxVerticalShock = 0.70;       // गड्ढों और झटकों को फ़िल्टर करने के लिए
const float maxPitchRate = 70.0;           // सस्पेंशन दबने (fork dive) पर ब्लॉक न हो (पहले 45.0)

bool isBraking = false;
unsigned long brakeStartTime = 0;
const unsigned long maxBrakeHoldTime = 2500; 
const unsigned long minBrakeHoldTime = 800;  
int brakeCounter = 0;

// Battery Monitoring Variables
int batteryPercent = 100;
float batteryVoltage = 4.2;
bool isLowBattery = false;
unsigned long lastBatteryCheck = 0;
const unsigned long batteryCheckInterval = 10000;

// Button Debounce Variables (Mode Button - D6)
int buttonState = HIGH;          
int lastButtonState = HIGH;      
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Button 2 Debounce & Long Press Variables (Battery Gauge / Soft AP Button - D3)
int button2State = HIGH;
int lastButton2State = HIGH;
unsigned long lastDebounceTime2 = 0;
unsigned long button2PressStartTime = 0;
bool button2LongPressTriggered = false;
const unsigned long longPressDuration = 2000; // Soft AP के लिए 2 सेकंड लॉन्ग प्रेस

// Button 3 Debounce Variables (ON/OFF Button - D1)
int button3State = HIGH;          
int lastButton3State = HIGH;      
unsigned long lastDebounceTime3 = 0;

// Button 4 Debounce Variables (Brightness Button - D7)
int button4State = HIGH;
int lastButton4State = HIGH;
unsigned long lastDebounceTime4 = 0;

// Sensor Readings & Filtering Variables
int16_t ax, ay, az, gx, gy, gz;
float prevAccelX = 0;
float prevAccelY = 0; 
float estimatedPitch = 0;
float estimatedRoll = 0;                        // झुकाव (Roll) ट्रैक करने के लिए
float filteredAx = 0;
float baselineAx = 0;   
float gravityZ = 1.0;   
float gxOffset = 0, gyOffset = 0, gzOffset = 0; // Gyro bias offsets
unsigned long lastSensorTime = 0;

// Turn Filtering & Brake Safety Variables
const float maxYawRate = 35.0;   // तेज़ Left/Right मोड़ (deg/s) फ़िल्टर करने के लिए
const float maxRollRate = 55.0;  // तेज़ झुकाव (lean) फ़िल्टर करने के लिए
bool waitForBrakeRelease = false; // maxBrakeHoldTime के बाद तुरंत री-ट्रिगर रोकने के लिए

// I2C Error & Recovery Counter
int i2cErrorCount = 0;

const float motionThreshold = 0.05; 

// Mobile-Friendly Web UI HTML
const char WEB_UI_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SmartBrake Settings</title>
<style>
- { box-sizing: border-box; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
  body { margin: 0; padding: 16px; background: #0f172a; color: #f8fafc; display: flex; justify-content: center; }
  .card { width: 100%; max-width: 420px; background: #1e293b; border-radius: 16px; padding: 20px; box-shadow: 0 10px 25px rgba(0,0,0,0.4); }
  h1 { margin: 0 0 6px; font-size: 22px; text-align: center; color: #38bdf8; }
  .sub { margin: 0 0 20px; font-size: 13px; text-align: center; color: #94a3b8; }
  .ctrl-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; margin-bottom: 20px; }
  .btn-ctrl { background: #334155; padding: 10px 6px; font-size: 13px; border-radius: 10px; display: flex; flex-direction: column; align-items: center; gap: 4px; }
  .btn-ctrl span { font-size: 12px; color: #38bdf8; font-weight: 700; }
  .group { margin-bottom: 20px; background: #0f172a; padding: 14px; border-radius: 12px; }
  .row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
  .label { font-size: 14px; font-weight: 600; }
  .val { font-size: 15px; font-weight: 700; color: #38bdf8; background: #1e293b; padding: 3px 10px; border-radius: 8px; }
  input[type=range] { width: 100%; accent-color: #38bdf8; height: 8px; cursor: pointer; }
  .range-ends { display: flex; justify-content: space-between; font-size: 11px; color: #64748b; margin-top: 4px; }
  .btns { display: flex; flex-direction: column; gap: 10px; margin-top: 22px; }
  button { width: 100%; padding: 13px; border: none; border-radius: 10px; font-size: 15px; font-weight: 600; cursor: pointer; color: #fff; }
  .btn-save { background: #16a34a; }
  .btn-default { background: #2563eb; }
  .btn-off { background: #dc2626; }
  button:active { opacity: 0.85; transform: scale(0.99); }
  #status { margin-top: 14px; text-align: center; font-size: 14px; min-height: 20px; color: #4ade80; font-weight: 500; }
</style>
</head>
<body>
<div class="card">
  <h1>SmartBrake</h1>
  <p class="sub">Brake Sensitivity Settings</p>

  <div class="ctrl-grid">
    <button class="btn-ctrl" onclick="changeMode()">Mode<span id="modeVal">%MODE% / 10</span></button>
    <button class="btn-ctrl" onclick="changeBrightness()">Brightness<span id="brightVal">%BRIGHT%</span></button>
    <button class="btn-ctrl" onclick="checkBattery()">Battery<span id="batVal">%BAT%%</span></button>
  </div>

  <div class="group">
    <div class="row">
      <span class="label">Brake Threshold</span>
      <span class="val" id="thrVal">%THR%</span>
    </div>
    <input type="range" id="thr" min="0.10" max="0.22" step="0.01" value="%THR%" oninput="document.getElementById('thrVal').innerText=parseFloat(this.value).toFixed(2)">
    <div class="range-ends"><span>0.10 (Sensitive)</span><span>0.22 (Hard)</span></div>
  </div>

  <div class="group">
    <div class="row">
      <span class="label">Brake Release Threshold</span>
      <span class="val" id="relVal">%REL%</span>
    </div>
    <input type="range" id="rel" min="0.04" max="0.08" step="0.01" value="%REL%" oninput="document.getElementById('relVal').innerText=parseFloat(this.value).toFixed(2)">
    <div class="range-ends"><span>0.04</span><span>0.08</span></div>
  </div>

  <div class="group">
    <div class="row">
      <span class="label">Required Brake Count</span>
      <span class="val" id="cntVal">%CNT%</span>
    </div>
    <input type="range" id="cnt" min="4" max="10" step="1" value="%CNT%" oninput="document.getElementById('cntVal').innerText=this.value">
    <div class="range-ends"><span>4 (Faster)</span><span>10 (Strict)</span></div>
  </div>

  <div class="group">
    <div class="row">
      <span class="label">Auto-Off Timeout</span>
      <span class="val" id="tmoVal">%TMO_LBL%</span>
    </div>
    <input type="range" id="tmo" min="0" max="30" step="1" value="%TMO%" oninput="document.getElementById('tmoVal').innerText=(this.value==0?'0s (Off)':this.value+'s')">
    <div class="range-ends"><span>0s (Off)</span><span>30s</span></div>
  </div>

  <div class="btns">
    <button class="btn-save" onclick="saveSettings()">Save Settings</button>
    <button class="btn-default" onclick="resetDefaults()">Default Settings</button>
    <button class="btn-off" onclick="turnOffAP()">Turn Off Soft AP</button>
  </div>
  <div id="status"></div>
</div>

<script>
function setStatus(msg, color) {
  const s = document.getElementById('status');
  s.style.color = color || '#4ade80';
  s.innerText = msg;
}
function changeMode() {
  fetch('/mode')
    .then(r => r.json())
    .then(d => {
      document.getElementById('modeVal').innerText = d.mode + ' / 10';
      setStatus('Switched to Mode ' + d.mode, '#4ade80');
    })
    .catch(() => setStatus('Error changing mode', '#f87171'));
}
function changeBrightness() {
  fetch('/brightness')
    .then(r => r.json())
    .then(d => {
      document.getElementById('brightVal').innerText = d.name;
      setStatus('Brightness: ' + d.name, '#38bdf8');
    })
    .catch(() => setStatus('Error changing brightness', '#f87171'));
}
function checkBattery() {
  fetch('/battery')
    .then(r => r.json())
    .then(d => {
      document.getElementById('batVal').innerText = d.pct + '%';
      setStatus(`Battery: ${d.pct}% (${d.volt}V)`, '#fbbf24');
    })
    .catch(() => setStatus('Error reading battery', '#f87171'));
}
function saveSettings() {
  const thr = document.getElementById('thr').value;
  const rel = document.getElementById('rel').value;
  const cnt = document.getElementById('cnt').value;
  const tmo = document.getElementById('tmo').value;
  fetch(`/save?thr=${thr}&rel=${rel}&cnt=${cnt}&tmo=${tmo}`)
    .then(r => r.text())
    .then(() => setStatus('Settings Saved!', '#4ade80'))
    .catch(() => setStatus('Error saving settings', '#f87171'));
}
function resetDefaults() {
  fetch('/default')
    .then(r => r.json())
    .then(d => {
      document.getElementById('thr').value = d.thr.toFixed(2);
      document.getElementById('thrVal').innerText = d.thr.toFixed(2);
      document.getElementById('rel').value = d.rel.toFixed(2);
      document.getElementById('relVal').innerText = d.rel.toFixed(2);
      document.getElementById('cnt').value = d.cnt;
      document.getElementById('cntVal').innerText = d.cnt;
      document.getElementById('tmo').value = d.tmo;
      document.getElementById('tmoVal').innerText = (d.tmo == 0 ? '0s (Off)' : d.tmo + 's');
      setStatus('Default Settings Restored & Saved!', '#38bdf8');
    })
    .catch(() => setStatus('Error resetting defaults', '#f87171'));
}
function turnOffAP() {
  fetch('/stop')
    .then(() => setStatus('Soft AP Turning Off... You can close this page.', '#fbbf24'))
    .catch(() => setStatus('Soft AP Turned Off.', '#fbbf24'));
}
</script>
</body>
</html>
)rawliteral";

// Forward Declarations
void colorAll(uint8_t r, uint8_t g, uint8_t b);
void stopSoftAP();
void readBattery();
void showStartupBattery();
void showBrightnessGauge();

void saveBrakeSettingsToNVS() {
  prefs.begin("taillight", false);
  prefs.putFloat("brkThr", brakeThreshold);
  prefs.putFloat("brkRel", brakeReleaseThreshold);
  prefs.putInt("brkCnt", requiredBrakeCount);
  prefs.putInt("tmOut", timeoutSeconds);
  prefs.end();
}

void handleRoot() {
  readBattery();
  String html = FPSTR(WEB_UI_HTML);
  String brightName = (brightnessLevel == 1) ? "Low" : ((brightnessLevel == 2) ? "Medium" : "High");
  html.replace("%MODE%", String(currentMode));
  html.replace("%BRIGHT%", brightName);
  html.replace("%BAT%", String(batteryPercent));
  html.replace("%THR%", String(brakeThreshold, 2));
  html.replace("%REL%", String(brakeReleaseThreshold, 2));
  html.replace("%CNT%", String(requiredBrakeCount));
  html.replace("%TMO%", String(timeoutSeconds));
  html.replace("%TMO_LBL%", timeoutSeconds == 0 ? "0s (Off)" : String(timeoutSeconds) + "s");
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.send(200, "text/html", html);
}

void handleSave() {
  if (server.hasArg("thr")) {
    brakeThreshold = constrain(server.arg("thr").toFloat(), 0.10f, 0.22f);
  }
  if (server.hasArg("rel")) {
    brakeReleaseThreshold = constrain(server.arg("rel").toFloat(), 0.04f, 0.08f);
  }
  if (server.hasArg("cnt")) {
    requiredBrakeCount = constrain(server.arg("cnt").toInt(), 4, 10);
  }
  if (server.hasArg("tmo")) {
    timeoutSeconds = constrain(server.arg("tmo").toInt(), 0, 30);
    timeoutDuration = (unsigned long)timeoutSeconds * 1000UL;
  }
  saveBrakeSettingsToNVS();
  Serial.printf("Saved Brake Settings -> Threshold: %.2f, Release: %.2f, Count: %d, Timeout: %ds\n",
                brakeThreshold, brakeReleaseThreshold, requiredBrakeCount, timeoutSeconds);
  server.send(200, "text/plain", "OK");
}

void handleDefault() {
  brakeThreshold = DEFAULT_BRAKE_THRESHOLD;
  brakeReleaseThreshold = DEFAULT_BRAKE_RELEASE_THRESHOLD;
  requiredBrakeCount = DEFAULT_REQUIRED_BRAKE_COUNT;
  timeoutSeconds = DEFAULT_TIMEOUT_SEC;
  timeoutDuration = (unsigned long)timeoutSeconds * 1000UL;
  saveBrakeSettingsToNVS();
  Serial.println("Brake Settings reset to defaults.");
  String json = "{\"thr\":" + String(brakeThreshold, 2) +
                ",\"rel\":" + String(brakeReleaseThreshold, 2) +
                ",\"cnt\":" + String(requiredBrakeCount) +
                ",\"tmo\":" + String(timeoutSeconds) + "}";
  server.send(200, "application/json", json);
}

void handleStopAP() {
  server.send(200, "text/plain", "Stopping AP");
  pendingStopAP = true;
  stopAPRequestTime = millis();
}

// Web UI Mode Button (D6 के जैसा + 2 सेकंड रियल-टाइम LED प्रीव्यू)
void handleMode() {
  currentMode = (currentMode % totalModes) + 1;
  prefs.begin("taillight", false);
  prefs.putInt("mode", currentMode);
  prefs.end();
  if (systemPower) lightsActive = true;
  lastMotionTime = millis();
  isModePreviewActive = true;
  modePreviewStartTime = millis();
  Serial.print("Web UI Pressed. Switched to Mode: ");
  Serial.println(currentMode);

  String json = "{\"mode\":" + String(currentMode) + "}";
  server.send(200, "application/json", json);
}

// Web UI Brightness Button (D7 के जैसा)
void handleBrightness() {
  brightnessLevel = (brightnessLevel % 3) + 1;
  prefs.begin("taillight", false);
  prefs.putInt("bright", brightnessLevel);
  prefs.end();
  String brightName = (brightnessLevel == 1) ? "Low" : ((brightnessLevel == 2) ? "Medium" : "High");
  Serial.print("Web UI Pressed. Brightness Level: ");
  Serial.println(brightName);

  if (systemPower) lightsActive = true;
  lastMotionTime = millis();
  isModePreviewActive = false;

  String json = "{\"level\":" + String(brightnessLevel) + ",\"name\":\"" + brightName + "\"}";
  server.send(200, "application/json", json);

  // HTTP रिस्पॉन्स जाने के बाद 1 सेकंड वाला LED ब्राइटनेस गेज दिखाएं
  pendingBrightnessGauge = true;
  gaugeRequestTime = millis();
}

// Web UI Battery Button (D3 Short Press के जैसा)
void handleBattery() {
  readBattery();
  Serial.println("Web UI Pressed. Showing Battery Gauge...");
  if (systemPower) lightsActive = true;
  lastMotionTime = millis();
  isModePreviewActive = false;

  String json = "{\"pct\":" + String(batteryPercent) + ",\"volt\":\"" + String(batteryVoltage, 2) + "\"}";
  server.send(200, "application/json", json);

  // HTTP रिस्पॉन्स जाने के बाद LED बैटरी गेज दिखाएं
  pendingBatteryGauge = true;
  gaugeRequestTime = millis();
}

void handleCaptivePortal() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.send(302, "text/plain", "");
}

void startSoftAP() {
  if (isSoftAPActive) return;

  Serial.println("Starting Soft AP: SmartBrake (192.168.4.1)");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(AP_SSID);

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", apIP);

  if (!webRoutesInitialized) {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_GET, handleSave);
    server.on("/default", HTTP_GET, handleDefault);
    server.on("/stop", HTTP_GET, handleStopAP);
    server.on("/mode", HTTP_GET, handleMode);
    server.on("/brightness", HTTP_GET, handleBrightness);
    server.on("/battery", HTTP_GET, handleBattery);
    server.on("/favicon.ico", []() { server.send(204); });

    // Captive Portal Auto-Open Endpoints (Android, iOS, macOS, Windows)
    server.on("/generate_204", handleCaptivePortal);
    server.on("/gen_204", handleCaptivePortal);
    server.on("/hotspot-detect.html", handleCaptivePortal);
    server.on("/library/test/success.html", handleCaptivePortal);
    server.on("/connecttest.txt", handleCaptivePortal);
    server.on("/ncsi.txt", handleCaptivePortal);
    server.on("/redirect", handleCaptivePortal);
    server.on("/fwlink", handleCaptivePortal);
    server.on("/canonical.html", handleCaptivePortal);
    server.on("/success.txt", handleCaptivePortal);
    server.onNotFound(handleCaptivePortal);
    webRoutesInitialized = true;
  }

  server.begin();
  isSoftAPActive = true;
  pendingStopAP = false;
  if (systemPower) lightsActive = true;
  lastMotionTime = millis();
}

void stopSoftAP() {
  if (!isSoftAPActive) return;

  pendingStopAP = false;
  pendingBrightnessGauge = false;
  pendingBatteryGauge = false;
  isModePreviewActive = false;
  isSoftAPActive = false;
  dnsServer.stop();
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  if (systemPower) lightsActive = true;
  lastMotionTime = millis();
  Serial.println("Soft AP Stopped.");
}

void handleSoftAP() {
  if (!isSoftAPActive) return;

  dnsServer.processNextRequest();
  server.handleClient();

  if (pendingBrightnessGauge && (millis() - gaugeRequestTime >= 50)) {
    pendingBrightnessGauge = false;
    showBrightnessGauge();
  }

  if (pendingBatteryGauge && (millis() - gaugeRequestTime >= 50)) {
    pendingBatteryGauge = false;
    showStartupBattery();
  }

  if (pendingStopAP && (millis() - stopAPRequestTime >= 500)) {
    stopSoftAP();
  }
}

// MPU6050 इनिशियलाइज़ेशन और रिकवरी फ़ंक्शन
void initMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); 
  Wire.write(0x00); // MPU6050 को स्लीप से जगाएं
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A); 
  Wire.write(0x04); // Digital Low Pass Filter (20Hz)
  Wire.endTransmission(true);
}

// D1 बटन से Deep Sleep में जाने का फ़ंक्शन
void enterDeepSleep() {
  Serial.println("Entering Deep Sleep Mode...");

  // यदि Soft AP चालू है, तो पहले उसे बंद करें
  if (isSoftAPActive) {
    stopSoftAP();
  }

  // 1. सभी LEDs बंद करें
  colorAll(0, 0, 0);
  strip.show();

  // 2. बैटरी बचाने के लिए MPU6050 को स्लीप मोड में डालें (0x6B रजिस्टर में बिट 6 = 0x40)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0x40);
  Wire.endTransmission(true);

  // 3. यूजर द्वारा D1 बटन छोड़ने का इंतज़ार करें ताकि तुरंत वेक-अप न हो जाए
  while (digitalRead(BUTTON3_PIN) == LOW) {
    delay(10);
  }
  delay(100); // डिबाउंस सेफ्टी डिले

  // 4. D1 पिन (LP GPIO1 - LOW पर) को Deep Sleep Wakeup सोर्स सेट करें
  gpio_num_t wakeupPin = (gpio_num_t)digitalPinToGPIONumber(BUTTON3_PIN);
  uint64_t pinMask = (1ULL << wakeupPin);

  rtc_gpio_init(wakeupPin);
  rtc_gpio_set_direction(wakeupPin, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(wakeupPin);
  rtc_gpio_pulldown_dis(wakeupPin);
  esp_sleep_enable_ext1_wakeup(pinMask, ESP_EXT1_WAKEUP_ANY_LOW);

  Serial.flush();
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);

  // Flash (NVS) से पिछले सेव किए गए Mode, Brightness और Brake Settings को लोड करें
  prefs.begin("taillight", true);
  currentMode = constrain(prefs.getInt("mode", 1), 1, totalModes);
  brightnessLevel = constrain(prefs.getInt("bright", 3), 1, 3);
  brakeThreshold = constrain(prefs.getFloat("brkThr", DEFAULT_BRAKE_THRESHOLD), 0.10f, 0.22f);
  brakeReleaseThreshold = constrain(prefs.getFloat("brkRel", DEFAULT_BRAKE_RELEASE_THRESHOLD), 0.04f, 0.08f);
  requiredBrakeCount = constrain(prefs.getInt("brkCnt", DEFAULT_REQUIRED_BRAKE_COUNT), 4, 10);
  timeoutSeconds = constrain(prefs.getInt("tmOut", DEFAULT_TIMEOUT_SEC), 0, 30);
  timeoutDuration = (unsigned long)timeoutSeconds * 1000UL;
  prefs.end();

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Mode बटन (D6)
  pinMode(BUTTON2_PIN, INPUT_PULLUP); // Battery Gauge / Soft AP बटन (D3)
  pinMode(BUTTON3_PIN, INPUT_PULLUP); // ON/OFF Deep Sleep बटन (D1)
  pinMode(BUTTON4_PIN, INPUT_PULLUP); // Brightness बटन (D7)
  pinMode(BATTERY_PIN, INPUT);

  // यदि Deep Sleep से जागा है, तो यूजर के D1 बटन छोड़ने का इंतज़ार करें
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
    rtc_gpio_deinit((gpio_num_t)digitalPinToGPIONumber(BUTTON3_PIN));
    pinMode(BUTTON3_PIN, INPUT_PULLUP);
    while (digitalRead(BUTTON3_PIN) == LOW) {
      delay(10);
    }
    delay(50);
  }

  strip.begin();
  strip.show(); 

  Wire.begin(D4, D5); 
  Wire.setClock(400000); // 400kHz I2C Speed
  Wire.setTimeOut(50);   // 50ms I2C Timeout (बस हैंग होने से बचाएगा)

  initMPU();

  showStartupBattery();
  calibrateSensorInitial();

  lastMotionTime = millis();
  lastSensorTime = millis();
}

void loop() {
  handleButton();       // मोड बदलने वाला बटन (D6)
  handleButton2();      // बैटरी गेज (Short Press) / Soft AP (Long Press) बटन (D3)
  handleButton3();      // ON/OFF (Deep Sleep) करने वाला बटन (D1)
  handleButton4();      // ब्राइटनेस बदलने वाला बटन (D7)

  handleSoftAP();       // Soft AP और Captive Portal हैंडलिंग
  readSensorAndMotion();
  checkBatteryPeriodic();
  updateLights();
}

void calibrateSensorInitial() {
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  int validSamples = 0;

  for (int i = 0; i < 40; i++) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);
    if (Wire.available() >= 14) {
      int16_t rawX  = Wire.read() << 8 | Wire.read();
      int16_t rawY  = Wire.read() << 8 | Wire.read();
      int16_t rAz   = Wire.read() << 8 | Wire.read();
      Wire.read(); Wire.read(); // Temperature skip
      int16_t rawGx = Wire.read() << 8 | Wire.read();
      int16_t rawGy = Wire.read() << 8 | Wire.read();
      int16_t rGz   = Wire.read() << 8 | Wire.read();

      // 90-degree rotation (Right-Hand Rule के अनुसार साइन सुधार: X = Y, Y = -X)
      int16_t rAx = rawY;
      int16_t rAy = -rawX;
      int16_t rGx = rawGy;
      int16_t rGy = -rawGx;

      sumAx += rAx / 16384.0;
      sumAy += rAy / 16384.0;
      sumAz += rAz / 16384.0;

      sumGx += rGx / 131.0;
      sumGy += rGy / 131.0;
      sumGz += rGz / 131.0;
      validSamples++;
    }
    delay(5);
  }

  if (validSamples > 0) {
    float avgAx = sumAx / validSamples;
    float avgAy = sumAy / validSamples;
    float avgAz = sumAz / validSamples;

    // हल्की हलचल में भी गलत ऑफ़सेट न आए इसलिए ±10 deg/s तक सीमित रखें
    gxOffset = constrain(sumGx / validSamples, -10.0f, 10.0f);
    gyOffset = constrain(sumGy / validSamples, -10.0f, 10.0f);
    gzOffset = constrain(sumGz / validSamples, -10.0f, 10.0f);

    estimatedPitch = atan2(-avgAx, sqrt(avgAy * avgAy + avgAz * avgAz)) * 180.0 / M_PI;
    estimatedRoll  = atan2(avgAy, avgAz) * 180.0 / M_PI;
    float initTrueAx = avgAx + sin(estimatedPitch * M_PI / 180.0);
    filteredAx = initTrueAx;
    baselineAx = initTrueAx;
    gravityZ = avgAz;
  }
}

void readSensorAndMotion() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastSensorTime) / 1000.0;
  if (dt < 0.01) return; 
  lastSensorTime = currentTime;

  // Battery/Brightness gauge के delay() या I2C recovery के बाद बड़ा dt spike रोकें
  if (dt > 0.1) dt = 0.01;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); 
  byte txError = Wire.endTransmission(false);
  uint8_t bytesReceived = Wire.requestFrom(MPU_addr, 14, true);

  if (txError == 0 && bytesReceived >= 14) {
    int16_t raw_ax = Wire.read() << 8 | Wire.read();
    int16_t raw_ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); 
    int16_t raw_gx = Wire.read() << 8 | Wire.read();
    int16_t raw_gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();

    // यदि सेंसर वाइब्रेशन से रीसेट होकर लगातार 0 दे रहा है, तो रिकवर करें
    if (raw_ax == 0 && raw_ay == 0 && az == 0) {
      i2cErrorCount++;
      if (i2cErrorCount > 10) {
        Serial.println("MPU6050 zero-reading detected! Re-initializing...");
        initMPU();
        i2cErrorCount = 0;
      }
      return;
    }
    i2cErrorCount = 0;

    // 90-degree rotation (Right-Hand Rule साइन सुधार: Pitch और Gyro_Y एक दिशा में काम करेंगे)
    ax = raw_ay;
    ay = -raw_ax;
    gx = raw_gy;
    gy = -raw_gx;
  } else {
    // I2C बस एरर या टाइमआउट होने पर ऑटो-रिकवरी
    i2cErrorCount++;
    if (i2cErrorCount > 10) {
      Serial.println("I2C Timeout/Disconnect! Resetting I2C Bus & MPU6050...");
      Wire.end();
      delay(10);
      Wire.begin(D4, D5);
      Wire.setClock(400000);
      Wire.setTimeOut(50);
      initMPU();
      i2cErrorCount = 0;
    }
    return;
  }

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  // कैलिब्रेटेड Gyro रीडिंग (deg/s)
  float gyroX_deg = (gx / 131.0) - gxOffset;
  float gyroY_deg = (gy / 131.0) - gyOffset;
  float gyroZ_deg = (gz / 131.0) - gzOffset;

  // 1. Roll (झुकाव) ट्रैक करें
  float accelRoll = atan2(ay_g, az_g) * 180.0 / M_PI;
  estimatedRoll = 0.992 * (estimatedRoll + gyroX_deg * dt) + 0.008 * accelRoll;
  estimatedRoll = constrain(estimatedRoll, -60.0f, 60.0f);

  // 2. Roll-Compensated Pitch Rate: Left/Right मोड़ से आने वाला झूठा pitch हट जाएगा
  float rollRad = estimatedRoll * M_PI / 180.0;
  float truePitchRate = gyroY_deg * cos(rollRad) - gyroZ_deg * sin(rollRad);

  float accelPitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / M_PI;

  // Adaptive Complementary Filter:
  // सामान्य राइड में pitch जल्दी सेटल हो (0.985), ब्रेक लगते समय धीमा रहे (0.996)
  float pitchAlpha = (!isBraking && brakeCounter == 0) ? 0.985f : 0.996f;
  estimatedPitch = pitchAlpha * (estimatedPitch + truePitchRate * dt) + (1.0f - pitchAlpha) * accelPitch;
  estimatedPitch = constrain(estimatedPitch, -25.0f, 25.0f);

  float true_ax = ax_g + sin(estimatedPitch * M_PI / 180.0);

  // MPU6050 में 20Hz DLPF पहले से ऑन है, इसलिए रिस्पॉन्स तेज़ रखें
  filteredAx = 0.45 * true_ax + 0.55 * filteredAx;

  gravityZ = 0.95 * gravityZ + 0.05 * az_g;
  float verticalShock = abs(az_g - gravityZ); 

  float netBrakeForce = (filteredAx - baselineAx) * BRAKE_DIRECTION;

  bool isBumpOrPothole = (verticalShock > maxVerticalShock); 
  bool isTiltingFast   = (abs(truePitchRate) > maxPitchRate);
  bool isTurning       = (abs(gyroZ_deg) > maxYawRate || abs(gyroX_deg) > maxRollRate);

  // Baseline केवल सीधी और सामान्य स्थिति में अपडेट करें (तेज़ acceleration या turn में नहीं)
  if (!isBraking && brakeCounter == 0 && !isTurning && verticalShock < 0.25 && abs(netBrakeForce) < 0.06) {
    baselineAx = 0.998 * baselineAx + 0.002 * filteredAx;
  }

  // maxBrakeHoldTime के बाद सिग्नल सामान्य होने पर ही दोबारा ब्रेक ट्रिगर होने दें
  if (waitForBrakeRelease && netBrakeForce < brakeReleaseThreshold) {
    waitForBrakeRelease = false;
  }

  if (!waitForBrakeRelease && netBrakeForce > brakeThreshold && !isBumpOrPothole && !isTiltingFast && !isTurning) {
    brakeCounter++;
    if (brakeCounter >= requiredBrakeCount) {
      if (!isBraking) {
        Serial.print("Confirmed Brake! Force: ");
        Serial.println(netBrakeForce, 2);
        brakeStartTime = currentTime; // सिर्फ ब्रेक शुरू होने पर ही सेट करें
      }
      isBraking = true;

      // सिर्फ तभी लाइट एक्टिवेट करें जब सिस्टम ON हो
      if (systemPower) lightsActive = true; 

      lastMotionTime = currentTime;
      brakeCounter = requiredBrakeCount; 
    }
  } else {
    if ((isBumpOrPothole || isTurning) && brakeCounter >= 2) {
      brakeCounter -= 2;
    } else if (brakeCounter > 0) {
      brakeCounter--;
    }
  }

  if (isBraking) {
    unsigned long brakeElapsed = currentTime - brakeStartTime;
    if (brakeElapsed > maxBrakeHoldTime) {
      // सेफ्टी टाइमआउट: 2.5 सेकंड से ज़्यादा रेड लाइट कभी अटक नहीं पाएगी
      isBraking = false;
      brakeCounter = 0;
      waitForBrakeRelease = true;
    } else if (brakeElapsed > minBrakeHoldTime && netBrakeForce < brakeReleaseThreshold) {
      isBraking = false;
      brakeCounter = 0;
    }
  }

  float accelDeltaX = abs(filteredAx - prevAccelX);
  float accelDeltaY = abs(ay_g - prevAccelY);

  if (accelDeltaX > motionThreshold || accelDeltaY > motionThreshold) {
    // सिर्फ तभी लाइट वेक-अप हो जब सिस्टम मैन्युअली ON हो
    if (!lightsActive && systemPower) {
      Serial.println("Motion detected! Waking up lights.");
      lightsActive = true;
    }
    lastMotionTime = currentTime;
  }

  prevAccelX = filteredAx;
  prevAccelY = ay_g;

  if (timeoutSeconds > 0 && lightsActive && !isSoftAPActive && (currentTime - lastMotionTime > timeoutDuration)) {
    lightsActive = false;
    Serial.printf("%ds inactivity reached. Turning lights OFF.\n", timeoutSeconds);
  }
}

void readBattery() {
  long sumMv = 0;
  for (int i = 0; i < 16; i++) {
    sumMv += analogReadMilliVolts(BATTERY_PIN);
  }
  float pinVoltage = (sumMv / 16.0) / 1000.0;
  batteryVoltage = pinVoltage * 2.095; 

  int pct = (int)((batteryVoltage - 3.30) * 100.0 / (4.20 - 3.30));
  batteryPercent = constrain(pct, 0, 100);
  isLowBattery = (batteryPercent <= 20);

  // सीरियल मॉनिटर में बैटरी प्रतिशत और वोल्टेज प्रिंट करें
  Serial.printf("Battery: %d%% (%.2fV)%s\n",
                batteryPercent,
                batteryVoltage,
                isLowBattery ? " [LOW]" : "");
}

void showStartupBattery() {
  readBattery();

  strip.setBrightness(80);
  strip.clear();

  int ledsToLight = (batteryPercent * NUM_LEDS + 99) / 100;
  if (ledsToLight == 0 && batteryPercent > 0) ledsToLight = 1;

  uint32_t barColor;
  if (batteryPercent > 50) {
    barColor = strip.Color(0, 255, 0);   
  } else if (batteryPercent > 20) {
    barColor = strip.Color(255, 180, 0); 
  } else {
    barColor = strip.Color(255, 0, 0);   
  }

  if (isLowBattery) {
    for (int blink = 0; blink < 5; blink++) {
      strip.clear();
      for (int i = 0; i < ledsToLight; i++) {
        strip.setPixelColor(i, barColor);
      }
      strip.show();
      delay(200);
      strip.clear();
      strip.show();
      delay(200);
    }
  } else {
    for (int i = 0; i < ledsToLight; i++) {
      strip.setPixelColor(i, barColor);
    }
    strip.show();
    delay(2000);
  }

  strip.clear();
  strip.show();
}

// ब्राइटनेस लेवल को 1 सेकंड के लिए मीटर गेज के रूप में दिखाने का फ़ंक्शन
void showBrightnessGauge() {
  int selectedBrightness = (brightnessLevel == 1) ? 75 : ((brightnessLevel == 2) ? 165 : 242);
  strip.setBrightness(selectedBrightness);
  strip.clear();

  // मीटर गेज: Low (1) = 2 LEDs, Medium (2) = 5 LEDs, High (3) = 8 LEDs
  int ledsToLight = (brightnessLevel == 1) ? 2 : ((brightnessLevel == 2) ? 5 : NUM_LEDS);

  for (int i = 0; i < ledsToLight; i++) {
    strip.setPixelColor(i, strip.Color(0, 255, 0));
  }
  strip.show();
  delay(1000); // 1 सेकंड तक गेज दिखाएं

  strip.clear();
  strip.show();
}

void checkBatteryPeriodic() {
  if (millis() - lastBatteryCheck >= batteryCheckInterval) {
    lastBatteryCheck = millis();
    readBattery();
  }
}

// मोड बदलने वाला बटन (D6)
void handleButton() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        currentMode = (currentMode % totalModes) + 1;
        prefs.begin("taillight", false);
        prefs.putInt("mode", currentMode);
        prefs.end();
        if (systemPower) lightsActive = true;
        lastMotionTime = millis();
        Serial.print("Button Pressed. Switched to Mode: ");
        Serial.println(currentMode);
      }
    }
  }
  lastButtonState = reading;
}

// On-Demand Battery Gauge (Short Press) & SmartBrake Soft AP (Long Press) बटन (D3)
void handleButton2() {
  int reading = digitalRead(BUTTON2_PIN);

  if (reading != lastButton2State) {
    lastDebounceTime2 = millis();
  }

  if ((millis() - lastDebounceTime2) > debounceDelay) {
    if (reading != button2State) {
      button2State = reading;

      if (button2State == LOW) {
        button2PressStartTime = millis();
        button2LongPressTriggered = false;
      } else {
        // बटन छोड़ने पर: यदि लॉन्ग प्रेस नहीं था, तो सामान्य Battery Gauge दिखाएं
        if (!button2LongPressTriggered && !isSoftAPActive) {
          Serial.println("Button 2 Pressed. Showing Battery Gauge...");
          showStartupBattery();
          if (systemPower) lightsActive = true;
          lastMotionTime = millis();
        }
        button2LongPressTriggered = false;
      }
    }
  }

  // Long Press (2 सेकंड) डिटेक्शन -> SmartBrake Soft AP को ON/OFF टॉगल करें
  if (button2State == LOW && !button2LongPressTriggered &&
      (millis() - button2PressStartTime >= longPressDuration)) {
    button2LongPressTriggered = true;
    if (!isSoftAPActive) {
      Serial.println("Button 2 Long Pressed. Starting SmartBrake Soft AP...");
      startSoftAP();
    } else {
      Serial.println("Button 2 Long Pressed. Stopping SmartBrake Soft AP...");
      stopSoftAP();
    }
  }

  lastButton2State = reading;
}

// ON/OFF बटन (D1) - Deep Sleep ट्रिगर करेगा
void handleButton3() {
  int reading = digitalRead(BUTTON3_PIN);

  if (reading != lastButton3State) {
    lastDebounceTime3 = millis();
  }

  if ((millis() - lastDebounceTime3) > debounceDelay) {
    if (reading != button3State) {
      button3State = reading;

      if (button3State == LOW) { 
        systemPower = false; 
        lightsActive = false;
        Serial.println("System turned OFF manually -> Going to Deep Sleep");
        enterDeepSleep(); // MPU6050 और ESP32C6 को Deep Sleep में भेजें
      }
    }
  }
  lastButton3State = reading;
}

// ब्राइटनेस लेवल बदलने वाला बटन (D7): 1 = Low, 2 = Medium, 3 = High
void handleButton4() {
  int reading = digitalRead(BUTTON4_PIN);

  if (reading != lastButton4State) {
    lastDebounceTime4 = millis();
  }

  if ((millis() - lastDebounceTime4) > debounceDelay) {
    if (reading != button4State) {
      button4State = reading;

      if (button4State == LOW) {
        brightnessLevel = (brightnessLevel % 3) + 1;
        prefs.begin("taillight", false);
        prefs.putInt("bright", brightnessLevel);
        prefs.end();
        Serial.print("Button 4 Pressed. Brightness Level: ");
        Serial.println(brightnessLevel == 1 ? "Low" : (brightnessLevel == 2 ? "Medium" : "High"));
        showBrightnessGauge(); // 1 सेकंड के लिए ब्राइटनेस मीटर गेज दिखाएं
        if (systemPower) lightsActive = true;
        lastMotionTime = millis();
      }
    }
  }
  lastButton4State = reading;
}

void updateLights() {
  static unsigned long lastLightUpdate = 0;
  if (millis() - lastLightUpdate < 15) return; 
  lastLightUpdate = millis();

  // अगर सिस्टम OFF है, तो लाइट बंद रखें
  if (!systemPower) {
    colorAll(0, 0, 0);
    strip.show();
    return;
  }

  // Soft AP एक्टिव होने पर: अगर Mode प्रीव्यू चालू है तो 2 सेकंड तक असली मोड दिखेगा, वरना नीली ब्लिंक
  if (isSoftAPActive) {
    if (isModePreviewActive && (millis() - modePreviewStartTime >= modePreviewDuration)) {
      isModePreviewActive = false;
    }
    if (!isModePreviewActive) {
      strip.setBrightness(120);
      if ((millis() / 350) % 2 == 0) {
        colorAll(0, 0, 255);
      } else {
        colorAll(0, 0, 0);
      }
      strip.show();
      return;
    }
  }

  // अगर स्लीप/इनएक्टिविटी मोड में है, तो लाइट बंद रखें
  if (!lightsActive) {
    colorAll(0, 0, 0);
    strip.show();
    return;
  }

  if (isBraking && !isModePreviewActive) {
    strip.setBrightness(255); 
    colorAll(255, 0, 0);
    strip.show();
    return;
  }

  unsigned long currentMillis = millis();
  int selectedBrightness = (brightnessLevel == 1) ? 75 : ((brightnessLevel == 2) ? 165 : 242);
  strip.setBrightness(isLowBattery ? 40 : selectedBrightness);

  switch (currentMode) {
    case 1: // Slow Flash
      if ((currentMillis / 800) % 2 == 0) {
        colorAll(255, 0, 0);
      } else {
        colorAll(0, 0, 0);
      }
      break;

    case 2: // Smooth Breathing Flash
      {
        int maxB = isLowBattery ? 100 : ((brightnessLevel == 1) ? 75 : ((brightnessLevel == 2) ? 165 : 255));
        
        // 0.0 से 1.0 के बीच एक परफ़ेक्ट वेव बनाएं (बिना क्लिप हुए)
        float breathCurve = (exp(sin(currentMillis / 400.0 * M_PI)) - 0.36787944) / 2.35040238;
        
        // उसे अपनी चुनी हुई ब्राइटनेस के हिसाब से स्केल करें (मिनिमम 15)
        int brightness = 15 + (breathCurve * (maxB - 15));
        
        strip.setBrightness(brightness); 
        colorAll(255, 0, 0);
      }
      break;

    case 3: // Triple Blink Strobe
      {
        int step = (currentMillis / 60) % 20; 
        if (step == 0 || step == 2 || step == 4) {
          colorAll(255, 0, 0); 
        } else {
          colorAll(0, 0, 0);   
        }
      }
      break;

    case 4: // Emergency Red-Yellow
      if ((currentMillis / 150) % 2 == 0) {
        for (int i = 0; i < 4; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));      
        for (int i = 4; i < NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(0, 0, 0)); 
      } else {
        for (int i = 0; i < 4; i++) strip.setPixelColor(i, strip.Color(0, 0, 0));        
        for (int i = 4; i < NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(255, 200, 0)); // नीले की जगह पीला (Yellow)
      }
      break;

    case 5: // One-way Scanner (a to z) with Tail Effect
      {
        int pos = (currentMillis / 65) % NUM_LEDS;
        strip.clear();

        // मुख्य LED (पूरी चमक)
        strip.setPixelColor(pos, strip.Color(255, 0, 0));

        // टेल इफेक्ट (पीछे छूटती हुई लाइट्स की चमक धीरे-धीरे कम होगी)
        if (pos >= 1) strip.setPixelColor(pos - 1, strip.Color(120, 0, 0)); 
        if (pos >= 2) strip.setPixelColor(pos - 2, strip.Color(40, 0, 0));  
        if (pos >= 3) strip.setPixelColor(pos - 3, strip.Color(10, 0, 0));  
      }
      break;

    case 6: // One-way Amber Comet (Red Base with Yellow Sweeper A to Z)
      {
        int speed = 70; // स्पीड (मिलीसेकंड में)
        int pos = (currentMillis / speed) % NUM_LEDS; // केवल A से Z (0 से 7) की तरफ जाएगा

        // 1. बैकग्राउंड को हमेशा हल्के लाल (Dim Red) रंग में जलाए रखें
        colorAll(40, 0, 0); 
        
        // 2. मुख्य LED (हेड) को तेज़ पीला (Amber/Yellow) करें
        strip.setPixelColor(pos, strip.Color(255, 200, 0)); 
        
        // 3. पीले रंग के ठीक पीछे (टेल) एक स्मूथ ऑरेंज (Orange) इफेक्ट दें
        if (pos >= 1) strip.setPixelColor(pos - 1, strip.Color(180, 80, 0)); // थोड़ी हल्की ऑरेंज
        if (pos >= 2) strip.setPixelColor(pos - 2, strip.Color(80, 20, 0));  // सबसे हल्की ऑरेंज/रेड
      }
      break;

    case 7: // Alternating 2 by 2 (Red)
      {
        bool state = ((currentMillis / 200) % 2 == 0);
        strip.clear();
        for (int i = 0; i < NUM_LEDS; i++) {
          // हर 2 LED के बाद स्टेट बदलेगी
          if (((i / 2) % 2 == 0) == state) {
            strip.setPixelColor(i, strip.Color(255, 0, 0)); // लाल
          }
        }
      }
      break;

    case 8: // Red Sparkle
      {
        colorAll(30, 0, 0); 
        if (random(10) > 7) { 
          int randomLED = random(NUM_LEDS);
          strip.setPixelColor(randomLED, strip.Color(255, 255, 255)); 
        }
      }
      break;

    case 9: // Heartbeat Double Pulse
      {
        int cycle = currentMillis % 1200; // 1.2 सेकंड का पूरा लूप
        
        // 0-80ms ON, 80-180ms OFF, 180-260ms ON, बाकी समय हल्की रोशनी
        if (cycle < 80 || (cycle > 180 && cycle < 260)) {
          colorAll(255, 0, 0); // तेज़ लाल (Bright Red)
        } else {
          colorAll(15, 0, 0);  // बहुत हल्का लाल बैकग्राउंड (Dim Red)
        }
      }
      break;

    case 10: // Heavy Rain / Fog Strobe (High Visibility)
      {
        int cycle = currentMillis % 1000; // 1 सेकंड का पूरा लूप
        
        // 3 बहुत तेज़ और छोटे ब्लिंक (0-50ms, 100-150ms, 200-250ms)
        if (cycle < 50 || (cycle > 100 && cycle < 150) || (cycle > 200 && cycle < 250)) {
          // लाल रंग के साथ थोड़ा सा पीला/सफेद मिलाया गया है ताकि चमक (Punch) और बढ़ जाए
          colorAll(255, 40, 0); 
        } else {
          colorAll(0, 0, 0);   // बाकी 750ms के लिए एकदम डार्क (कंट्रास्ट के लिए)
        }
      }
      break;
  }

  if (isLowBattery) {
    if ((currentMillis / 250) % 2 == 0) {
      strip.setPixelColor(NUM_LEDS - 1, strip.Color(255, 180, 0));
    } else {
      strip.setPixelColor(NUM_LEDS - 1, strip.Color(0, 0, 0));
    }
  }

  strip.show();
}

void colorAll(uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
}