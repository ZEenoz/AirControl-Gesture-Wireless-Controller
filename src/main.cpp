#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <BleKeyboard.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- WIFI CREDENTIALS ---
const char* ssid = "ZEenoz";
const char* pass = "6kwhR471jdj";

// --- OBJECT DECLARATIONS ---
BleKeyboard bleKeyboard("ESP32 Presenter", "FurrTech", 100);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_MPU6050 mpu;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
const int LDRSensor = 34;

// --- KALMAN FILTER VARIABLES ---
float x_angle = 0, y_angle = 0, x_bias = 0, y_bias = 0;
float P[2][2] = {{1, 0}, {0, 1}};
float q_angle = 0.01, q_bias = 0.01, r_measure = 0.01;

// --- TIMER & GESTURE CONTROL ---
unsigned long lastTime = 0;
const long timerDelay = 100; 
bool gesture_sent = false;
const float GESTURE_THRESHOLD = 5.0;

// --- STATE MACHINE VARIABLES (Global States) ---
bool slideModeEnabled = true;     
int lastPhysicalLdrStatus = 0;  
String actionStatus = "STATUS: IDLE"; 
bool webClientReady = false; 


sensors_event_t mpu_accel, mpu_gyro, mpu_temp;
int ldrValue = 0;
int currentPhysicalLdrStatus = 0;
float filteredX = 0, filteredY = 0, z_accel = 0;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
<title>ESP32 Sensor Dashboard</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: Arial, sans-serif; text-align: center; background-color: #282c34; color: white; margin: 0; padding: 20px; }
h1 { color: #61dafb; }
#sensorCanvas { background-color: #20232a; border: 1px solid #61dafb; margin: 20px auto; }
.sensor-grid { display: grid; grid-template-columns: 1fr 1fr; max-width: 400px; margin: auto; gap: 10px; }
.sensor-data { font-size: 1.2em; background-color: #3c4049; padding: 10px; border-radius: 5px; }
#action-status { font-size: 1.5em; color: #61dafb; margin-top: 20px; min-height: 1.5em; }
</style>
</head>
<body>
<h1>ESP32 Live Dashboard</h1>
<div class="sensor-grid">
<div class="sensor-data">X: <span id="x_val">0.00</span></div>
<div class="sensor-data">Y: <span id="y_val">0.00</span></div>
<div class="sensor-data">Z: <span id="z_val">0.00</span></div>
<div class="sensor-data">LDR: <span id="ldr_val">...</span></div>
</div>
<canvas id="sensorCanvas" width="300" height="300"></canvas>
<div id="action-status">STATUS: IDLE</div>
<script>
var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
window.addEventListener('load', function() { initWebSocket(); drawGrid(0, 0); });
function initWebSocket() { websocket = new WebSocket(gateway); websocket.onopen = onOpen; websocket.onclose = onClose; websocket.onmessage = onMessage; }
function onOpen(event) { console.log('Connection opened'); websocket.send("CLIENT_READY"); }
function onClose(event) { console.log('Connection closed'); setTimeout(initWebSocket, 2000); }
const canvas = document.getElementById('sensorCanvas'); const ctx = canvas.getContext('2d'); const centerX = canvas.width / 2; const centerY = canvas.height / 2;
function drawGrid(x, y) {
ctx.clearRect(0, 0, canvas.width, canvas.height); ctx.strokeStyle = '#4a4f58'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(0, centerY); ctx.lineTo(canvas.width, centerY); ctx.moveTo(centerX, 0); ctx.lineTo(centerX, canvas.height); ctx.stroke();
let plotX = centerX + (x / 45.0) * centerX; let plotY = centerY - (y / 45.0) * centerY;
ctx.beginPath(); ctx.arc(plotX, plotY, 12, 0, 2 * Math.PI, false); ctx.fillStyle = '#61dafb'; ctx.fill(); ctx.strokeStyle = '#cff4ff'; ctx.lineWidth = 2; ctx.stroke();
}
function onMessage(event) {
var data = JSON.parse(event.data);
document.getElementById('x_val').innerText = data.x.toFixed(2); 
document.getElementById('y_val').innerText = data.y.toFixed(2); 
document.getElementById('z_val').innerText = data.z.toFixed(2);
document.getElementById('ldr_val').innerText = (data.ldr === 1) ? "DARKNESS" : "BRIGHT";
if (data.action) { document.getElementById('action-status').innerText = data.action; }
drawGrid(data.x, data.y);
}
</script>
</body>
</html>
)rawliteral";


// Kalman filter function (Unchanged)
float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias) {
    float rate = newRate - bias; angle += dt * rate; P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + q_angle); P[0][1] -= dt * P[1][1]; P[1][0] -= dt * P[1][1]; P[1][1] += q_bias * dt;
    float S = P[0][0] + r_measure; float K[2] = {P[0][0] / S, P[1][0] / S}; float y = newAngle - angle; angle += K[0] * y; bias += K[1] * y;
    P[0][0] -= K[0] * P[0][0]; P[0][1] -= K[0] * P[0][1]; P[1][0] -= K[1] * P[0][0]; P[1][1] -= K[1] * P[0][1]; return angle;
}

// Full WebSocket Event Handler (Unchanged)
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      webClientReady = false; 
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      webClientReady = false; 
      break;
    case WS_EVT_DATA: { 
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        if (strcmp((char*)data, "CLIENT_READY") == 0) {
          Serial.println("Web client is ready. Starting data stream.");
          webClientReady = true; 
        }
      }
      break;
    } 
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// Setup function (Unchanged)
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  bleKeyboard.begin();
  
  if (!mpu.begin()) { Serial.println("MPU6050 not found!"); while (1) delay(10); }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { Serial.println(F("SSD1306 allocation failed")); }
  else {
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0); display.println("System Booting..."); display.display();
  }

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) { delay(1000); Serial.println("Connecting to WiFi.."); }
  Serial.println(WiFi.localIP());

  ws.onEvent(onWebSocketEvent); 
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(200, "text/html", index_html); });
  server.begin();
}


// --- STATE 1: READ INPUTS ---
void readAllSensors() {
  mpu.getEvent(&mpu_accel, &mpu_gyro, &mpu_temp);
  ldrValue = analogRead(LDRSensor);
  currentPhysicalLdrStatus = (ldrValue < 2000) ? 0 : 1; // 0=Bright, 1=Dark
}

// --- STATE 2.1: PROCESS LDR LOGIC ---
void updateLDRToggleState() {
  if (currentPhysicalLdrStatus == 1 && lastPhysicalLdrStatus == 0) {
    slideModeEnabled = !slideModeEnabled; 
    Serial.print("Mode Toggled: "); Serial.println(slideModeEnabled ? "BRIGHT" : "DARK");
  }
  lastPhysicalLdrStatus = currentPhysicalLdrStatus; 
}

// --- STATE 2.2: PROCESS SENSOR DATA ---
void runKalmanFilter() {
  float dt = timerDelay / 1000.0;
  filteredX = kalmanFilter(mpu_accel.acceleration.x, mpu_gyro.gyro.x, dt, x_angle, x_bias);
  filteredY = kalmanFilter(mpu_accel.acceleration.y, mpu_gyro.gyro.y, dt, y_angle, y_bias);
  z_accel = mpu_accel.acceleration.z; 
}

// --- STATE 2.3: PROCESS GESTURE LOGIC ---
void updateGestureState() {
  if (bleKeyboard.isConnected()) {
    if (slideModeEnabled) {
      if (filteredY >= GESTURE_THRESHOLD && !gesture_sent) {
        bleKeyboard.write(KEY_LEFT_ARROW); 
        actionStatus = "SENT: <- LEFT"; 
        gesture_sent = true;
      } else if (filteredY <= -GESTURE_THRESHOLD && !gesture_sent) {
        bleKeyboard.write(KEY_RIGHT_ARROW); 
        actionStatus = "SENT: RIGHT ->"; 
        gesture_sent = true;
      }
    }

    if (abs(filteredY) < 3) { 
      gesture_sent = false;
      if (actionStatus.startsWith("SENT:")) { 
        actionStatus = "STATUS: IDLE"; 
      }
    }
  } else {
      actionStatus = "BLE Waiting..."; 
  }
}

// --- STATE 3.1: WRITE OUTPUT (WebSocket) ---
void sendDataToWebSocket() {
  if (webClientReady && ws.count() > 0) {
    int toggledModeStatus = (slideModeEnabled) ? 0 : 1; // 0=BRIGHT, 1=DARK
    String jsonData = "{\"x\":"+String(filteredX)+", \"y\":"+String(filteredY)+", \"z\":"+String(z_accel)+
                      ", \"ldr\":"+String(toggledModeStatus)+", \"action\":\""+actionStatus+"\"}";
    ws.textAll(jsonData);
  }
}

// --- STATE 3.2: WRITE OUTPUT (OLED) ---
void updateOLEDDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.printf("X: %-7.2f\n", filteredX);
  display.printf("Y: %-7.2f\n", filteredY);
  display.printf("Z: %-7.2f\n", z_accel);
  display.println("---------------------");

  display.print("MODE: "); 
  display.println(slideModeEnabled ? "BRIGHT" : "DARK");
  display.print("BLE: ");
  display.println(bleKeyboard.isConnected() ? "Connected" : "Waiting...");
  display.print("IP: ");
  display.println(WiFi.localIP());
  display.println("---------------------");
  
  display.println(actionStatus);
  
  display.display();
}

// --- NEW MAIN LOOP (State Machine) ---
void loop() {
  ws.cleanupClients(); 

  if ((millis() - lastTime) > timerDelay) {
    lastTime = millis();
    
    // --- STATE 1: READ INPUTS ---
    readAllSensors(); 
    
    // --- STATE 2: PROCESS LOGIC ---
    updateLDRToggleState();
    runKalmanFilter();
    updateGestureState();
    
    // --- STATE 3: WRITE OUTPUTS ---
    sendDataToWebSocket();
    updateOLEDDisplay();
  }
}