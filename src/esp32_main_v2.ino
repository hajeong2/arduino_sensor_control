#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "esp_wifi.h"
#include <ESP32PWM.h>
#include <lwip/sockets.h>

// ============================================================
// ✅ Gate 구조체
// ============================================================
struct Gate {
  bool warmed = false;
  bool armed = false;
  int lastLevel = HIGH;
  uint32_t warmStartMs = 0, highStart = 0, lowStart = 0;
  uint32_t ticksTot = 0, ticksAcc = 0;
};

// ==================== Wi-Fi ====================
const char* ssid     = "tambbb";
const char* password = "tambbb123";
WiFiServer server(3333);
WiFiClient client;

// ==================== Servo / ESC ====================
Servo servo1, servo2, esc;
#define ESC_PIN 27
int servo1Angle = 15;
int servo2Angle = 45;
int escPower    = 0;
SemaphoreHandle_t servoMutex;

// ==================== KeepAlive ====================
#define KEEPALIVE_IDLE     10
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT    3
void setupKeepAlive(WiFiClient& c) {
  int sock = c.fd();
  int yes = 1, idle = KEEPALIVE_IDLE, intv = KEEPALIVE_INTERVAL, cnt = KEEPALIVE_COUNT;
  setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(int));
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(int));
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intv, sizeof(int));
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(int));
  c.setNoDelay(true);
}

// ==================== Timecode ====================
unsigned long baseTime = 0;
bool timeStarted = false;

void setBaseTime() {
  baseTime = millis() + 1;    // ✅ 1ms 보정
  timeStarted = true;
  Serial.printf("🎯 기준점 설정됨 (t₀=%lu)\n", baseTime);
}

unsigned long getRelativeTime() {
  if (!timeStarted) return 0;
  unsigned long now = millis();
  if (now < baseTime) return 0;
  return now - baseTime;
}

// ==================== LiDAR ====================
HardwareSerial LIDARserial(2);
#define LIDAR_RX 16
#define LIDAR_TX 17
int lidarDistance = 0;
int lidarStrength = 0;

bool lidar_read() {
  static uint8_t buf[9];
  static int idx = 0;
  while (LIDARserial.available()) {
    uint8_t b = LIDARserial.read();
    if (idx == 0 && b != 0x59) continue;
    if (idx == 1 && b != 0x59) { idx = 0; continue; }
    buf[idx++] = b;
    if (idx == 9) {
      idx = 0;
      uint8_t sum = 0;
      for (int i = 0; i < 8; i++) sum += buf[i];
      if (sum != buf[8]) return false;
      lidarDistance = buf[2] | (buf[3] << 8);
      lidarStrength = buf[4] | (buf[5] << 8);
      return true;
    }
  }
  return false;
}

// ==================== IR 센서 ====================
#define IR_PIN 33
int irRaw = 0;
float irVolt = 0;
float irDistCm = 0;

// ==================== Encoder ====================
#define LEFT_PIN  34
#define RIGHT_PIN 35
static const float WHEEL_DIAMETER_M = 0.065f;
static const int   MAGNETS_PER_WHEEL = 5;
static const float METERS_PER_TICK = (M_PI * WHEEL_DIAMETER_M) / MAGNETS_PER_WHEEL;

Gate L, R;
inline void updateGate(Gate& g, int pin) {
  uint32_t nowUs = micros();
  int lvl = digitalRead(pin);
  if (!g.warmed) {
    if (g.warmStartMs == 0) g.warmStartMs = millis();
    if (millis() - g.warmStartMs > 800) { g.warmed = true; g.lastLevel = lvl; }
    return;
  }
  if (lvl == HIGH) {
    if (g.lastLevel == LOW) { g.highStart = nowUs; g.lowStart = 0; }
    else if (!g.armed && g.highStart && nowUs - g.highStart >= 6000) g.armed = true;
  } else {
    if (g.lastLevel == HIGH) g.lowStart = g.armed ? nowUs : 0;
    else if (g.armed && g.lowStart && nowUs - g.lowStart >= 4000) {
      g.ticksTot++; g.ticksAcc++; g.armed = false; g.lowStart = 0;
    }
  }
  g.lastLevel = lvl;
}

// ==================== 좌표계 스위치 ====================
#define SWAP_XY_DEFAULT  true
#define INVERT_X_DEFAULT false
#define INVERT_Y_DEFAULT false
volatile bool SWAP_XY  = SWAP_XY_DEFAULT;
volatile bool INVERT_X = INVERT_X_DEFAULT;
volatile bool INVERT_Y = INVERT_Y_DEFAULT;

// ==================== IMU ====================
#define MPU6050_ADDR 0x68
float bgz = 0, yaw_f = 0;
#define DEG2RAD (3.1415926535f / 180.0f)
#define RAD2DEG (180.0f / 3.1415926535f)
#define YAW_SIGN +1
#define HEADING_OFFSET_DEG 0.0f

bool mpu_read(float &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);
  if (Wire.available() != 14) return false;
  for (int i = 0; i < 12; i++) Wire.read();
  int16_t gzr = (Wire.read() << 8) | Wire.read();
  gz = gzr / 131.0f * DEG2RAD;
  return true;
}

void mpu_init() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1B); Wire.write(0); Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1C); Wire.write(0); Wire.endTransmission();
  Serial.println("✅ IMU Ready (MPU6050)");
  float s = 0; for (int i = 0; i < 200; i++) { float g; if (mpu_read(g)) s += g; delay(5); }
  bgz = s / 200;
  Serial.printf("Gyro bias=%.5f\n", bgz);
}

// ==================== Odometry ====================
float posX = 0, posY = 0;
int motionDir = 0;

// ==================== TaskCore0 : 센서 + 제어 ====================
void TaskCore0(void*) {
  static unsigned long lastSend = 0;
  float gz = 0;
  while (true) {
    // Servo / ESC
    if (xSemaphoreTake(servoMutex, portMAX_DELAY)) {
      servo1.write(servo1Angle);
      servo2.write(servo2Angle);
      esc.writeMicroseconds(map(escPower, 0, 100, 1000, 2000));
      xSemaphoreGive(servoMutex);
    }

    // IMU
    if (mpu_read(gz)) { gz -= bgz; yaw_f += (YAW_SIGN * gz) * 0.02f; }

    // LiDAR / IR
    lidar_read();
    irRaw = analogRead(IR_PIN);
    irVolt = irRaw * (3.3f / 4095.0f);
    irDistCm = 27.86f * pow(irVolt, -1.15f);

    // Encoder
    updateGate(L, LEFT_PIN); updateGate(R, RIGHT_PIN);
    uint32_t dL = L.ticksAcc, dR = R.ticksAcc; L.ticksAcc = R.ticksAcc = 0;
    float ds = motionDir * 0.5f * (dL + dR) * METERS_PER_TICK;
    float heading = (yaw_f * RAD2DEG + HEADING_OFFSET_DEG) * DEG2RAD;
    float dX = ds * sinf(heading), dY = ds * cosf(heading);
    posX += SWAP_XY ? dY : dX;
    posY += SWAP_XY ? dX : dY;
    if (INVERT_X) posX = -posX;
    if (INVERT_Y) posY = -posY;

    // Node-RED 전송
    if (millis() - lastSend > 300 && client.connected()) {
      lastSend = millis();
      StaticJsonDocument<256> doc;
      doc["yaw"] = yaw_f * RAD2DEG;
      doc["lidar"] = lidarDistance;
      doc["ir"] = irDistCm;
      doc["x"] = posX;
      doc["y"] = posY;
      doc["t_s"] = (unsigned long)(getRelativeTime() / 1000UL) + 1;  // ✅ 초 단위 타임코드 (1초에 +1)
      doc["dir"] = (motionDir == 1) ? "forward" : (motionDir == -1) ? "backward" : "stop";
      serializeJson(doc, client);
      client.println();
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ==================== TaskCore1 : WiFi + 명령 ====================
void TaskCore1(void*) {
  while (true) {
    if (!client || !client.connected()) {
      WiFiClient newClient = server.accept();
      if (newClient) {
        client = newClient;
        setupKeepAlive(client);
        Serial.println("📩 Node-RED 연결됨");
      }
      vTaskDelay(50 / portTICK_PERIOD_MS);
      continue;
    }

    static String buf = "";
    while (client.available()) {
      char c = client.read();
      if (c == '\n') {
        buf.trim();
        if (buf.length() > 0) {
          if (buf == "RESET_TIME") { setBaseTime(); buf = ""; continue; }

          StaticJsonDocument<256> doc;
          DeserializationError err = deserializeJson(doc, buf);
          if (!err) {
            String cmd = doc["cmd"] | "";
            if (cmd == "servo") {
              int id = doc["id"] | 0;
              int angle = constrain((int)doc["angle"], 0, 180);
              if (xSemaphoreTake(servoMutex, portMAX_DELAY)) {
                if (id == 1) servo1Angle = angle;
                else if (id == 2) servo2Angle = angle;
                xSemaphoreGive(servoMutex);
              }
            } else if (cmd == "esc") {
              int p = constrain((int)doc["power"], 0, 100);
              if (xSemaphoreTake(servoMutex, portMAX_DELAY)) { escPower = p; xSemaphoreGive(servoMutex); }
            } else if (cmd == "dir") {
              motionDir = doc["value"].as<int>();
            } else if (cmd == "coord") {
              if (doc.containsKey("swap")) SWAP_XY = doc["swap"].as<bool>();
              if (doc.containsKey("ix")) INVERT_X = doc["ix"].as<bool>();
              if (doc.containsKey("iy")) INVERT_Y = doc["iy"].as<bool>();
            }
          }
        }
        buf = "";
      } else {
        if (buf.length() < 240) buf += c; else buf = "";
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ==================== setup ====================
void setup() {
  Serial.begin(115200);
  Serial.println("🚀 ESP32 DualCore (Node-RED 전용, Timecode 안정화)");

  mpu_init();
  pinMode(LEFT_PIN, INPUT);
  pinMode(RIGHT_PIN, INPUT);
  pinMode(IR_PIN, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  Serial.printf("\n✅ Wi-Fi OK IP=%s\n", WiFi.localIP().toString().c_str());
  server.begin();

  LIDARserial.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  ESP32PWM::allocateTimer(0);
  servo1.attach(25, 500, 2500);
  servo2.attach(26, 500, 2500);
  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(1000);
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  servoMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(TaskCore0, "Core0_Task", 8192, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(TaskCore1, "Core1_Task", 8192, nullptr, 1, nullptr, 1);

  Serial.println("✅ Dual Core Tasks Started");
}

// ==================== loop ====================
void loop() { vTaskDelay(1000 / portTICK_PERIOD_MS); }
