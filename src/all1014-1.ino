#include <WiFi.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <lwip/sockets.h>
#include "esp_wifi.h"
#include "time.h"
#include <Wire.h>

// ==================== Wi-Fi ====================
const char* ssid     = "tambbb";
const char* password = "tambbb123";
WiFiServer server(3333);
WiFiClient client;

// ==================== Servo / ESC ====================
#include <ESP32PWM.h>
Servo servo1, servo2, esc;
#define ESC_PIN 27
int servo1Angle = 15;
int servo2Angle = 45;
int escPower    = 0;    // 0–100 %
SemaphoreHandle_t servoMutex;

// ==================== KeepAlive ====================
#define KEEPALIVE_IDLE     10
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT    3

// ==================== Timecode ====================
unsigned long baseTime = 0;
bool timeStarted = false;

// ==================== IMU ====================
#define MPU6050_ADDR 0x68
float bgz=0, yaw_f=0, yaw_forward_zero=0;
float posX=0, posY=0, dX=0, dY=0;

// ==================== LiDAR ====================
HardwareSerial LIDARserial(2);
#define LIDAR_RX 16
#define LIDAR_TX 17
int lidarDistance = 0;
int lidarStrength = 0;

// ==================== IR 센서 ====================
#define IR_PIN 33
int irRaw = 0;
float irVolt = 0;
float irDistCm = 0;

// ==================== 엔코더 ====================
#define LEFT_PIN   34
#define RIGHT_PIN  35

static const float WHEEL_DIAMETER_M  = 0.14f;
static const int   MAGNETS_PER_WHEEL = 5;
static const float WHEEL_BASE_M      = 0.28f;
static const float METERS_PER_TICK   = (M_PI * WHEEL_DIAMETER_M) / MAGNETS_PER_WHEEL;

struct Gate {
  bool warmed=false; bool armed=false; int lastLevel=HIGH;
  uint32_t warmStartMs=0, highStart=0, lowStart=0, ticksTot=0, ticksAcc=0;
};
Gate L,R;

// =================================================
// =============== 공용 함수 =======================
void setupKeepAlive(WiFiClient& c) {
  int sock = c.fd();
  int yes = 1, idle = KEEPALIVE_IDLE, intv = KEEPALIVE_INTERVAL, cnt = KEEPALIVE_COUNT;
  setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(int));
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(int));
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intv, sizeof(int));
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(int));
  c.setNoDelay(true);
}

void setBaseTime() {
  struct timeval now; gettimeofday(&now,nullptr);
  baseTime = now.tv_sec * 1000UL + now.tv_usec / 1000UL;
  timeStarted = true;
  Serial.printf("🎯 기준점 설정됨 (t₀=%lu)\n", baseTime);
}

unsigned long getRelativeTime() {
  if (!timeStarted) return 0;
  struct timeval now; gettimeofday(&now,nullptr);
  return now.tv_sec * 1000UL + now.tv_usec / 1000UL - baseTime;
}

// ==================== IMU ====================
bool mpu_read(float &gz) {
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR,14);
  if (Wire.available()!=14) return false;
  for(int i=0;i<12;i++) Wire.read(); // skip accel+temp+gx,gy
  int16_t gzr=(Wire.read()<<8)|Wire.read();
  gz = gzr/131.0f*DEG_TO_RAD;
  return true;
}

void mpu_init() {
  Wire.begin(21,22);
  Wire.setClock(400000);
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1B); Wire.write(0); Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1C); Wire.write(0); Wire.endTransmission();
  Serial.println("✅ IMU Ready (MPU6050 @0x68)");
  float s=0; for(int i=0;i<200;i++){float g; if(mpu_read(g)) s+=g; delay(5);} bgz=s/200;
  Serial.printf("Gyro bias=%.5f\n", bgz);
}

// ==================== LiDAR ====================
bool lidar_read() {
  while (LIDARserial.available() >= 2) {
    if (LIDARserial.peek() == 0x59) {
      LIDARserial.read();
      if (LIDARserial.available() && LIDARserial.peek() == 0x59) {
        LIDARserial.read();
        uint8_t buf[7];
        uint32_t startTime = millis();
        for (int i = 0; i < 7; ) {
          if (LIDARserial.available()) buf[i++] = LIDARserial.read();
          else if (millis() - startTime > 10) return false;
        }
        uint8_t distL = buf[0], distH = buf[1];
        uint8_t strL  = buf[2], strH  = buf[3];
        uint8_t tmpL  = buf[4], tmpH  = buf[5];
        uint8_t sumRx = buf[6];
        uint16_t sum = 0x59 + 0x59 + distL + distH + strL + strH + tmpL + tmpH;
        if ((sum & 0xFF) != sumRx) continue;
        lidarDistance = (distH << 8) | distL;
        lidarStrength = (strH << 8) | strL;
        return true;
      }
    } else LIDARserial.read();
  }
  return false;
}

// ==================== Encoder ====================
inline void updateGate(Gate& g,int pin){
  uint32_t nowUs=micros();
  int lvl=digitalRead(pin);
  if(!g.warmed){ if(g.warmStartMs==0) g.warmStartMs=millis();
    if(millis()-g.warmStartMs>800){g.warmed=true;g.lastLevel=lvl;}
    return;}
  if(lvl==HIGH){
    if(g.lastLevel==LOW){g.highStart=nowUs; g.lowStart=0;}
    else if(!g.armed && g.highStart && nowUs-g.highStart>=6000){g.armed=true;}
  }else{
    if(g.lastLevel==HIGH) g.lowStart=g.armed?nowUs:0;
    else if(g.armed && g.lowStart && nowUs-g.lowStart>=4000){
      g.ticksTot++; g.ticksAcc++; g.armed=false; g.lowStart=0;
    }
  }
  g.lastLevel=lvl;
}

// =================================================
// =============== Core0 Task ======================
void TaskCore0(void*) {
  static unsigned long lastSend=0;
  float gz=0;

  while(true){
    // ---- Servo/ESC ----
    if(xSemaphoreTake(servoMutex,portMAX_DELAY)){
      servo1.write(servo1Angle);
      servo2.write(servo2Angle);
      int pwm_us=map(escPower,0,100,1000,2000);   // ✅ 50Hz PWM 구간
      esc.writeMicroseconds(pwm_us);
      xSemaphoreGive(servoMutex);
    }

    // ---- IMU ----
    if(mpu_read(gz)){
      gz-=bgz;
      yaw_f += gz*0.02f;
    }

    // ---- LiDAR ----
    lidar_read();

    // ---- IR 센서 ----
    irRaw = analogRead(IR_PIN);
    irVolt = irRaw * (3.3f / 4095.0f);
    irDistCm = 27.86f * pow(irVolt, -1.15f);

    // ---- Encoder ----
    updateGate(L,LEFT_PIN); updateGate(R,RIGHT_PIN);
    uint32_t dL=L.ticksAcc; uint32_t dR=R.ticksAcc; L.ticksAcc=R.ticksAcc=0;
    float ds=0.5f*(dL+dR)*METERS_PER_TICK;
    float heading_y=yaw_f - yaw_forward_zero;
    dX=ds*sinf(heading_y); dY=ds*cosf(heading_y);
    posX+=dX; posY+=dY;

    // ---- Node-RED 송신 ----
    if(millis()-lastSend>300 && client.connected()){
      lastSend=millis();
      StaticJsonDocument<200> doc;
      doc["t_ms"]=getRelativeTime();
      doc["yaw"]=yaw_f*RAD_TO_DEG;
      doc["lidar"]=lidarDistance;
      doc["ir"]=irDistCm;
      doc["esc"]=escPower;
      doc["x"]=posX;
      doc["y"]=posY;
      serializeJson(doc,client); client.println();
    }

    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

// =================================================
// =============== setup ===========================
void setup(){
  Serial.begin(115200);
  Serial.println("🚀 ESP32 DualCore Servo+ESC+IMU+LiDAR+Encoder+IR");

  // IMU
  mpu_init();

  // LiDAR
  LIDARserial.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  Serial.println("✅ LiDAR Ready (UART2 RX=16, TX=17)");

  // IR 센서
  pinMode(IR_PIN, INPUT);
  Serial.println("✅ IR Sensor Ready (ADC pin 33)");

  // Encoder
  pinMode(LEFT_PIN,INPUT);
  pinMode(RIGHT_PIN,INPUT);

  // Wi-Fi
  WiFi.mode(WIFI_STA); WiFi.setSleep(false); esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.begin(ssid,password);
  while(WiFi.status()!=WL_CONNECTED){delay(400); Serial.print(".");}
  Serial.printf("\n✅ Wi-Fi OK IP=%s\n",WiFi.localIP().toString().c_str());
  server.begin();

  // =====================================================
  // ✅ Servo / ESC 초기화 (50Hz RC ESC)
  // =====================================================
  ESP32PWM::allocateTimer(0);
  bool ok1 = servo1.attach(25,500,2500);
  bool ok2 = servo2.attach(26,500,2500);
  esc.setPeriodHertz(50);          // ✅ RC ESC는 50 Hz
  bool ok3 = esc.attach(ESC_PIN,1000,2000);
  Serial.printf("Servo1=%s, Servo2=%s, ESC=%s\n",
                ok1 ? "OK":"FAIL", ok2 ? "OK":"FAIL", ok3 ? "OK":"FAIL");

  esc.writeMicroseconds(1000); // idle
  vTaskDelay(3000 / portTICK_PERIOD_MS);

  servoMutex=xSemaphoreCreateMutex();

  // Core0 task start
  xTaskCreatePinnedToCore(TaskCore0,"Core0",8192,nullptr,1,nullptr,0);
}

// =================================================
// =============== loop (Core1) ====================
void loop(){
  if(!client||!client.connected()){
    WiFiClient newClient=server.accept();
    if(newClient){client=newClient;setupKeepAlive(client);
      Serial.println("📩 Node-RED 연결됨");}
    delay(50); return;
  }

  static String buf="";
  while(client.available()){
    char c=client.read();
    if(c=='\n'){
      buf.trim();
      if(buf.length()>0){
        if(buf=="RESET_TIME"){setBaseTime();}
        else{
          StaticJsonDocument<128> doc;
          if(!deserializeJson(doc,buf)){
            if(doc["cmd"]=="servo"){
              int id=doc["id"],angle=constrain((int)doc["angle"],0,180);
              if(xSemaphoreTake(servoMutex,portMAX_DELAY)){
                if(id==1)servo1Angle=angle; else if(id==2)servo2Angle=angle;
                xSemaphoreGive(servoMutex);
              }
              Serial.printf("Servo%d=%d°\n",id,angle);
            }else if(doc["cmd"]=="esc"){
              int p=constrain((int)doc["power"],0,100);
              if(xSemaphoreTake(servoMutex,portMAX_DELAY)){escPower=p;xSemaphoreGive(servoMutex);}
              Serial.printf("ESC=%d%%\n",p);
            }
          }
        }
      }
      buf="";
    }else{
      if(buf.length()<200) buf+=c; else buf="";
    }
  }
}
