#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

HardwareSerial SerialFPGA(2);
#define SDA_PIN  8
#define SCL_PIN  9
#define FPGA_RX  17
#define FPGA_TX  18

#define TRIG_PIN 7
#define ECHO_PIN 4

const uint8_t MPU6050_ADDR = 0x68;
const uint8_t PWR_MGMT_1   = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;

float roll  = 0.0;
float pitch = 0.0;
float yaw   = 0.0;
const float alpha = 0.98;
unsigned long lastTime;

bool calFinish = false;
unsigned long calStartTime = 0;
const unsigned long CAL_TIME = 3000;
float gx_sum = 0, gy_sum = 0, gz_sum = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

BLEServer *pServer = NULL;
BLECharacteristic *pTxChara;
BLECharacteristic *pRxChara;
bool deviceConnected = false;

#define SERVICE_UUID        "6E400001-B5A3-5378-0400-020000000000"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-5378-0400-020000000000"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-5378-0400-020000000000"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

class MyCharaCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChara) {
      String rx = pChara->getValue();
      // 清除换行、空格
      rx.trim();
      Serial.print("收到蓝牙指令: ");
      Serial.println(rx); // 串口打印

      if(rx == "F") { SerialFPGA.print("#F;"); Serial.println("→发送 #F;"); }
      if(rx == "S") { SerialFPGA.print("#S;"); Serial.println("→发送 #S;"); }
      if(rx == "B") { SerialFPGA.print("#B;"); Serial.println("→发送 #B;"); }
      if(rx == "L") { SerialFPGA.print("#L;"); Serial.println("→发送 #L;"); }
      if(rx == "R") { SerialFPGA.print("#R;"); Serial.println("→发送 #R;"); }
      if(rx == "A") { SerialFPGA.print("#A;"); Serial.println("→发送 #A;"); }
      if(rx == "M") { SerialFPGA.print("#M;"); Serial.println("→发送 #M;"); }
    }
};


void readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR,14,true);
  int16_t ax_raw = Wire.read()<<8 | Wire.read();
  int16_t ay_raw = Wire.read()<<8 | Wire.read();
  int16_t az_raw = Wire.read()<<8 | Wire.read();
  int16_t gx_raw = Wire.read()<<8 | Wire.read();
  int16_t gy_raw = Wire.read()<<8 | Wire.read();
  int16_t gz_raw = Wire.read()<<8 | Wire.read();

  ax = ax_raw / 16384.0f;
  ay = ay_raw / 16384.0f;
  az = az_raw / 16384.0f;
  gx = (gx_raw / 131.0f) - gx_offset;
  gy = (gy_raw / 131.0f) - gy_offset;
  gz = (gz_raw / 131.0f) - gz_offset;
}

float readUltrasonic()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long t = pulseIn(ECHO_PIN, HIGH, 30000);
  if(t == 0) return 999.0f;
  float d = t * 0.034f / 2.0f;
  return (d>400)?999:d;
}

void setup() {
  Serial.begin(115200);
  SerialFPGA.begin(9600, SERIAL_8N1, FPGA_RX, FPGA_TX);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);

  BLEDevice::init("ESP32-CAR");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxChara = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxChara->addDescriptor(new BLE2902());
  pRxChara = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxChara->setCallbacks(new MyCharaCallbacks());
  pService->start();
  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  calStartTime = millis();
  lastTime = micros();
}

void loop() {
  float ax,ay,az,gx,gy,gz;
  readMPU(ax,ay,az,gx,gy,gz);

  if(!calFinish)
  {
    if(millis() - calStartTime < CAL_TIME)
    {
      gx_sum += gx; gy_sum += gy; gz_sum += gz;
      delay(2);
      return;
    }
    else
    {
      gx_offset = gx_sum / (CAL_TIME/2);
      gy_offset = gy_sum / (CAL_TIME/2);
      gz_offset = gz_sum / (CAL_TIME/2);
      calFinish = true;
      Serial.println("校准完成");
    }
  }

  unsigned long now = micros();
  float dt = (now - lastTime)/1000000.0f;
  lastTime = now;

  float acc_roll  = atan2(ay, az) * 180.0 / PI;
  float acc_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI - 10.7;
  roll  = alpha*(roll + gx*dt) + (1-alpha)*acc_roll;
  pitch = alpha*(pitch + gy*dt) + (1-alpha)*acc_pitch;
  yaw   = yaw + gz*dt;

  float dist = readUltrasonic();
  Serial.print("ax:");Serial.print(ax);
  Serial.print(" ay:");Serial.print(ay);
  Serial.print(" az:");Serial.print(az);
  Serial.print(" | roll:");Serial.print(roll);
  Serial.print(" pitch:");Serial.print(pitch);
  Serial.print(" yaw:");Serial.print(yaw);
  Serial.print(" dist:");Serial.println(dist);

  SerialFPGA.printf("A:%.2f,%.2f,%.2f,%.1f;", roll, pitch, yaw, dist);
  delay(50);
}
