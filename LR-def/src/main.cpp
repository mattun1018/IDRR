#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// =================== ピン設定 (ESP32-C3 SuperMini用) ===================
// Dynamixelとの通信に使用するシリアルポートとピンを指定します
// ESP32-C3にはSerial2がないため、Serial1を使用します
#define DXL_SERIAL Serial1
const int DXL_RX_PIN = 8;      // Serial1のRXピン(以前は16,32 TX_OUT)
const int DXL_TX_PIN = 9;      // Serial1のTXピン(以前は17,27 RX_IN)
const uint8_t DXL_DIR_PIN = 4; // モーターの方向制御ピン 33

// =================== DYNAMIXEL 設定 ===================
const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const float DXL_PROTOCOL_VERSION = 2.0;
const float neutralDeg = 0.0;

// Dynamixel2Arduinoのインスタンスを作成
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// =================== BLE 設定 ===================
BLECharacteristic *pCommandCharacteristic = nullptr;

const char *SERVICE_UUID = "180C";
const char *CHARACTERISTIC_UUID = "2A56";

// =================== 非同期動作制御用 ===================
enum AltState
{
  ALT_IDLE,
  ALT_M1_FORWARD,
  ALT_M2_FORWARD,
  ALT_M1_BACK,
  ALT_M2_BACK
};
AltState altState = ALT_IDLE;
unsigned long altStartTime = 0;
const unsigned long altInterval = 500;
bool altMotionActive = false;

// =================== DXL 初期化と操作関数 ===================
void setupDxl(uint8_t id)
{
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
}

void moveToPositionDegrees(uint8_t id, float deg)
{
  // 角度範囲の制限
  if (deg < -180.0)
    deg = -180.0;
  if (deg > 180.0)
    deg = 180.0;

  // Dynamixel内部単位へ変換（中心が2048）
  const float DEGREE_TO_UNIT = 11.377777; // ＝4095 / 360
  const int CENTER_POSITION = 2048;

  int pos = CENTER_POSITION + int(deg * DEGREE_TO_UNIT);

  // 安全に制限（オーバーフロー防止）
  pos = constrain(pos, 0, 4095);

  dxl.setGoalPosition(id, pos, UNIT_RAW);
}

void moveToAndReturnDegrees(uint8_t id, float deg, int wait_ms, bool ret)
{
  moveToPositionDegrees(id, deg);
  delay(wait_ms);
  if (ret)
    moveToPositionDegrees(id, neutralDeg);
}

void calibAll()
{
  moveToPositionDegrees(DXL_ID1, neutralDeg);
  moveToPositionDegrees(DXL_ID2, neutralDeg);
}

// =================== 非同期交互動作 ===================
void updateAltMotion()
{
  if (!altMotionActive)
    return;
  unsigned long now = millis();
  if (now - altStartTime >= altInterval)
  {
    altStartTime = now;
    switch (altState)
    {
    case ALT_M1_FORWARD:
      moveToPositionDegrees(DXL_ID1, -90);
      altState = ALT_M2_FORWARD;
      break;
    case ALT_M2_FORWARD:
      moveToPositionDegrees(DXL_ID2, 90);
      altState = ALT_M1_BACK;
      break;
    case ALT_M1_BACK:
      moveToPositionDegrees(DXL_ID1, 90);
      altState = ALT_M2_BACK;
      break;
    case ALT_M2_BACK:
      moveToPositionDegrees(DXL_ID2, -90);
      altState = ALT_M1_FORWARD;
      break;
    default:
      break;
    }
  }
}

// =================== コマンド処理関数 ===================
void handleCommand(const String &cmd)
{
  Serial.print("BLEコマンド受信: ");
  Serial.println(cmd);

  // 各コマンドに対応する動作
  if (cmd == "wave_forward")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID1, -90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, -90, 300, true);
    }
  }
  else if (cmd == "wave_return")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID1, -90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, -90, 300, true);
    }
    delay(1000);
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID2, -90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, -90, 300, true);
    }
  }
  else if (cmd == "wave_back")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID2, -90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, -90, 300, true);
    }
  }
  else if (cmd == "wave_large")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID1, -120, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 120, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 120, 300, true);
      moveToAndReturnDegrees(DXL_ID2, -120, 300, true);
    }
  }
  else if (cmd == "wave_random")
  {
    for (int i = 0; i < 5; i++)
    {
      float a1 = random(-120, 121);
      float a2 = random(-120, 121);
      int d1 = random(200, 800);
      int d2 = random(200, 800);
      moveToAndReturnDegrees(DXL_ID1, a1, d1, false);
      moveToAndReturnDegrees(DXL_ID2, a2, d2, true);
    }
  }
  else if (cmd == "wave_left")
  {
    moveToAndReturnDegrees(DXL_ID1, -90, 2000, false);
    moveToAndReturnDegrees(DXL_ID2, 90, 1000, true);
  }
  else if (cmd == "start_async")
  {
    altMotionActive = true;
    altState = ALT_M1_FORWARD;
    altStartTime = millis();
    Serial.println("Async alternating motion started");
  }
  else if (cmd == "stop_async")
  {
    altMotionActive = false;
    altState = ALT_IDLE;
    calibAll();
    Serial.println("Async alternating motion stopped");
  }
  else if (cmd == "calibrate")
  {
    calibAll();
  }
  else if (cmd.startsWith("set_1_"))
  {
    float angle = cmd.substring(6).toFloat();
    moveToPositionDegrees(DXL_ID1, angle);
    Serial.print("Motor1 set to angle: ");
    Serial.println(angle);
  }
  else if (cmd.startsWith("set_2_"))
  {
    float angle = cmd.substring(6).toFloat();
    moveToPositionDegrees(DXL_ID2, angle);
    Serial.print("Motor2 set to angle: ");
    Serial.println(angle);
  }
}

// =================== BLEコールバッククラス ===================
class CommandCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic) override
  {
    std::string rx = pCharacteristic->getValue();
    String cmd = String(rx.c_str());
    handleCommand(cmd);
  }
};

// =================== setup ===================
void setup()
{
  Serial.begin(115200);

  // --- MODIFIED FOR ESP32-C3 ---
  // Dynamixel通信用のシリアルポート(Serial1)を、指定したピンで初期化
  DXL_SERIAL.begin(57600, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);

  // Dynamixelライブラリを初期化
  // dxl.begin()はシリアルポートのボーレートのみを設定するため、begin()の前に
  // シリアルポート自体の初期化が必要です。
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // 各モーターをセットアップ
  setupDxl(DXL_ID1);
  setupDxl(DXL_ID2);
  calibAll();
  Serial.println("Dynamixel Motors Initialized.");

  // BLE 初期化
  BLEDevice::init("DynamixelCtrlESP32-C3");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCommandCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  pCommandCharacteristic->setCallbacks(new CommandCallback());

  pService->start();
  BLEDevice::getAdvertising()->start();
  Serial.println("BLE Ready. Connect and write commands.");
}

// =================== loop ===================
void loop()
{
  updateAltMotion();
}
