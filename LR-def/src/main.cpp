#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// =================== DYNAMIXEL 設定 ===================
#define DXL_SERIAL Serial2
const uint8_t DXL_DIR_PIN = 33;
const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const uint8_t DXL_ID3 = 3;
const uint8_t DXL_ID4 = 4;
const uint8_t DXL_ID5 = 5;
const float DXL_PROTOCOL_VERSION = 2.0;
const float neutralDeg = 5.0; // calibrateで戻す角度は0度に設定

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
  // 角度範囲を0〜360に制限
  if (deg < 0.0)
    deg = 0.0;
  if (deg > 360.0)
    deg = 360.0;

  // Dynamixel内部単位へ変換 (0°→0, 360°→4095)
  const float DEGREE_TO_UNIT = 11.377777; // ＝4095 / 360
  int pos = int(deg * DEGREE_TO_UNIT);

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
  moveToPositionDegrees(DXL_ID3, neutralDeg);
  moveToPositionDegrees(DXL_ID4, neutralDeg);
  moveToPositionDegrees(DXL_ID5, neutralDeg);
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
      moveToPositionDegrees(DXL_ID1, 270); // -90° → 270°
      altState = ALT_M2_FORWARD;
      break;
    case ALT_M2_FORWARD:
      moveToPositionDegrees(DXL_ID2, 90); // +90°
      altState = ALT_M1_BACK;
      break;
    case ALT_M1_BACK:
      moveToPositionDegrees(DXL_ID1, 90); // +90°
      altState = ALT_M2_BACK;
      break;
    case ALT_M2_BACK:
      moveToPositionDegrees(DXL_ID2, 270); // -90° → 270°
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

  if (cmd == "wave_forward")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID1, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 270, 300, true);
    }
  }
  else if (cmd == "wave_return")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID1, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 270, 300, true);
    }
    delay(1000);
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID2, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 270, 300, true);
    }
  }
  else if (cmd == "wave_back")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID2, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 270, 300, true);
    }
  }
  else if (cmd == "wave_large")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID1, 240, 300, true); // -120° → 240°
      moveToAndReturnDegrees(DXL_ID2, 120, 300, true); // +120°
      moveToAndReturnDegrees(DXL_ID1, 120, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 240, 300, true);
    }
  }
  else if (cmd == "wave_random")
  {
    for (int i = 0; i < 5; i++)
    {
      float a1 = random(0, 361); // 0〜360°
      float a2 = random(0, 361);
      int d1 = random(200, 800);
      int d2 = random(200, 800);
      moveToAndReturnDegrees(DXL_ID1, a1, d1, false);
      moveToAndReturnDegrees(DXL_ID2, a2, d2, true);
    }
  }
  else if (cmd == "wave_left")
  {
    moveToAndReturnDegrees(DXL_ID1, 270, 2000, false); // -90° → 270°
    moveToAndReturnDegrees(DXL_ID2, 90, 1000, true);   // +90°
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
  else if (cmd.startsWith("1_"))
  {
    float angle = cmd.substring(2).toFloat();
    moveToPositionDegrees(DXL_ID1, angle);
    Serial.print("Motor1 set to angle: ");
    Serial.println(angle);
  }
  else if (cmd.startsWith("2_"))
  {
    float angle = cmd.substring(2).toFloat();
    moveToPositionDegrees(DXL_ID2, angle);
    Serial.print("Motor2 set to angle: ");
    Serial.println(angle);
  }
  else if (cmd.startsWith("3_"))
  {
    float angle = cmd.substring(2).toFloat();
    moveToPositionDegrees(DXL_ID3, angle);
    Serial.print("Motor3 set to angle: ");
    Serial.println(angle);
  }
  else if (cmd.startsWith("4_"))
  {
    float angle = cmd.substring(2).toFloat();
    moveToPositionDegrees(DXL_ID4, angle);
    Serial.print("Motor4 set to angle: ");
    Serial.println(angle);
  }
  else if (cmd.startsWith("5_"))
  {
    float angle = cmd.substring(2).toFloat();
    moveToPositionDegrees(DXL_ID5, angle);
    Serial.print("Motor5 set to angle: ");
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
  Serial2.begin(57600, SERIAL_8N1, 32, 27);

  // Dynamixel 初期化
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  setupDxl(DXL_ID1);
  setupDxl(DXL_ID2);
  setupDxl(DXL_ID3);
  setupDxl(DXL_ID4);
  setupDxl(DXL_ID5);
  calibAll();

  // BLE 初期化
  BLEDevice::init("DynamixelCtrlESP32");
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