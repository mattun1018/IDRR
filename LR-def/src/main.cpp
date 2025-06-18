#include <Arduino.h>
#include <DynamixelShield.h> // ライブラリをDynamixelShieldに変更
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// --- ライブラリのデフォルト定義を上書き ---
// DynamixelShield.h内で定義されているマクロを一旦未定義にする
#undef DXL_SERIAL
#undef DXL_DIR_PIN

// ESP32用に再定義する
#define DXL_SERIAL Serial2 // DXL通信に使用するシリアルポート
#define DXL_DIR_PIN 4      // 方向制御ピン

// =================== DYNAMIXEL 定義 (DynamixelShieldライブラリ用) ===================
const float DXL_PROTOCOL_VERSION = 1.0;
const int TIMEOUT = 10; // 通信タイムアウト(ms)

// AX-12A コントロールテーブルアドレス
const uint8_t ADDR_TORQUE_ENABLE = 24;
const uint8_t ADDR_GOAL_POSITION = 30;
const uint8_t ADDR_MOVING_SPEED = 32;
const uint8_t ADDR_CW_ANGLE_LIMIT = 6;
const uint8_t ADDR_CCW_ANGLE_LIMIT = 8;

// データ長
const uint8_t LEN_1_BYTE = 1;
const uint8_t LEN_2_BYTE = 2;

// モーター設定
const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const uint16_t TORQUE_ON = 1;
const uint16_t TORQUE_OFF = 0;
const uint16_t MAX_SPEED = 1023; // 0-1023
const uint16_t ANGLE_LIMIT_MIN = 0;
const uint16_t ANGLE_LIMIT_MAX = 1023;
const float neutralDeg = 0.0;

// DynamixelShieldオブジェクトを生成 (コンストラクタは引数を取らない)
DynamixelShield dxl;

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

// =================== DXL 初期化と操作関数 (DynamixelShield仕様) ===================

// 角度(deg)をポジション値(0-1023)に変換
uint16_t angleToValue(float degree)
{
  return constrain(map(degree, -150, 150, 0, 1023), 0, 1023);
}

void moveToPosition(uint8_t id, uint16_t pos)
{
  dxl.write(id, ADDR_GOAL_POSITION, (uint8_t *)&pos, LEN_2_BYTE, TIMEOUT);
}

void moveToPositionDegrees(uint8_t id, float deg)
{
  moveToPosition(id, angleToValue(deg));
}

void setupDxl(uint8_t id)
{
  uint8_t torque_off_val = TORQUE_OFF;
  uint8_t torque_on_val = TORQUE_ON;
  uint16_t angle_min = ANGLE_LIMIT_MIN;
  uint16_t angle_max = ANGLE_LIMIT_MAX;
  uint16_t speed = MAX_SPEED;

  dxl.write(id, ADDR_TORQUE_ENABLE, &torque_off_val, LEN_1_BYTE, TIMEOUT);        // 一旦トルクオフ
  dxl.write(id, ADDR_CW_ANGLE_LIMIT, (uint8_t *)&angle_min, LEN_2_BYTE, TIMEOUT); // 関節モードに設定
  dxl.write(id, ADDR_CCW_ANGLE_LIMIT, (uint8_t *)&angle_max, LEN_2_BYTE, TIMEOUT);
  dxl.write(id, ADDR_MOVING_SPEED, (uint8_t *)&speed, LEN_2_BYTE, TIMEOUT); // 速度を設定
  dxl.write(id, ADDR_TORQUE_ENABLE, &torque_on_val, LEN_1_BYTE, TIMEOUT);   // トルクオン
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
      Serial.println("Async: Motor1 -> -90");
      altState = ALT_M2_FORWARD;
      break;
    case ALT_M2_FORWARD:
      moveToPositionDegrees(DXL_ID2, 90);
      Serial.println("Async: Motor2 -> 90");
      altState = ALT_M1_BACK;
      break;
    case ALT_M1_BACK:
      moveToPositionDegrees(DXL_ID1, 90);
      Serial.println("Async: Motor1 -> 90");
      altState = ALT_M2_BACK;
      break;
    case ALT_M2_BACK:
      moveToPositionDegrees(DXL_ID2, -90);
      Serial.println("Async: Motor2 -> -90");
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
  Serial.print("コマンド受信: ");
  Serial.println(cmd);

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
    if (rx.length() > 0)
    {
      String cmd = String(rx.c_str());
      handleCommand(cmd);
    }
  }
};

// =================== setup ===================
void setup()
{
  // デバッグ用シリアルを初期化
  Serial.begin(115200);

  // Dynamixel 初期化
  // AX-12Aのボーレートは工場出荷時1,000,000の場合が多いため、1000000に設定します。
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  setupDxl(DXL_ID1);
  setupDxl(DXL_ID2);
  calibAll();
  Serial.println("Dynamixel setup complete with DynamixelShield library.");

  // BLE 初期化
  BLEDevice::init("DynamixelCtrlESP32_AX12A");
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
  // 非同期動作を更新
  updateAltMotion();

  // シリアルモニタからのコマンド入力を処理
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // 前後の空白や改行コードを削除
    if (cmd.length() > 0)
    {
      handleCommand(cmd);
    }
  }
}
