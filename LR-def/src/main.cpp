#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// =================== ハードウェア設定 ===================
#define DXL_SERIAL Serial2
const uint8_t DXL_DIR_PIN = 4;

// =================== DYNAMIXEL 設定 ===================
const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const float neutralDeg = 0.0; // 中央位置の角度

// Dynamixel2Arduinoオブジェクトを生成
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// モーターの接続状態を追跡するフラグ
bool isMotor1Connected = false;
bool isMotor2Connected = false;

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

// =================== DXL 操作関数 ===================

// 角度(degree)をDynamixelのポジション値(0-1023)に変換
uint16_t angleToValue(float deg)
{
  // AX-12Aの動作範囲 (-150度から150度) に角度を制限
  deg = constrain(deg, -150.0, 150.0);
  // 角度をポジション値にマッピング
  return map(deg, -150, 150, 0, 1023);
}

void moveToPositionDegrees(uint8_t id, float deg)
{
  // ★変更点: モーターが接続されていない場合は、コマンドを送信せずに処理を抜ける
  if (id == DXL_ID1 && !isMotor1Connected)
    return;
  if (id == DXL_ID2 && !isMotor2Connected)
    return;

  uint16_t pos = angleToValue(deg);
  dxl.setGoalPosition(id, pos, UNIT_RAW);
}

void moveToAndReturnDegrees(uint8_t id, float deg, int wait_ms, bool ret)
{
  moveToPositionDegrees(id, deg);
  delay(wait_ms);
  if (ret)
  {
    moveToPositionDegrees(id, neutralDeg);
  }
}

void calibAll()
{
  moveToPositionDegrees(DXL_ID1, neutralDeg);
  moveToPositionDegrees(DXL_ID2, neutralDeg);
}

// モーターの初期設定を行う関数
void setupDxl(uint8_t id)
{
  // 関節モード(ポジションコントロール)に設定
  if (dxl.setOperatingMode(id, OP_POSITION))
  {
    Serial.print("モーター(ID:");
    Serial.print(id);
    Serial.println(") の関節モード設定に成功");
  }
  else
  {
    Serial.print("モーター(ID:");
    Serial.print(id);
    Serial.println(") の関節モード設定に失敗");
  }
  // トルクをオンにする
  if (dxl.torqueOn(id))
  {
    Serial.print("モーター(ID:");
    Serial.print(id);
    Serial.println(") のトルクONに成功");
  }
  else
  {
    Serial.print("モーター(ID:");
    Serial.print(id);
    Serial.println(") のトルクONに失敗");
  }
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
  while (!Serial)
    ;

  Serial.println("AX-12A フル機能コントローラーを起動します。");

  // Dynamixel 初期化 (動作確認が取れた1,000,000bpsで設定)
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(1.0);

  // --- ★変更点: 診断機能を修正 ---
  // モーター1の確認
  if (dxl.ping(DXL_ID1))
  {
    Serial.print("モーター(ID: ");
    Serial.print(DXL_ID1);
    Serial.println(") との通信に成功！");
    setupDxl(DXL_ID1);
    isMotor1Connected = true;
  }
  else
  {
    Serial.print("【警告】モーター(ID: ");
    Serial.print(DXL_ID1);
    Serial.println(") が応答しません。");
    isMotor1Connected = false;
  }
  // モーター2の確認
  if (dxl.ping(DXL_ID2))
  {
    Serial.print("モーター(ID: ");
    Serial.print(DXL_ID2);
    Serial.println(") との通信に成功！");
    setupDxl(DXL_ID2);
    isMotor2Connected = true;
  }
  else
  {
    Serial.print("【警告】モーター(ID: ");
    Serial.print(DXL_ID2);
    Serial.println(") が応答しません。");
    isMotor2Connected = false;
  }

  calibAll();
  Serial.println("Dynamixelのセットアップが完了しました。");

  // BLE 初期化
  BLEDevice::init("DynamixelCtrlESP32_AX-12");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCommandCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  pCommandCharacteristic->setCallbacks(new CommandCallback());

  pService->start();
  BLEDevice::getAdvertising()->start();
  Serial.println("BLEの準備ができました。接続を待っています...");
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
    cmd.trim();
    if (cmd.length() > 0)
    {
      handleCommand(cmd);
    }
  }
}
