#include <Arduino.h>
#include <DynamixelShield.h>
#include <ArduinoBLE.h>

// --- シリアル設定 ---
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8);
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif

// --- DYNAMIXEL定義（XL430用：Protocol 2.0） ---
#define DXL_PROTOCOL_VERSION 2.0
#define TIMEOUT 10

#define TORQUE_ENABLE_ADDR 64
#define GOAL_POSITION_ADDR 116
#define VELOCITY_ADDR 104
#define ADDR_LEN_1B 1
#define ADDR_LEN_2B 2
#define ADDR_LEN_4B 4

const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;

const uint32_t calibSpeed = 200; // 約45 rpm
const uint16_t angleLimitMin = 0;
const uint16_t angleLimitMax = 4095;
const uint16_t neutralPosition = 2048;
const uint16_t calibPositions[] = {2048, 2048};

DynamixelShield dxl;

// BLE定義
BLEService controlService("180C");
BLECharacteristic commandChar("2A56", BLEWrite, 20);

// --- 補助関数 ---
uint16_t angleToValue(float degree)
{
  return constrain(map(degree, -180, 180, 0, 4095), 0, 4095);
}

void setupDxl(uint8_t id)
{
  uint8_t torque_off = 0, torque_on = 1;
  dxl.write(id, TORQUE_ENABLE_ADDR, &torque_off, ADDR_LEN_1B, TIMEOUT);
  dxl.write(id, VELOCITY_ADDR, (uint8_t *)&calibSpeed, ADDR_LEN_4B, TIMEOUT);
  dxl.write(id, TORQUE_ENABLE_ADDR, &torque_on, ADDR_LEN_1B, TIMEOUT);
}

void moveToPosition(uint8_t id, uint16_t pos)
{
  dxl.write(id, GOAL_POSITION_ADDR, (uint8_t *)&pos, ADDR_LEN_4B, TIMEOUT);
}
void moveToPositionDegrees(uint8_t id, float deg)
{
  moveToPosition(id, angleToValue(deg));
}
void moveToAndReturn(uint8_t id, uint16_t pos, int wait_ms, bool ret)
{
  moveToPosition(id, pos);
  delay(wait_ms);
  if (ret)
    moveToPosition(id, neutralPosition);
}
void moveToAndReturnDegrees(uint8_t id, float deg, int wait_ms, bool ret)
{
  moveToAndReturn(id, angleToValue(deg), wait_ms, ret);
}
void calibAll()
{
  moveToPosition(DXL_ID1, calibPositions[0]);
  moveToPosition(DXL_ID2, calibPositions[1]);
}

// 非同期交互動作用
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
      DEBUG_SERIAL.println("Motor1 → -90");
      altState = ALT_M2_FORWARD;
      break;
    case ALT_M2_FORWARD:
      moveToPositionDegrees(DXL_ID2, 90);
      DEBUG_SERIAL.println("Motor2 → 90");
      altState = ALT_M1_BACK;
      break;
    case ALT_M1_BACK:
      moveToPositionDegrees(DXL_ID1, 90);
      DEBUG_SERIAL.println("Motor1 → 90");
      altState = ALT_M2_BACK;
      break;
    case ALT_M2_BACK:
      moveToPositionDegrees(DXL_ID2, -90);
      DEBUG_SERIAL.println("Motor2 → -90");
      altState = ALT_M1_FORWARD;
      break;
    default:
      break;
    }
  }
}

// --- 共通コマンド処理 ---
void handleCommand(const String &cmd)
{
  if (cmd == "wave_forward")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturn(DXL_ID1, angleToValue(-90), 300, true);
      moveToAndReturn(DXL_ID2, angleToValue(+90), 300, true);
      moveToAndReturn(DXL_ID1, angleToValue(+90), 300, true);
      moveToAndReturn(DXL_ID2, angleToValue(-90), 300, true);
    }
  }
  else if (cmd == "wave_back")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturn(DXL_ID2, angleToValue(-90), 300, true);
      moveToAndReturn(DXL_ID1, angleToValue(+90), 300, true);
      moveToAndReturn(DXL_ID2, angleToValue(+90), 300, true);
      moveToAndReturn(DXL_ID1, angleToValue(-90), 300, true);
    }
  }
  else if (cmd == "wave_return")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturn(DXL_ID1, angleToValue(-90), 300, true);
      moveToAndReturn(DXL_ID2, angleToValue(+90), 300, true);
      moveToAndReturn(DXL_ID1, angleToValue(+90), 300, true);
      moveToAndReturn(DXL_ID2, angleToValue(-90), 300, true);
    }
    delay(1000);
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturn(DXL_ID2, angleToValue(-90), 300, true);
      moveToAndReturn(DXL_ID1, angleToValue(+90), 300, true);
      moveToAndReturn(DXL_ID2, angleToValue(+90), 300, true);
      moveToAndReturn(DXL_ID1, angleToValue(-90), 300, true);
    }
    DEBUG_SERIAL.println("Executed wave_return");
  }
  else if (cmd == "wave_large")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID1, -120, 300, true);
      moveToAndReturnDegrees(DXL_ID2, +120, 300, true);
      moveToAndReturnDegrees(DXL_ID1, +120, 300, true);
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
    DEBUG_SERIAL.println("Async alternating motion started");
  }
  else if (cmd == "stop_async")
  {
    altMotionActive = false;
    altState = ALT_IDLE;
    moveToPosition(DXL_ID1, neutralPosition);
    moveToPosition(DXL_ID2, neutralPosition);
    DEBUG_SERIAL.println("Async alternating motion stopped");
  }
  else if (cmd == "calibrate")
  {
    calibAll();
  }
  else if (cmd.startsWith("set_1_"))
  {
    float angle = cmd.substring(6).toFloat();
    moveToPositionDegrees(DXL_ID1, angle);
    DEBUG_SERIAL.print("Motor1 set to angle: ");
    DEBUG_SERIAL.println(angle);
  }
  else if (cmd.startsWith("set_2_"))
  {
    float angle = cmd.substring(6).toFloat();
    moveToPositionDegrees(DXL_ID2, angle);
    DEBUG_SERIAL.print("Motor2 set to angle: ");
    DEBUG_SERIAL.println(angle);
  }
}

// --- setup ---
void setup()
{
  DEBUG_SERIAL.begin(115200);
  DXL_SERIAL.begin(57600); // ← これで試す
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  setupDxl(DXL_ID1);
  setupDxl(DXL_ID2);
  calibAll();

  // BLEセットアップ
  if (!BLE.begin())
  {
    DEBUG_SERIAL.println("BLE init failed");
    while (1)
      ;
  }
  BLE.setLocalName("DynamixelCtrl");
  BLE.setAdvertisedService(controlService);
  controlService.addCharacteristic(commandChar);
  BLE.addService(controlService);
  commandChar.writeValue("");
  BLE.advertise();
  DEBUG_SERIAL.println("BLE Ready");
}

// --- loop ---
void loop()
{
  updateAltMotion();

  char val = Serial.read();
  if (val > 0)
  {
    String cmd(1, val);
    handleCommand(cmd);
  }

  BLEDevice central = BLE.central();
  if (central)
  {
    DEBUG_SERIAL.print("Connected to: ");
    DEBUG_SERIAL.println(central.address());

    while (central.connected())
    {
      updateAltMotion();
      if (commandChar.written())
      {
        String cmd = String((const char *)commandChar.value(), commandChar.valueLength());
        DEBUG_SERIAL.print("BLE受信: ");
        DEBUG_SERIAL.println(cmd);
        handleCommand(cmd);
      }
    }

    DEBUG_SERIAL.println("Disconnected");
  }
}