#include <Arduino.h>
#include <DynamixelShield.h>
#include <ArduinoBLE.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8);
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif

DynamixelShield dxl;
using namespace ControlTableItem;

#define DXL_ID1 1
#define DXL_ID2 2

BLEService controlService("180C");
BLECharacteristic commandChar("2A56", BLEWrite, 20);

const float neutralDeg = 0.0;

// --- セットアップ関数 ---
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

// --- 非同期交互動作用 ---
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

// --- コマンド処理 ---
void handleCommand(const String &cmd)
{
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
    DEBUG_SERIAL.println("Async alternating motion started");
  }
  else if (cmd == "stop_async")
  {
    altMotionActive = false;
    altState = ALT_IDLE;
    calibAll();
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
  dxl.begin(115200); // もしくは 57600
  dxl.setPortProtocolVersion(2.0);
  setupDxl(DXL_ID1);
  setupDxl(DXL_ID2);
  calibAll();

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