#include <Arduino.h>
#include <DynamixelShield.h>

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
#define DXL_ID3 3
#define DXL_ID4 4
#define DXL_ID5 5

const float neutralDeg = 10.0;

// --- セットアップ関数 ---
void setupDxl(uint8_t id)
{
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
}

void moveToPositionDegrees(uint8_t id, float deg)
{
  // 角度範囲の制限（0～360度）
  if (deg < 0.0)
    deg = 0.0;
  if (deg > 360.0)
    deg = 360.0;

  // Dynamixel内部単位へ変換（0度が0、360度が4095）
  const float DEGREE_TO_UNIT = 11.377777; // ＝4095 / 360

  int pos = int(deg * DEGREE_TO_UNIT);

  // 安全に制限（オーバーフロー防止）
  pos = constrain(pos, 0, 4095);

  dxl.setGoalPosition(id, pos, UNIT_RAW);
}

// 現在位置を角度で取得
float getCurrentPositionDegrees(uint8_t id)
{
  int currentPos = dxl.getPresentPosition(id, UNIT_RAW);
  const float UNIT_TO_DEGREE = 360.0 / 4095.0;
  return currentPos * UNIT_TO_DEGREE;
}

// 最短経路で10度に移動（起動時専用）
void moveToNeutralShortestPath(uint8_t id)
{
  float currentDeg = getCurrentPositionDegrees(id);
  float targetDeg = neutralDeg; // 10.0度

  DEBUG_SERIAL.print("Motor");
  DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(" current position: ");
  DEBUG_SERIAL.println(currentDeg);

  // 既に10度付近にある場合は移動しない
  if (abs(currentDeg - targetDeg) < 5.0)
  {
    DEBUG_SERIAL.print("Already near ");
    DEBUG_SERIAL.print(targetDeg);
    DEBUG_SERIAL.println(" degrees, no movement needed");
    return;
  }

  // 10度への最短距離を計算
  float directDistance = abs(currentDeg - targetDeg);
  float wrapDistance = 360.0 - directDistance;

  DEBUG_SERIAL.print("Direct distance: ");
  DEBUG_SERIAL.print(directDistance);
  DEBUG_SERIAL.print(", Wrap distance: ");
  DEBUG_SERIAL.println(wrapDistance);

  // 最短距離で移動（常に直接10度に移動）
  DEBUG_SERIAL.print("Moving to ");
  DEBUG_SERIAL.print(targetDeg);
  DEBUG_SERIAL.print(" degrees via shortest path (distance: ");
  DEBUG_SERIAL.print(min(directDistance, wrapDistance));
  DEBUG_SERIAL.println(")");

  moveToPositionDegrees(id, targetDeg);
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

// 起動時専用キャリブレーション（最短経路で10度に移動）
void calibAllStartup()
{
  DEBUG_SERIAL.println("Starting calibration with shortest path to 10 degrees...");

  moveToNeutralShortestPath(DXL_ID1);
  delay(100);
  moveToNeutralShortestPath(DXL_ID2);
  delay(100);
  moveToNeutralShortestPath(DXL_ID3);
  delay(100);
  moveToNeutralShortestPath(DXL_ID4);
  delay(100);
  moveToNeutralShortestPath(DXL_ID5);

  DEBUG_SERIAL.println("Startup calibration completed");
} // --- 非同期交互動作用 ---
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
      moveToPositionDegrees(DXL_ID1, 90);
      DEBUG_SERIAL.println("Motor1 → 90");
      altState = ALT_M2_FORWARD;
      break;
    case ALT_M2_FORWARD:
      moveToPositionDegrees(DXL_ID2, 270);
      DEBUG_SERIAL.println("Motor2 → 270");
      altState = ALT_M1_BACK;
      break;
    case ALT_M1_BACK:
      moveToPositionDegrees(DXL_ID1, 270);
      DEBUG_SERIAL.println("Motor1 → 270");
      altState = ALT_M2_BACK;
      break;
    case ALT_M2_BACK:
      moveToPositionDegrees(DXL_ID2, 90);
      DEBUG_SERIAL.println("Motor2 → 90");
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
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
    }
  }
  else if (cmd == "wave_back")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
    }
  }
  else if (cmd == "wave_return")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
    }
    delay(1000);
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID2, 90, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 270, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 90, 300, true);
    }
  }
  else if (cmd == "wave_large")
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID1, 60, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 300, 300, true);
      moveToAndReturnDegrees(DXL_ID1, 300, 300, true);
      moveToAndReturnDegrees(DXL_ID2, 60, 300, true);
    }
  }
  else if (cmd == "wave_random")
  {
    for (int i = 0; i < 5; i++)
    {
      float a1 = random(60, 301);
      float a2 = random(60, 301);
      int d1 = random(200, 800);
      int d2 = random(200, 800);
      moveToAndReturnDegrees(DXL_ID1, a1, d1, false);
      moveToAndReturnDegrees(DXL_ID2, a2, d2, true);
    }
  }
  else if (cmd == "wave_left")
  {
    moveToAndReturnDegrees(DXL_ID1, 90, 2000, false);
    moveToAndReturnDegrees(DXL_ID2, 270, 1000, true);
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
  else
  {
    // "ID_角度" 形式の処理（例: "1_360", "3_90"）
    int underscorePos = cmd.indexOf('_');
    if (underscorePos > 0)
    {
      String idStr = cmd.substring(0, underscorePos);
      String angleStr = cmd.substring(underscorePos + 1);
      int id = idStr.toInt();
      float angle = angleStr.toFloat();

      if (id >= 1 && id <= 5)
      {
        moveToPositionDegrees(id, angle);
        DEBUG_SERIAL.print("Motor");
        DEBUG_SERIAL.print(id);
        DEBUG_SERIAL.print(" set to angle: ");
        DEBUG_SERIAL.println(angle);
      }
      else
      {
        DEBUG_SERIAL.print("Invalid motor ID: ");
        DEBUG_SERIAL.println(id);
      }
    }
  }
}

// --- setup ---
void setup()
{
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600); // もしくは 115200
  dxl.setPortProtocolVersion(2.0);
  setupDxl(DXL_ID1);
  setupDxl(DXL_ID2);
  setupDxl(DXL_ID3);
  setupDxl(DXL_ID4);
  setupDxl(DXL_ID5);
  calibAllStartup();

  DEBUG_SERIAL.println("System Ready - Serial control");
  DEBUG_SERIAL.println("Available commands:");
  DEBUG_SERIAL.println("  wave_forward, wave_back, wave_return, wave_large");
  DEBUG_SERIAL.println("  wave_random, wave_left, start_async, stop_async");
  DEBUG_SERIAL.println("  calibrate, set_1_90, 1_180 (ID_angle format)");
}

// --- loop ---
void loop()
{
  updateAltMotion();

  // シリアル入力処理（改行終端）
  if (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0)
    {
      DEBUG_SERIAL.print("Command received: ");
      DEBUG_SERIAL.println(cmd);
      handleCommand(cmd);
    }
  }

  delay(10);
}