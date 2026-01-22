#include <Arduino.h>
#include <DynamixelShield.h>
#include "motion_data.h" // Pythonで生成したヘッダーファイル

// --- デバッグシリアル設定 ---
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8);
#define DEBUG_SERIAL soft_serial
#else
#define DEBUG_SERIAL Serial
#endif

DynamixelShield dxl;
using namespace ControlTableItem;

// --- DYNAMIXEL定義 ---
#define DXL_PROTOCOL_VERSION 2.0
const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const uint8_t DXL_ID3 = 3;
const uint8_t DXL_ID4 = 4;
const uint8_t DXL_IDS[] = {DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4};

// (BLE removed) Serial-only control

// --- セットアップ関数 ---
void setupDxl(uint8_t id)
{
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION); // プロトコル2.0の角度制御モード
  dxl.torqueOn(id);
}

// no BLE functions: using Serial-only commands

// 全モータを初期位置(0)へ戻す
void calibAll()
{
  DEBUG_SERIAL.println("Returning to origin (0)...");
  for (int i = 0; i < 4; i++)
  {
    dxl.setGoalPosition(DXL_IDS[i], 0, UNIT_RAW);
  }
}

// --- CSVモーション再生機能 ---
void playCsvMotion()
{
  DEBUG_SERIAL.print("Executing CSV Motion: ");
  DEBUG_SERIAL.print(TOTAL_STEPS);
  DEBUG_SERIAL.println(" steps.");

  for (int s = 0; s < TOTAL_STEPS; s++)
  {
    // 4基同時に目標位置(Step値)を送信
    dxl.setGoalPosition(DXL_ID1, MOTION_DATA[s][0], UNIT_RAW);
    dxl.setGoalPosition(DXL_ID2, MOTION_DATA[s][1], UNIT_RAW);
    dxl.setGoalPosition(DXL_ID3, MOTION_DATA[s][2], UNIT_RAW);
    dxl.setGoalPosition(DXL_ID4, MOTION_DATA[s][3], UNIT_RAW);

    // シミュレーションのdt=1msに同期
    delay(1);
  }
  DEBUG_SERIAL.println("Motion Finished.");
}

// --- コマンド処理 ---
void handleCommand(const String &cmd)
{
  if (cmd == "sc")
  {
    playCsvMotion();
  }
  else if (cmd == "c" || cmd == "calibrate")
  {
    calibAll();
  }
  else if (cmd == "off")
  {
    for (int i = 0; i < 4; i++)
      dxl.torqueOff(DXL_IDS[i]);
    DEBUG_SERIAL.println("Torque Off");
  }
  else if (cmd == "on")
  {
    for (int i = 0; i < 4; i++)
      dxl.torqueOn(DXL_IDS[i]);
    DEBUG_SERIAL.println("Torque On");
  }
  else if (cmd.startsWith("set_"))
  {
    // 例: "set_1_2048" -> ID1を2048に設定
    int first_ = cmd.indexOf('_');
    int second_ = cmd.indexOf('_', first_ + 1);
    int id = cmd.substring(first_ + 1, second_).toInt();
    int pos = cmd.substring(second_ + 1).toInt();
    if (id >= 1 && id <= 4)
      dxl.setGoalPosition(id, pos, UNIT_RAW);
  }
}

// --- setup ---
void setup()
{
  DEBUG_SERIAL.begin(115200);

  // Dynamixel初期化 (以前の 57600 から、XL430で一般的な 1000000 に調整)
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i < 4; i++)
    setupDxl(DXL_IDS[i]);

  calibAll(); // 初期位置 0 へ
  DEBUG_SERIAL.println("System Ready. Send 'sc' via Serial to start.");
}

// --- loop ---
void loop()
{
  // シリアル入力
  if (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }

  // idle
  delay(10);
}