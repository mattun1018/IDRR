#include <Arduino.h>
#include <DynamixelShield.h>
#include <ArduinoBLE.h>
#include "motion_data.h" // Pythonで作成したヘッダーファイル

// --- シリアル設定 ---
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8);
#define DEBUG_SERIAL soft_serial
#else
#define DEBUG_SERIAL Serial
#endif

// --- DYNAMIXEL XL430 (Protocol 2.0) 定数 ---
#define DXL_PROTOCOL_VERSION 2.0
#define TIMEOUT 10
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define POSITION_CONTROL_MODE 3

const uint8_t DXL_IDS[] = {1, 2, 3, 4}; // モータID: 1(左上-左下), 2(左下-右下), 3(右下-右上), 4(右上-左上)
const uint32_t DXL_BAUD = 1000000;

DynamixelShield dxl;

// BLE定義
BLEService controlService("180C");
BLECharacteristic commandChar("2A56", BLEWrite, 20);

// --- 補助関数 ---
void setupDxl(uint8_t id)
{
  uint8_t torque_off = 0, torque_on = 1;
  uint8_t mode = POSITION_CONTROL_MODE;

  dxl.write(id, ADDR_TORQUE_ENABLE, &torque_off, 1, TIMEOUT);
  dxl.write(id, ADDR_OPERATING_MODE, &mode, 1, TIMEOUT);
  dxl.write(id, ADDR_TORQUE_ENABLE, &torque_on, 1, TIMEOUT);
}

void calibAll()
{
  DEBUG_SERIAL.println("Resetting all motors to 0...");
  for (int i = 0; i < 4; i++)
  {
    dxl.setGoalPosition(DXL_IDS[i], 0);
  }
}

// CSVモーション再生関数
void playCsvMotion()
{
  DEBUG_SERIAL.print("Starting motion execution: ");
  DEBUG_SERIAL.print(TOTAL_STEPS);
  DEBUG_SERIAL.println(" steps.");

  for (int s = 0; s < TOTAL_STEPS; s++)
  {
    // 4基のモータにCSVからの計算値を送信
    dxl.setGoalPosition(DXL_IDS[0], MOTION_DATA[s][0]);
    dxl.setGoalPosition(DXL_IDS[1], MOTION_DATA[s][1]);
    dxl.setGoalPosition(DXL_IDS[2], MOTION_DATA[s][2]);
    dxl.setGoalPosition(DXL_IDS[3], MOTION_DATA[s][3]);

    // シミュレーションステップ 1ms に同期
    delay(1);
  }
  DEBUG_SERIAL.println("Motion complete.");
}

// --- コマンド処理 ---
void handleCommand(const String &cmd)
{
  if (cmd == "sc")
  { // BLEやシリアルで "sc" と打つと実行
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
    DEBUG_SERIAL.println("Torque Off.");
  }
  else if (cmd == "on")
  {
    for (int i = 0; i < 4; i++)
      dxl.torqueOn(DXL_IDS[i]);
    DEBUG_SERIAL.println("Torque On.");
  }
}

// --- setup ---
void setup()
{
  DEBUG_SERIAL.begin(115200);
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i < 4; i++)
    setupDxl(DXL_IDS[i]);
  calibAll();

  // BLEセットアップ
  if (!BLE.begin())
  {
    DEBUG_SERIAL.println("BLE init failed");
    while (1)
      ;
  }
  BLE.setLocalName("XL430_Motion_Ctrl");
  BLE.setAdvertisedService(controlService);
  controlService.addCharacteristic(commandChar);
  BLE.addService(controlService);
  commandChar.writeValue("");
  BLE.advertise();

  DEBUG_SERIAL.println("System Ready. Send 'sc' via BLE/Serial to start.");

  // --- quick sanity test: move each motor a small amount to verify they respond ---
  auto sanityTest = [&]()
  {
    DEBUG_SERIAL.println("Running sanity test: small moves on all motors");
    const int center = 2048;
    const int offset = 300; // small movement

    // Move to center+offset
    dxl.setGoalPosition(DXL_IDS[0], center + offset);
    dxl.setGoalPosition(DXL_IDS[1], center - offset);
    dxl.setGoalPosition(DXL_IDS[2], center + offset);
    dxl.setGoalPosition(DXL_IDS[3], center - offset);
    delay(1000);

    // Move back to center
    dxl.setGoalPosition(DXL_IDS[0], center);
    dxl.setGoalPosition(DXL_IDS[1], center);
    dxl.setGoalPosition(DXL_IDS[2], center);
    dxl.setGoalPosition(DXL_IDS[3], center);
    delay(1000);

    DEBUG_SERIAL.println("Sanity test done");
  };

  // Run the quick sanity test once at startup
  sanityTest();
}

// --- loop ---
void loop()
{
  // Serial入力
  if (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }

  // BLE入力
  BLEDevice central = BLE.central();
  if (central)
  {
    DEBUG_SERIAL.print("Connected to: ");
    DEBUG_SERIAL.println(central.address());

    while (central.connected())
    {
      if (commandChar.written())
      {
        String cmd = String((const char *)commandChar.value(), commandChar.valueLength());
        cmd.trim();
        DEBUG_SERIAL.print("BLE Command: ");
        DEBUG_SERIAL.println(cmd);
        handleCommand(cmd);
      }
    }
    DEBUG_SERIAL.println("Disconnected - Advertising...");
    BLE.advertise();
  }
}