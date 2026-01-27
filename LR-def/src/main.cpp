#include "motion_fc.h"
#include "motion_fd.h"
#include "motion_sc.h"
#include "motion_sd.h"
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <DynamixelShield.h>

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

// --- BLE設定 ---
// UUIDs generated for this specific service and characteristic
BLEService dxlService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic rxChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLEWrite,
                               32);

// --- セットアップ関数 ---
void setupDxl(uint8_t id) {
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION); // プロトコル2.0の角度制御モード
  dxl.torqueOn(id);
}

// no BLE functions: using Serial-only commands

// 全モータを初期位置(2048)へ戻す
void calibAll() {
  DEBUG_SERIAL.println("Returning to origin (2048)...");
  for (int i = 0; i < 4; i++) {
    dxl.setGoalPosition(DXL_IDS[i], 2048, UNIT_RAW);
  }
}

// --- CSVモーション再生機能 ---
void playCsvMotion(const uint16_t data[][4], int steps) {
  DEBUG_SERIAL.print("Executing Motion: ");
  DEBUG_SERIAL.print(steps);
  DEBUG_SERIAL.println(" steps.");

  for (int s = 0; s < steps; s++) {
    // 4基同時に目標位置(Step値)を送信
    dxl.setGoalPosition(DXL_ID1, data[s][0], UNIT_RAW);
    dxl.setGoalPosition(DXL_ID2, data[s][1], UNIT_RAW);
    dxl.setGoalPosition(DXL_ID3, data[s][2], UNIT_RAW);
    dxl.setGoalPosition(DXL_ID4, data[s][3], UNIT_RAW);

    // シミュレーションのdt=1msに同期
    delay(1);
  }
  DEBUG_SERIAL.println("Motion Finished.");
}

// --- コマンド処理 ---
void handleCommand(const String &cmd) {
  if (cmd == "fc") {
    playCsvMotion(MOTION_FC, STEPS_FC);
  } else if (cmd == "sc") {
    playCsvMotion(MOTION_SC, STEPS_SC);
  } else if (cmd == "fd") {
    playCsvMotion(MOTION_FD, STEPS_FD);
  } else if (cmd == "sd") {
    playCsvMotion(MOTION_SD, STEPS_SD);
  } else if (cmd == "c" || cmd == "calibrate") {
    calibAll();
  } else if (cmd == "off") {
    for (int i = 0; i < 4; i++)
      dxl.torqueOff(DXL_IDS[i]);
    DEBUG_SERIAL.println("Torque Off");
  } else if (cmd == "on") {
    for (int i = 0; i < 4; i++)
      dxl.torqueOn(DXL_IDS[i]);
    DEBUG_SERIAL.println("Torque On");
  } else if (cmd.startsWith("set_")) {
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
void setup() {
  DEBUG_SERIAL.begin(115200);

  // Dynamixel初期化 (以前の 57600 から、XL430で一般的な 1000000 に調整)
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i < 4; i++)
    setupDxl(DXL_IDS[i]);

  calibAll(); // 初期位置 0 へ

  // --- BLE初期化 ---
  if (!BLE.begin()) {
    DEBUG_SERIAL.println("starting BLE failed!");
    while (1)
      ;
  }

  BLE.setLocalName("DynamixelController");
  BLE.setAdvertisedService(dxlService);
  dxlService.addCharacteristic(rxChar);
  BLE.addService(dxlService);

  BLE.advertise();
  DEBUG_SERIAL.println("BLE Active. Waiting for connections...");
  DEBUG_SERIAL.println(
      "System Ready. Send 'fc', 'sc', 'fd', 'sd' via Serial or BLE to start.");
}

// --- loop ---
void loop() {
  // --- BLE処理 ---
  BLEDevice central = BLE.central();
  if (central) {
    // 接続されたら
    if (central.connected()) {
      if (rxChar.written()) {
        String bleCmd = rxChar.value();
        bleCmd.trim();
        DEBUG_SERIAL.print("BLE Command: ");
        DEBUG_SERIAL.println(bleCmd);
        handleCommand(bleCmd);
      }
    }
  }

  // シリアル入力
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }

  // idle
  delay(10);
}