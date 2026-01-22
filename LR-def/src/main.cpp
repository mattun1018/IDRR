#include <Arduino.h>
#include <DynamixelShield.h>
#include "motion_data.h" // Pythonで生成したファイルをインポート

// --- XL430 (Protocol 2.0) 専用定数 ---
#define DXL_PROTOCOL_VERSION 2.0
#define TIMEOUT 10
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define POSITION_CONTROL_MODE 3

const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const uint8_t DXL_ID3 = 3;
const uint8_t DXL_ID4 = 4;

DynamixelShield dxl;

void setupDxl(uint8_t id)
{
  uint8_t torque_off = 0, torque_on = 1;
  uint8_t mode = POSITION_CONTROL_MODE;

  // 初期設定時にはトルクをオフにする
  dxl.write(id, ADDR_TORQUE_ENABLE, &torque_off, 1, TIMEOUT);
  // XL430を角度制御モード(3)に設定
  dxl.write(id, ADDR_OPERATING_MODE, &mode, 1, TIMEOUT);
  // トルクをオンにする
  dxl.write(id, ADDR_TORQUE_ENABLE, &torque_on, 1, TIMEOUT);
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("XL430 Control Starting...");

  // Dynamixel 通信初期化 (1Mbps)
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  setupDxl(DXL_ID1);
  setupDxl(DXL_ID2);
  setupDxl(DXL_ID3);
  setupDxl(DXL_ID4);

  // 最初は全モータをニュートラル位置(2048)へ移動
  dxl.setGoalPosition(DXL_ID1, 0);
  dxl.setGoalPosition(DXL_ID2, 0);
  dxl.setGoalPosition(DXL_ID3, 0);
  dxl.setGoalPosition(DXL_ID4, 0);

  delay(2000);
  Serial.println("Setup Complete. Starting Motion...");
}

void loop()
{
  // Pythonスクリプトで生成された TOTAL_STEPS 分だけ実行
  for (int s = 0; s < TOTAL_STEPS; s++)
  {
    // 4つのモータにCSVから変換された値を送信
    // この値は既に「範囲外なら0」というルールが適用されています
    dxl.setGoalPosition(DXL_ID1, MOTION_DATA[s][0]);
    dxl.setGoalPosition(DXL_ID2, MOTION_DATA[s][1]);
    dxl.setGoalPosition(DXL_ID3, MOTION_DATA[s][2]);
    dxl.setGoalPosition(DXL_ID4, MOTION_DATA[s][3]);

    // 1step = 1ms の再現を試みるが、XL430の通信速度に合わせて調整が必要な場合あり
    delay(1);
  }

  Serial.println("Motion Finished. Waiting 5 seconds...");
  delay(5000);
}