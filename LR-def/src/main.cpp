#include <Arduino.h>
#include <DynamixelShield.h>
#include "motion_data.h" // Pythonで生成されたファイルをインポート

#define DXL_PROTOCOL_VERSION 2.0
#define TIMEOUT 10
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
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, POSITION_CONTROL_MODE);
  dxl.torqueOn(id);
}

void setup()
{
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  setupDxl(DXL_ID1);
  setupDxl(DXL_ID2);
  setupDxl(DXL_ID3);
  setupDxl(DXL_ID4);

  // 初期位置へ移動
  dxl.setGoalPosition(DXL_ID1, 2048);
  dxl.setGoalPosition(DXL_ID2, 2048);
  dxl.setGoalPosition(DXL_ID3, 2048);
  dxl.setGoalPosition(DXL_ID4, 2048);
  delay(2000);
}

void loop()
{
  for (int s = 0; s < TOTAL_STEPS; s++)
  {
    // データが既に 0step 処理されているため、そのまま送信
    dxl.setGoalPosition(DXL_ID1, MOTION_DATA[s][0]);
    dxl.setGoalPosition(DXL_ID2, MOTION_DATA[s][1]);
    dxl.setGoalPosition(DXL_ID3, MOTION_DATA[s][2]);
    dxl.setGoalPosition(DXL_ID4, MOTION_DATA[s][3]);

    delay(1); // 1step = 1ms
  }
  delay(5000);
}