#include <Arduino.h>
#include <DynamixelShield.h>

// ------- シリアル設定 --------
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8);
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif

// ------- DYNAMIXEL定義 --------
#define DXL_PROTOCOL_VERSION 1.0
#define TIMEOUT 10

#define TORQUE_ENABLE_ADDR 24
#define GOAL_POSITION_ADDR 30
#define PRESENT_POSITION_ADDR 36
#define MOVING_SPEED_ADDR 32
#define CW_ANGLE_LIMIT_ADDR 6
#define CCW_ANGLE_LIMIT_ADDR 8

#define ADDR_LEN_1B 1
#define ADDR_LEN_2B 2

const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;

// ------- 初期設定 --------
const uint16_t calibSpeed = 1023;
const uint16_t angleLimitMin = 0;
const uint16_t angleLimitMax = 1023;
const uint16_t neutralPosition = 517;

// IDごとのキャリブレーション位置
const uint16_t calibPositions[] = {517, 517};

DynamixelShield dxl;

// ------- 変換関数 --------
uint16_t angleToValue(float degree)
{
  return constrain(map(degree, -150, 150, 0, 1023), 0, 1023);
}

// ------- 初期設定 --------
void setupDxl(uint8_t id)
{
  uint8_t torque_off = 0;
  uint8_t torque_on = 1;

  dxl.write(id, TORQUE_ENABLE_ADDR, &torque_off, ADDR_LEN_1B, TIMEOUT);
  dxl.write(id, CW_ANGLE_LIMIT_ADDR, (uint8_t *)&angleLimitMin, ADDR_LEN_2B, TIMEOUT);
  dxl.write(id, CCW_ANGLE_LIMIT_ADDR, (uint8_t *)&angleLimitMax, ADDR_LEN_2B, TIMEOUT);
  dxl.write(id, MOVING_SPEED_ADDR, (uint8_t *)&calibSpeed, ADDR_LEN_2B, TIMEOUT);
  dxl.write(id, TORQUE_ENABLE_ADDR, &torque_on, ADDR_LEN_1B, TIMEOUT);
}

// ------- 制御 --------
void moveToPosition(uint8_t id, uint16_t position)
{
  dxl.write(id, GOAL_POSITION_ADDR, (uint8_t *)&position, ADDR_LEN_2B, TIMEOUT);
}

void moveToPositionDegrees(uint8_t id, float degree)
{
  moveToPosition(id, angleToValue(degree));
}

void moveToAndReturn(uint8_t id, uint16_t target, int wait_ms)
{
  moveToPosition(id, target);
  delay(wait_ms);
  moveToPosition(id, neutralPosition);
}

void moveToAndReturnDegrees(uint8_t id, float targetDegree, int wait_ms)
{
  moveToAndReturn(id, angleToValue(targetDegree), wait_ms);
}

void calibAll()
{
  moveToPosition(DXL_ID1, calibPositions[0]);
  moveToPosition(DXL_ID2, calibPositions[1]);
}

// ------- Arduino初期化 --------
void setup()
{
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL)
    ;

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  setupDxl(DXL_ID1);
  setupDxl(DXL_ID2);

  calibAll();
}

// ------- メインループ --------
void loop()
{
  char val = Serial.read();

  if (val == '1')
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturn(DXL_ID1, angleToValue(-90), 300);
      moveToAndReturn(DXL_ID2, angleToValue(+90), 300);
      moveToAndReturn(DXL_ID1, angleToValue(+90), 300);
      moveToAndReturn(DXL_ID2, angleToValue(-90), 300);
    }
  }

  if (val == '2')
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID1, -120, 1000);
      moveToAndReturnDegrees(DXL_ID2, +120, 300);
      moveToAndReturnDegrees(DXL_ID1, +120, 1000);
      moveToAndReturnDegrees(DXL_ID2, -120, 300);
    }
  }

  if (val == '9')
  {
    calibAll();
  }
}