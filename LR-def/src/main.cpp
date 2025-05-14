#include <Arduino.h>
#include <DynamixelShield.h>

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

// --- DYNAMIXEL 定義 ---
#define DXL_PROTOCOL_VERSION 1.0
#define TIMEOUT 10

#define TORQUE_ENABLE_ADDR 24
#define GOAL_POSITION_ADDR 30
#define MOVING_SPEED_ADDR 32
#define CW_ANGLE_LIMIT_ADDR 6
#define CCW_ANGLE_LIMIT_ADDR 8

#define ADDR_LEN_1B 1
#define ADDR_LEN_2B 2

const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;

const uint16_t calibSpeed = 1023;
const uint16_t angleLimitMin = 0;
const uint16_t angleLimitMax = 1023;
const uint16_t neutralPosition = 517;
const uint16_t calibPositions[] = {517, 517};

DynamixelShield dxl;

// --- 変換関数 ---
uint16_t angleToValue(float degree)
{
  return constrain(map(degree, -150, 150, 0, 1023), 0, 1023);
}

// --- 初期設定 ---
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

void moveToPosition(uint8_t id, uint16_t position)
{
  dxl.write(id, GOAL_POSITION_ADDR, (uint8_t *)&position, ADDR_LEN_2B, TIMEOUT);
}

void moveToPositionDegrees(uint8_t id, float degree)
{
  moveToPosition(id, angleToValue(degree));
}

void moveToAndReturn(uint8_t id, uint16_t target, int wait_ms, bool returnToNeutral)
{
  moveToPosition(id, target);
  delay(wait_ms);
  if (returnToNeutral)
  {
    moveToPosition(id, neutralPosition);
  }
}

void moveToAndReturnDegrees(uint8_t id, float targetDegree, int wait_ms, bool returnToNeutral)
{
  moveToAndReturn(id, angleToValue(targetDegree), wait_ms, returnToNeutral);
}

void calibAll()
{
  moveToPosition(DXL_ID1, calibPositions[0]);
  moveToPosition(DXL_ID2, calibPositions[1]);
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

// --- Arduino setup ---
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

// --- loop ---
void loop()
{
  updateAltMotion(); // 非同期交互動作の更新

  char val = Serial.read();

  if (val == '1')
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturn(DXL_ID1, angleToValue(-90), 300, true);
      moveToAndReturn(DXL_ID2, angleToValue(+90), 300, true);
      moveToAndReturn(DXL_ID1, angleToValue(+90), 300, true);
      moveToAndReturn(DXL_ID2, angleToValue(-90), 300, true);
    }
  }

  if (val == '2')
  {
    for (int i = 0; i < 2; i++)
    {
      moveToAndReturnDegrees(DXL_ID1, -120, 1000, true);
      moveToAndReturnDegrees(DXL_ID2, +120, 300, true);
      moveToAndReturnDegrees(DXL_ID1, +120, 1000, true);
      moveToAndReturnDegrees(DXL_ID2, -120, 300, true);
    }
  }

  if (val == '4')
  {
    for (int i = 0; i < 5; i++)
    {
      float angle1 = random(-120, 121);
      float angle2 = random(-120, 121);
      int delay1 = random(200, 800);
      int delay2 = random(200, 800);

      moveToAndReturnDegrees(DXL_ID1, angle1, delay1, false);
      moveToAndReturnDegrees(DXL_ID2, angle2, delay2, true);
    }
  }

  if (val == '5')
  {
    moveToAndReturnDegrees(DXL_ID1, -90, 2000, false);
    moveToAndReturnDegrees(DXL_ID2, 90, 1000, true);
  }

  if (val == '6')
  {
    altMotionActive = true;
    altState = ALT_M1_FORWARD;
    altStartTime = millis();
    DEBUG_SERIAL.println("Async alternating motion started");
  }
  if (val == '7')
  {
    altMotionActive = false;
    altState = ALT_IDLE;
    moveToPosition(DXL_ID1, neutralPosition);
    moveToPosition(DXL_ID2, neutralPosition);
    DEBUG_SERIAL.println("Async alternating motion stopped");
  }
  if (val == '9')
  {
    calibAll();
  }
}