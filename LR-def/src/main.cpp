#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial2
#define DXL_DIR_PIN 4

// ⚠️ TXとRXのピンはIO16/17を使ってください（WROVERでは安全）
#define DXL_TX_PIN 17
#define DXL_RX_PIN 16

const uint8_t DXL_ID = 1; // 必要に応じて2にも変えて試す

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // UART2開始
  DXL_SERIAL.begin(57600, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);
  dxl.begin();
  dxl.setPortProtocolVersion(2.0);

  Serial.println("Pinging Dynamixel...");

  if (dxl.ping(DXL_ID))
  {
    Serial.println("✅ DXL found!");
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    dxl.torqueOn(DXL_ID);
    dxl.setGoalPosition(DXL_ID, 2048, UNIT_RAW); // 中立位置
  }
  else
  {
    Serial.println("❌ DXL not found.");
  }
}

void loop()
{
}