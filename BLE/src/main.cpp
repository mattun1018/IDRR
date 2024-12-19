#include <Arduino.h>
#include <DynamixelShield.h>
#include <ArduinoBLE.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif

#define CW_ANGLE_LIMIT_ADDR 6
#define CCW_ANGLE_LIMIT_ADDR 8
#define ANGLE_LIMIT_ADDR_LEN 2
#define OPERATING_MODE_ADDR_LEN 2
#define TORQUE_ENABLE_ADDR 24
#define TORQUE_ENABLE_ADDR_LEN 1
#define LED_ADDR 25
#define LED_ADDR_LEN 1
#define GOAL_POSITION_ADDR 30
#define GOAL_POSITION_ADDR_LEN 2
#define PRESENT_POSITION_ADDR 36
#define PRESENT_POSITION_ADDR_LEN 2
#define MOVING_SPEED_ADDR 32
#define MOVING_SPEED_ADDR_LEN 2
#define TIMEOUT 10 // default communication timeout 10ms

#define SERVICE_UUID "99474AC9-9C46-435D-AAE7-2C4B7E017494"
#define CHARACTERISTIC_UUID "B7747AB7-CFE4-45A1-9175-6FCDBFF7834A"

const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const float DXL_PROTOCOL_VERSION = 1.0;
uint8_t turn_on = 1;
uint8_t turn_off = 0;
uint16_t calibSpeed = 1023;
uint16_t goalPositionLimitMin = 0;
uint16_t goalPositionLimitMax = 1023;
uint16_t calibPosition1 = 517;
uint16_t calibPosition2 = 517;
// DXL_ID1の初期位置  512 +90度 820 -90度 204DXL_ID2の初期位置 358 +90度 666 -90度 50

String receivedValue;

DynamixelShield dxl;

BLEService idrrService(SERVICE_UUID);
BLEStringCharacteristic idrrCharacteristic(CHARACTERISTIC_UUID, BLERead | BLEWrite, 20);

void dxlSetup(uint8_t id)
{
  // Turn off torque when configuring items in EEPROM area
  if (dxl.write(id, TORQUE_ENABLE_ADDR, (uint8_t *)&turn_off, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("DYNAMIXEL Torque off");
  else
    DEBUG_SERIAL.println("Error: Torque off failed");

  // Set to Joint Mode
  if (dxl.write(id, CW_ANGLE_LIMIT_ADDR, (uint8_t *)&goalPositionLimitMin, ANGLE_LIMIT_ADDR_LEN, TIMEOUT) && dxl.write(id, CCW_ANGLE_LIMIT_ADDR, (uint8_t *)&goalPositionLimitMax, ANGLE_LIMIT_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Set operating mode");
  else
    DEBUG_SERIAL.println("Error: Set operating mode failed");
  delay(200);

  if (dxl.write(id, MOVING_SPEED_ADDR, (uint8_t *)&calibSpeed, MOVING_SPEED_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Set moving speed");
  else
    DEBUG_SERIAL.println("Error: Set moving speed failed");
  delay(200);

  // Turn on torque
  if (dxl.write(id, TORQUE_ENABLE_ADDR, (uint8_t *)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Torque on");
  else
    DEBUG_SERIAL.println("Error: Torque on failed");
  delay(200);
}

void controlDxl(uint8_t id, uint16_t goalPosition1, uint16_t goalPosition2, uint16_t goalPosition3)
{

  dxl.write(id, MOVING_SPEED_ADDR, (uint8_t *)&calibSpeed, MOVING_SPEED_ADDR_LEN, TIMEOUT);
  DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print("Goal Position : ");
  DEBUG_SERIAL.println(goalPosition1);
  dxl.write(id, GOAL_POSITION_ADDR, (uint8_t *)&goalPosition1, GOAL_POSITION_ADDR_LEN, TIMEOUT);
  delay(300);

  // DEBUG_SERIAL.print("Goal Position : ");
  // DEBUG_SERIAL.println(goalPosition2);
  // dxl.write(id, GOAL_POSITION_ADDR, (uint8_t *)&goalPosition2, GOAL_POSITION_ADDR_LEN, TIMEOUT);
  // delay(1000);

  DEBUG_SERIAL.print("Goal Position : ");
  DEBUG_SERIAL.println(goalPosition3);
  dxl.write(id, GOAL_POSITION_ADDR, (uint8_t *)&goalPosition3, GOAL_POSITION_ADDR_LEN, TIMEOUT);
  delay(1);
}

void calibDxl(uint8_t id, uint16_t calibPosition)
{
  DEBUG_SERIAL.print("Calib Position : ");
  DEBUG_SERIAL.println(calibPosition);
  dxl.write(id, GOAL_POSITION_ADDR, (uint8_t *)&calibPosition, GOAL_POSITION_ADDR_LEN, TIMEOUT);
  delay(300);
}

void setup()
{
  DEBUG_SERIAL.begin(115200); // Set debugging port baudrate to 115200bps
  if (!BLE.begin())
  {
    Serial.println("Starting BLE failed!");
    while (1)
      ;
  }
  BLE.setLocalName("IDRR");
  BLE.setAdvertisedService(idrrService);

  idrrService.addCharacteristic(idrrCharacteristic);
  BLE.addService(idrrService);

  idrrCharacteristic.writeValue("Hello from idrr!");

  BLE.advertise();
  Serial.println("Waiting for a client connection...");

  while (!DEBUG_SERIAL)
    ; // Wait until the serial port for terminal is opened

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxlSetup(DXL_ID1);
  dxlSetup(DXL_ID2);
  calibDxl(DXL_ID1, calibPosition1);
  calibDxl(DXL_ID2, calibPosition2);
}

void loop()
{
  BLEDevice central = BLE.central();
  if (central)
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected())
    {
      if (idrrCharacteristic.written())
      {
        receivedValue = idrrCharacteristic.value();
        Serial.print("Received Value: ");
        Serial.println(receivedValue);

        if (receivedValue == "forward")
        {
          for (int i = 0; i < 2; i++)
          {
            controlDxl(DXL_ID2, 109, 925, 517);
            controlDxl(DXL_ID1, 925, 109, 517);
            controlDxl(DXL_ID2, 925, 109, 517);
            controlDxl(DXL_ID1, 109, 925, 517);
          }
        }
        else if (receivedValue == "cal")
        {
          calibDxl(DXL_ID1, calibPosition1);

          calibDxl(DXL_ID2, calibPosition2);
        }
      }
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
