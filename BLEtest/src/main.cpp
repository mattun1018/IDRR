#include <Arduino.h>
// #include <Servo.h>
#include <ArduinoBLE.h>

#define ESC_PIN 4 // MKR WiFi 1010の使用可能なピン番号
#define STOP_VOLUME 1000

int targetVolume = STOP_VOLUME;
String receivedValue;

// UUIDs for the BLE service and characteristic
#define SERVICE_UUID "99474AC9-9C46-435D-AAE7-2C4B7E017494"
#define CHARACTERISTIC_UUID "B7747AB7-CFE4-45A1-9175-6FCDBFF7834A"

// BLEオブジェクト
BLEService brushlessService(SERVICE_UUID);
BLEStringCharacteristic brushlessCharacteristic(CHARACTERISTIC_UUID, BLERead | BLEWrite, 20);

void setup()
{
  Serial.begin(115200);

  targetVolume = STOP_VOLUME;

  // BLE setup
  if (!BLE.begin())
  {
    Serial.println("Starting BLE failed!");
    while (1)
      ;
  }

  BLE.setLocalName("Brushless_sample");
  BLE.setAdvertisedService(brushlessService);

  brushlessService.addCharacteristic(brushlessCharacteristic);
  BLE.addService(brushlessService);

  brushlessCharacteristic.writeValue("Hello from Brushless_sample!");

  BLE.advertise();
  Serial.println("Waiting for a client connection...");
}

void loop()
{
  // BLEデバイスの接続を監視
  BLEDevice central = BLE.central();

  if (central)
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected())
    {
      if (brushlessCharacteristic.written())
      {
        receivedValue = brushlessCharacteristic.value();
        Serial.print("Received Value: ");
        Serial.println(receivedValue);

        if (receivedValue == "0")
        {
          targetVolume = 1000;
        }
        else if (receivedValue.length() > 0)
        {
          targetVolume = receivedValue.toInt();
          Serial.print("Updated targetVolume: ");
          Serial.println(targetVolume);
        }
      }
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
