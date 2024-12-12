#include <ArduinoBLE.h>

// サービスと特徴UUIDの設定
BLEService myService("180D");                                       // サービスUUID（例：心拍サービス）
BLECharacteristic myCharacteristic("2A37", BLEWrite | BLERead, 20); // 特徴UUID（例：心拍数）、最大サイズ20バイト

void setup()
{
  // シリアル通信の開始
  Serial.begin(9600);
  while (!Serial)
    ;

  // BLEの初期化
  if (!BLE.begin())
  {
    Serial.println("BLEの初期化に失敗しました。");
    while (1)
      ;
  }

  // BLEデバイス名の設定
  BLE.setDeviceName("MKR1010_BLE");

  // サービスと特徴の追加
  BLE.addService(myService);
  myService.addCharacteristic(myCharacteristic);

  // サービスの開始
  BLE.advertise();
  Serial.println("BLEデバイスが開始されました。");
}

void loop()
{
  // BLEの接続を確認
  BLEDevice central = BLE.central();

  if (central)
  {
    Serial.print("デバイスが接続されました: ");
    Serial.println(central.address());

    // 接続中にBLEクライアントからデータを受け取る
    while (central.connected())
    {
      if (myCharacteristic.written())
      {
        // 受け取ったデータをconst uint8_t配列として取得
        const uint8_t *receivedData = myCharacteristic.value();
        size_t len = myCharacteristic.valueLength();

        // 受け取ったデータを文字列に変換
        String receivedString = "";
        for (size_t i = 0; i < len; i++)
        {
          receivedString += (char)receivedData[i];
        }

        Serial.print("受信データ: ");
        Serial.println(receivedString); // シリアルモニタに表示

        // 受け取ったデータに応じて条件分岐
        if (receivedString == "1")
        {
          Serial.println("OK"); // 1を受け取った場合
        }
        else if (receivedString == "abc")
        {
          Serial.println("False"); // abcを受け取った場合
        }
        else
        {
          Serial.println("Unknown data received");
        }
      }
    }

    // 接続が切れると通知
    Serial.print("デバイスが切断されました: ");
    Serial.println(central.address());
  }
}
