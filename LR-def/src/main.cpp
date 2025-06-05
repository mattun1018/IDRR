#include <Dynamixel2Arduino.h>

// サーボに送る信号についての初期設定
#define OPERATING_MODE_ADDR 11
#define OPERATING_MODE_ADDR_LEN 1
#define TORQUE_ENABLE_ADDR 64
#define TORQUE_ENABLE_ADDR_LEN 1
#define LED_ADDR 65
#define LED_ADDR_LEN 1
#define GOAL_POSITION_ADDR 116
#define GOAL_POSITION_ADDR_LEN 4
#define PRESENT_POSITION_ADDR 132
#define PRESENT_POSITION_ADDR_LEN 4
#define POSITION_CONTROL_MODE 3 // 位置の指示による制御モード
#define TIMEOUT 10              // サーボ信号のタイムアウト. デフォルトは10ms

// その他の初期設定
#define DXL_SERIAL Serial2              // 使うシリアル系統
const uint8_t DXL_DIR_PIN = 32;         // 半二重回路用のENピン
uint32_t centerPosition = 2048;         // サーボ位置のセンター値
uint32_t goalPosition = 0;              // サーボの目標位置
float radiansval = 0.0;                 // サインカーブ算出用のラジアン値
float radiansIncrement = 0.06;          // ループ毎のラジアン値の増加量
float maxPosition = 2000;               // センター値を2048としたとき、±どこまで振るか(0-2048)
const uint8_t DXL_ID = 1;               // サーボID
const float DXL_PROTOCOL_VERSION = 2.0; // プロトコルのバージョン
uint8_t turn_on = 1;
uint8_t turn_off = 0;
uint8_t operatingMode = POSITION_CONTROL_MODE;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // Dynamixel用ライブラリのインスタンス化

void led_on()
{ // LEDをオン
  dxl.write(DXL_ID, LED_ADDR, (uint8_t *)&turn_on, LED_ADDR_LEN, TIMEOUT);
}

void led_off()
{ // LEDをオフ
  dxl.write(DXL_ID, LED_ADDR, (uint8_t *)&turn_off, LED_ADDR_LEN, TIMEOUT);
}

void setup()
{
  dxl.begin(57600); // デフォルトのbaudrate. 必要に応じてサーボの設定にあわせる.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // サーボの初期設定時はトルクオフ
  if (dxl.write(DXL_ID, TORQUE_ENABLE_ADDR, (uint8_t *)&turn_off, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    led_on(); // 成功ならLEDをオン
  else
    led_off(); // 失敗ならLEDをオフ

  // サーボのOperating Modeを設定する
  if (dxl.write(DXL_ID, OPERATING_MODE_ADDR, (uint8_t *)&operatingMode, OPERATING_MODE_ADDR_LEN, TIMEOUT))
    led_on(); // 成功ならLEDをオン
  else
    led_off(); // 失敗ならLEDをオフ

  // サーボをトルクオン
  if (dxl.write(DXL_ID, TORQUE_ENABLE_ADDR, (uint8_t *)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    led_on(); // 成功ならLEDをオン
  else
    led_off(); // 失敗ならLEDをオフ
  delay(100);
}

void loop()
{
  // サインカーブ用の値を算出
  radiansval += radiansIncrement;                      // ラジアン値を増加
  radiansval = (radiansval > 2 * PI) ? 0 : radiansval; // ラジアン値が2πを超えたら0にリセット

  // サーボの値をセットする
  goalPosition = centerPosition + int(sin(radiansval) * maxPosition);
  // サーボにコマンドを送信
  dxl.write(DXL_ID, GOAL_POSITION_ADDR, (uint8_t *)&goalPosition, GOAL_POSITION_ADDR_LEN, TIMEOUT);

  delay(20);
}
