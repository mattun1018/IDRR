#include <Arduino.h>
#include <Dynamixel2Arduino.h>

// --- 基本設定 ---
// 使用するシリアルポートと方向制御ピン
#define DXL_SERIAL Serial2
const uint8_t DXL_DIR_PIN = 4;

// テストするモーターのID
const uint8_t DXL_ID = 1;

// Dynamixel2Arduinoのオブジェクトを生成
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup()
{
  // デバッグ用シリアルの開始
  Serial.begin(115200);
  while (!Serial)
    ; // シリアルモニタが開くまで待機

  Serial.println("AX-12A 最小動作テストを開始します。");

  // --- モーターとの通信テスト ---
  // まずはボーレート 1,000,000 bps で試す
  Serial.println("ボーレート 1000000 bps で初期化を試みます...");
  dxl.begin(1000000);
  // プロトコルバージョンを1.0に設定 (AXシリーズで必須) - 定数名を修正
  dxl.setPortProtocolVersion(1.0);

  // pingを送信してモーターの応答を確認
  if (dxl.ping(DXL_ID))
  {
    Serial.println("モーターとの通信に成功しました！ (Baud: 1000000)");
  }
  else
  {
    Serial.println("ボーレート 1000000 bps では応答がありません。");
    Serial.println("ボーレート 57600 bps で再試行します...");

    // ボーレート 57,600 bps で再試行
    dxl.begin(57600);
    // プロトコルバージョンを1.0に設定 (AXシリーズで必須) - 定数名を修正
    dxl.setPortProtocolVersion(1.0);

    if (dxl.ping(DXL_ID))
    {
      Serial.println("モーターとの通信に成功しました！ (Baud: 57600)");
    }
    else
    {
      Serial.println("-------------------------------------------");
      Serial.println("【致命的なエラー】");
      Serial.println("どのボーレートでもモーターが応答しません。");
      Serial.println("ハードウェア（配線・電源・共通GND）を再確認してください。");
      Serial.println("-------------------------------------------");
      while (1)
        ; // ここで処理を停止
    }
  }

  // --- 動作準備 ---
  // 関節モードに設定
  if (dxl.setOperatingMode(DXL_ID, OP_POSITION))
  {
    Serial.println("関節モードへの設定に成功しました。");
  }
  else
  {
    Serial.println("関節モードへの設定に失敗しました。");
    while (1)
      ;
  }

  // トルクをオンにする
  if (dxl.torqueOn(DXL_ID))
  {
    Serial.println("トルクをオンにしました。モーターが動きます。");
  }
  else
  {
    Serial.println("トルクの有効化に失敗しました。");
    while (1)
      ;
  }
}

void loop()
{
  Serial.println("位置 0 へ移動します。");
  // AX-12Aの最小位置(0)へ移動
  dxl.setGoalPosition(DXL_ID, 0, UNIT_RAW);
  delay(1500); // 1.5秒待機

  Serial.println("位置 1023 へ移動します。");
  // AX-12Aの最大位置(1023)へ移動
  dxl.setGoalPosition(DXL_ID, 1023, UNIT_RAW);
  delay(1500); // 1.5秒待機
}
